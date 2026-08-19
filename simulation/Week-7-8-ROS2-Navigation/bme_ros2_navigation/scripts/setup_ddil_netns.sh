#!/usr/bin/env bash
# Sets up (or tears down) a real, tc-shapeable network path per robot for the
# vxch/baseline DDIL uplink, so bandwidth/loss/delay can be enforced with actual
# `tc netem` -- the same tool used to throttle the wireless interface in the
# hardware test -- instead of (or alongside) ddil_proxy_node's in-process
# token-bucket simulation.
#
# Topology (mirrors "robot <-wifi-> router <-wifi-> robot" from the hardware rig):
#
#   [ns-robot1]--veth-robot1--[br-ddil, main netns]--veth-robot2--[ns-robot2]  ...
#
# br-ddil plays the role of the router. Each robot gets one veth pair, one end
# in its own netns (its "radio"), one end on the bridge (its attachment point
# at the router) -- tc netem is applied on BOTH ends of each pair, so a robot's
# own uplink and downlink are both shaped, matching a real wifi link's roughly
# symmetric degradation under distance/interference.
#
# Only the DDIL-relevant nodes (vxch_encoder_{robot}, ddil_proxy_{robot}_from_*)
# actually run inside a robot's netns (see multi_robot_vxch_experiment.launch.py's
# impairment_mode:=tc path) -- everything else (nav2, slam, physics) stays in the
# main netns, same as software-impairment mode, since real hardware doesn't
# throttle a robot's own onboard buses either, only the link between robots.
#
# Usage:
#   setup_ddil_netns.sh up <num_robots>
#   setup_ddil_netns.sh down <num_robots>
#   setup_ddil_netns.sh update <num_robots> <bandwidth_kbps> <loss_pct> <delay_ms>
#
# `up` always creates links unshaped -- run `update` (typically after a delay,
# via launch.sh's IMPAIRMENT_DELAY_S) once ROS2/DDS discovery across the netns
# boundary has had time to settle, since applying tight bandwidth/loss/delay
# limits before discovery completes can prevent it from ever completing.
#
# Requires CAP_NET_ADMIN (docker run --cap-add=NET_ADMIN) and iproute2 (`ip`, `tc`).

set -euo pipefail

BRIDGE="br-ddil"
SUBNET_PREFIX="10.77.0"
BRIDGE_IP="${SUBNET_PREFIX}.1"

robot_netns() { echo "ns-robot$1"; }
robot_veth_main() { echo "veth-r$1"; }
robot_veth_ns() { echo "veth-r$1-ns"; }
robot_ip() { echo "${SUBNET_PREFIX}.$((10 + $1))"; }

# Numeric-args may be floats (e.g. "12.5"), so plain bash [[ ]]/(( )) integer
# comparison isn't safe; python3 is guaranteed present (the whole stack is ROS2),
# unlike bc, so use it instead of adding another package dependency.
_gt_zero() { python3 -c "import sys; sys.exit(0 if float(sys.argv[1]) > 0 else 1)" "$1"; }
_all_zero() { python3 -c "
import sys
sys.exit(0 if all(float(v) == 0 for v in sys.argv[1:]) else 1)
" "$1" "$2" "$3"; }

# netem's `rate` needs a nonzero value (0 means "don't touch bandwidth", not
# "unlimited" -- omit the clause entirely for the unlimited/0 case instead).
netem_args() {
  local bandwidth_kbps="$1" loss_pct="$2" delay_ms="$3"
  local -a args=(netem)
  _gt_zero "${delay_ms}" && args+=(delay "${delay_ms}ms") || true
  _gt_zero "${loss_pct}" && args+=(loss "${loss_pct}%") || true
  _gt_zero "${bandwidth_kbps}" && args+=(rate "${bandwidth_kbps}kbit") || true
  echo "${args[@]}"
}

apply_tc() {
  local dev="$1" netns_prefix="$2" bandwidth_kbps="$3" loss_pct="$4" delay_ms="$5"
  local -a ns_exec=()
  if [[ -n "${netns_prefix}" ]]; then
    ns_exec=(ip netns exec "${netns_prefix}")
  fi
  "${ns_exec[@]}" tc qdisc del dev "${dev}" root 2>/dev/null || true
  # If all three are zero, leave the qdisc absent entirely (unshaped) rather
  # than installing a no-op netem -- cheaper and avoids `tc` erroring on an
  # empty args list.
  if _all_zero "${bandwidth_kbps}" "${loss_pct}" "${delay_ms}"; then
    return
  fi
  # shellcheck disable=SC2046
  "${ns_exec[@]}" tc qdisc add dev "${dev}" root $(netem_args "${bandwidth_kbps}" "${loss_pct}" "${delay_ms}")
}

up() {
  # Always creates links unshaped (no tc qdisc applied yet), regardless of the
  # target bandwidth/loss/delay -- letting ROS2/DDS discovery establish over an
  # unimpaired link first. Call `update` (optionally after a delay) once
  # discovery has had time to settle to actually apply the requested shaping.
  local num_robots="$1"

  if ! ip link show "${BRIDGE}" >/dev/null 2>&1; then
    ip link add name "${BRIDGE}" type bridge
    ip addr add "${BRIDGE_IP}/24" dev "${BRIDGE}"
    ip link set "${BRIDGE}" up
  fi

  for ((i = 1; i <= num_robots; i++)); do
    local ns veth_main veth_ns ip_addr
    ns="$(robot_netns "${i}")"
    veth_main="$(robot_veth_main "${i}")"
    veth_ns="$(robot_veth_ns "${i}")"
    ip_addr="$(robot_ip "${i}")"

    if ip netns list | grep -q "^${ns}\b"; then
      echo "[setup_ddil_netns] ${ns} already exists, skipping creation" >&2
      continue
    fi

    ip netns add "${ns}"
    ip link add "${veth_main}" type veth peer name "${veth_ns}"
    ip link set "${veth_ns}" netns "${ns}"

    ip link set "${veth_main}" master "${BRIDGE}"
    ip link set "${veth_main}" up

    ip netns exec "${ns}" ip addr add "${ip_addr}/24" dev "${veth_ns}"
    ip netns exec "${ns}" ip link set "${veth_ns}" up
    ip netns exec "${ns}" ip link set lo up
    ip netns exec "${ns}" ip route add default via "${BRIDGE_IP}"

    echo "[setup_ddil_netns] ${ns} up (unshaped): ${ip_addr}/24 <-> ${BRIDGE_IP}"
  done
}

down() {
  local num_robots="$1"
  for ((i = 1; i <= num_robots; i++)); do
    local ns
    ns="$(robot_netns "${i}")"
    ip netns delete "${ns}" 2>/dev/null || true
  done
  # veth-r{i} main-side ends are destroyed automatically when their peer's
  # netns is deleted; only the bridge itself needs explicit cleanup.
  ip link delete "${BRIDGE}" 2>/dev/null || true
  echo "[setup_ddil_netns] torn down (${num_robots} robot netns + ${BRIDGE})"
}

update() {
  local num_robots="$1" bandwidth_kbps="$2" loss_pct="$3" delay_ms="$4"
  for ((i = 1; i <= num_robots; i++)); do
    local ns veth_main veth_ns
    ns="$(robot_netns "${i}")"
    veth_main="$(robot_veth_main "${i}")"
    veth_ns="$(robot_veth_ns "${i}")"
    apply_tc "${veth_main}" "" "${bandwidth_kbps}" "${loss_pct}" "${delay_ms}"
    apply_tc "${veth_ns}" "${ns}" "${bandwidth_kbps}" "${loss_pct}" "${delay_ms}"
  done
  echo "[setup_ddil_netns] updated ${num_robots} robot links to ${bandwidth_kbps}kbps, ${loss_pct}% loss, ${delay_ms}ms delay"
}

cmd="${1:-}"
case "${cmd}" in
  up)
    up "${2:?num_robots required}"
    ;;
  down)
    down "${2:?num_robots required}"
    ;;
  update)
    update "${2:?num_robots required}" "${3:-0}" "${4:-0.0}" "${5:-0}"
    ;;
  *)
    echo "Usage: $0 {up|down|update} <num_robots> [bandwidth_kbps loss_pct delay_ms]" >&2
    exit 1
    ;;
esac
