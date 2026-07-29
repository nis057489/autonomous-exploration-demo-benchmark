#!/usr/bin/env bash
set -euo pipefail

# Applies (or clears) a token-bucket rate limit on the OpenWrt router's
# 2.4GHz AP interface, which is what the turtlebots associate to. This is a
# real link-layer bandwidth cap on the *entire* AP (SSH, ROS control traffic,
# everything) for testing under actual wifi constraint -- deliberately kept
# separate from BANDWIDTH_KBPS in experiment.conf, which is ddil_proxy_node's
# own software-side simulation of just the map-transport messages. Wiring
# both to the same number double-limits (the proxy tries to rate-limit on
# top of a link the router already crushed) and, worse, the router's cap
# also throttles the SSH session used to launch/control the robots. Pick one
# mechanism per run -- don't set both nonzero at once.
#
# Requires the router to have `tc` (iproute2-tc via opkg) installed, and
# additionally `kmod-ifb` for --both-directions.
#
# Usage:
#   ./set_wifi_bandwidth.sh apply <rate>[bps|kbps|mbps|gbps] [--both-directions] [--viz-kbps <n>] [--viz-ip <ip>]
#   ./set_wifi_bandwidth.sh apply --profile <name> [--viz-kbps <n>] [--viz-ip <ip>]
#   ./set_wifi_bandwidth.sh clear
#
# --profile looks <name> up in wifi_profiles.json (rate + both_directions +
# optional viz_kbps), alongside this script. Flags on the command line
# override whatever the profile specifies.
#
# --both-directions additionally polices ingress (robot -> router) via an
# IFB redirect; without it, only egress (router -> robot) is shaped, since a
# plain root qdisc can't touch traffic arriving on the interface.
#
# --viz-kbps reserves an EXTRA, separate lane on the ingress side for
# traffic addressed to --viz-ip (default: VIZ_LAPTOP_IP env var, or
# 192.168.100.20 -- same convention launch_real_hardware.sh uses), so a
# visualization laptop's own subscriptions (TF/scans/map) stop competing
# with whatever map-transport rate you're actually testing for the single
# shaped queue. Only meaningful together with --both-directions: a wired
# viz laptop's downlink never touches this interface at all, so there's
# nothing to carve out on the egress-only path. The map-transport rate
# itself is unaffected either way -- this lane is additive, not borrowed
# from <rate>. Needs htb + sfq qdisc support on the router in addition to
# tbf/ifb (usually already present, but see kmod-sched-* via opkg if not).
#
# Safe to re-run: apply/clear always tear down any existing shaping first,
# so changing the rate, direction, or viz lane is just running apply again.

ROUTER_USER="${ROUTER_USER:-root}"
ROUTER_IP="${ROUTER_IP:-192.168.100.1}"
ROUTER_SSH_KEY="${ROUTER_SSH_KEY:-}"
WIFI_IFACE="${WIFI_IFACE:-phy0-ap0}"
IFB_IFACE="ifb0"
VIZ_IP="${VIZ_IP:-${VIZ_LAPTOP_IP:-192.168.100.20}}"
VIZ_KBPS="${VIZ_KBPS:-0}"  # 0 = no reserved viz lane (default, unchanged behavior)

SSH_OPTS=(-o ConnectTimeout=10)
[[ -n "${ROUTER_SSH_KEY}" ]] && SSH_OPTS+=(-i "${ROUTER_SSH_KEY}")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILES_FILE="${SCRIPT_DIR}/wifi_profiles.json"

USAGE="Usage: $0 apply <rate>[bps|kbps|mbps|gbps] [--both-directions] [--viz-kbps <n>] [--viz-ip <ip>] | apply --profile <name> [--viz-kbps <n>] [--viz-ip <ip>] | clear"

ACTION="${1:-}"

if [[ "${ACTION}" != "apply" && "${ACTION}" != "clear" ]]; then
  echo "${USAGE}" >&2
  exit 1
fi

if [[ "${ACTION}" == "apply" ]]; then
  if [[ "${2:-}" == "--profile" ]]; then
    PROFILE_NAME="${3:-}"
    if [[ -z "${PROFILE_NAME}" ]]; then
      echo "${USAGE}" >&2
      exit 1
    fi
    if [[ ! -f "${PROFILES_FILE}" ]]; then
      echo "Profile file not found: ${PROFILES_FILE}" >&2
      exit 1
    fi
    if ! jq -e --arg p "${PROFILE_NAME}" '.[$p]' "${PROFILES_FILE}" >/dev/null 2>&1; then
      echo "Unknown profile '${PROFILE_NAME}'. Available: $(jq -r 'keys | join(", ")' "${PROFILES_FILE}")" >&2
      exit 1
    fi
    RATE_ARG="$(jq -r --arg p "${PROFILE_NAME}" '.[$p].rate' "${PROFILES_FILE}")"
    BOTH_DIRECTIONS=0
    [[ "$(jq -r --arg p "${PROFILE_NAME}" '.[$p].both_directions' "${PROFILES_FILE}")" == "true" ]] && BOTH_DIRECTIONS=1
    VIZ_KBPS="$(jq -r --arg p "${PROFILE_NAME}" '.[$p].viz_kbps // 0' "${PROFILES_FILE}")"
    EXTRA_ARGS=("${@:4}")
  else
    RATE_ARG="${2:-}"
    BOTH_DIRECTIONS=0
    EXTRA_ARGS=("${@:3}")
  fi

  # Scan any trailing flags (after either "--profile <name>" or "<rate>"):
  # --both-directions, --viz-kbps <n>, --viz-ip <ip>. These override
  # whatever a --profile lookup set above.
  while [[ ${#EXTRA_ARGS[@]} -gt 0 ]]; do
    case "${EXTRA_ARGS[0]}" in
      --both-directions)
        BOTH_DIRECTIONS=1
        EXTRA_ARGS=("${EXTRA_ARGS[@]:1}")
        ;;
      --viz-kbps)
        VIZ_KBPS="${EXTRA_ARGS[1]:-}"
        EXTRA_ARGS=("${EXTRA_ARGS[@]:2}")
        ;;
      --viz-ip)
        VIZ_IP="${EXTRA_ARGS[1]:-}"
        EXTRA_ARGS=("${EXTRA_ARGS[@]:2}")
        ;;
      *)
        echo "${USAGE}" >&2
        echo "Unrecognized argument: ${EXTRA_ARGS[0]}" >&2
        exit 1
        ;;
    esac
  done

  if ! [[ "${VIZ_KBPS}" =~ ^[0-9]+$ ]]; then
    echo "${USAGE}" >&2
    echo "--viz-kbps must be a non-negative integer (got '${VIZ_KBPS}')." >&2
    exit 1
  fi
  if [[ "${VIZ_KBPS}" -gt 0 ]] && [[ ${BOTH_DIRECTIONS} -eq 0 ]]; then
    echo "Note: --viz-kbps only has an effect together with --both-directions" >&2
    echo "(only the ingress/robot->router leg is shaped otherwise, so a wired" >&2
    echo "viz laptop's traffic never touches this queue to begin with)." >&2
  fi

  # Bare numbers default to kbps for backward compatibility. Everything is
  # normalized to kbit/s (RATE_KBPS) since that's the unit tc's tbf gets
  # invoked with below.
  if [[ "${RATE_ARG}" =~ ^([0-9]+(\.[0-9]+)?)$ ]]; then
    NUM="${BASH_REMATCH[1]}"
    UNIT="kbps"
  elif [[ "${RATE_ARG,,}" =~ ^([0-9]+(\.[0-9]+)?)(bps|kbps|mbps|gbps)$ ]]; then
    NUM="${BASH_REMATCH[1]}"
    UNIT="${BASH_REMATCH[3]}"
  else
    echo "${USAGE}" >&2
    echo "<rate> must be a positive number, optionally suffixed with bps/kbps/mbps/gbps -- use '$0 clear' to remove shaping." >&2
    exit 1
  fi

  case "${UNIT}" in
    bps)  DIVISOR=1000 ;;
    kbps) DIVISOR=1 ;;
    mbps) DIVISOR=0.001 ;;
    gbps) DIVISOR=0.000001 ;;
  esac
  RATE_KBPS="$(awk -v n="${NUM}" -v d="${DIVISOR}" 'BEGIN{printf "%d", n/d}')"

  if [[ "${RATE_KBPS}" -eq 0 ]]; then
    echo "${USAGE}" >&2
    echo "'${RATE_ARG}' rounds to 0 kbit/s -- use '$0 clear' to remove shaping instead." >&2
    exit 1
  fi
fi

remote() {
  ssh "${SSH_OPTS[@]}" "${ROUTER_USER}@${ROUTER_IP}" "$1"
}

# Errors redirected to /dev/null because most of these fail harmlessly on a
# clean router (e.g. "no qdisc to delete") -- the trailing `true` makes the
# overall block succeed regardless, so apply/clear stay idempotent.
clear_cmd="
tc qdisc del dev ${WIFI_IFACE} root 2>/dev/null
tc qdisc del dev ${WIFI_IFACE} ingress 2>/dev/null
tc qdisc del dev ${IFB_IFACE} root 2>/dev/null
ip link del ${IFB_IFACE} 2>/dev/null
true
"

if [[ "${ACTION}" == "clear" ]]; then
  echo "==> Clearing wifi shaping on ${WIFI_IFACE} (${ROUTER_IP})"
  remote "${clear_cmd}"
  echo "Done."
  exit 0
fi

# tbf burst must be at least ~1 MTU's worth to avoid stalling; rate/4 with an
# 8kbit floor keeps that comfortably true across the ranges this project
# actually tests (tens to hundreds of kbit/s). Same floor applies to the viz
# lane's own burst, sized off VIZ_KBPS instead.
BURST_KBIT=$(( RATE_KBPS / 4 ))
[[ ${BURST_KBIT} -lt 8 ]] && BURST_KBIT=8
VIZ_BURST_KBIT=$(( VIZ_KBPS / 4 ))
[[ ${VIZ_BURST_KBIT} -lt 8 ]] && VIZ_BURST_KBIT=8

apply_cmd="
${clear_cmd}
tc qdisc add dev ${WIFI_IFACE} root tbf rate ${RATE_KBPS}kbit burst ${BURST_KBIT}kbit latency 400ms
"

if [[ ${BOTH_DIRECTIONS} -eq 1 ]]; then
  apply_cmd+="
modprobe ifb
ip link add ${IFB_IFACE} type ifb
ip link set ${IFB_IFACE} up
tc qdisc add dev ${WIFI_IFACE} handle ffff: ingress
tc filter add dev ${WIFI_IFACE} parent ffff: protocol all u32 match u32 0 0 action mirred egress redirect dev ${IFB_IFACE}
"
  if [[ "${VIZ_KBPS}" -gt 0 ]]; then
    # Two hard-capped lanes on the ingress (robot -> router) side instead of
    # one plain tbf. Class 1:20 (the htb default) gets exactly RATE_KBPS --
    # the same worst-case rate a plain tbf would've given ALL traffic -- so
    # whatever map-transport rate you're testing is unaffected. Class 1:10
    # is a separate, additional lane for anything addressed to VIZ_IP (the
    # viz laptop), so RViz's own subscriptions stop competing with the
    # map-transport link for that single queue. rate == ceil on both classes
    # -- no borrowing between them -- so this can't quietly loosen the rate
    # you asked to test.
    apply_cmd+="
tc qdisc add dev ${IFB_IFACE} root handle 1: htb default 20
tc class add dev ${IFB_IFACE} parent 1: classid 1:10 htb rate ${VIZ_KBPS}kbit ceil ${VIZ_KBPS}kbit burst ${VIZ_BURST_KBIT}kbit
tc class add dev ${IFB_IFACE} parent 1: classid 1:20 htb rate ${RATE_KBPS}kbit ceil ${RATE_KBPS}kbit burst ${BURST_KBIT}kbit
tc qdisc add dev ${IFB_IFACE} parent 1:10 handle 10: sfq perturb 10
tc qdisc add dev ${IFB_IFACE} parent 1:20 handle 20: sfq perturb 10
tc filter add dev ${IFB_IFACE} parent 1: protocol ip u32 match ip dst ${VIZ_IP}/32 flowid 1:10
"
  else
    apply_cmd+="
tc qdisc add dev ${IFB_IFACE} root tbf rate ${RATE_KBPS}kbit burst ${BURST_KBIT}kbit latency 400ms
"
  fi
fi

DIR_DESC="router->robot only"
[[ ${BOTH_DIRECTIONS} -eq 1 ]] && DIR_DESC="both directions"
VIZ_DESC=""
[[ "${VIZ_KBPS}" -gt 0 ]] && [[ ${BOTH_DIRECTIONS} -eq 1 ]] && VIZ_DESC=", +${VIZ_KBPS}kbit reserved for ${VIZ_IP}"
echo "==> Limiting ${WIFI_IFACE} on ${ROUTER_IP} to ${RATE_KBPS}kbit (${DIR_DESC}${VIZ_DESC})"
remote "${apply_cmd}"
echo "Done."
