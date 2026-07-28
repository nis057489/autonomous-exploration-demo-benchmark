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
#   ./set_wifi_bandwidth.sh apply <rate>[bps|kbps|mbps|gbps] [--both-directions]
#   ./set_wifi_bandwidth.sh apply --profile <name>
#   ./set_wifi_bandwidth.sh clear
#
# --profile looks <name> up in wifi_profiles.json (rate + both_directions),
# alongside this script.
#
# --both-directions additionally polices ingress (robot -> router) via an
# IFB redirect; without it, only egress (router -> robot) is shaped, since a
# plain root qdisc can't touch traffic arriving on the interface.
#
# Safe to re-run: apply/clear always tear down any existing shaping first,
# so changing the rate or direction is just running apply again.

ROUTER_USER="${ROUTER_USER:-root}"
ROUTER_IP="${ROUTER_IP:-192.168.100.1}"
ROUTER_SSH_KEY="${ROUTER_SSH_KEY:-}"
WIFI_IFACE="${WIFI_IFACE:-phy0-ap0}"
IFB_IFACE="ifb0"

SSH_OPTS=(-o ConnectTimeout=10)
[[ -n "${ROUTER_SSH_KEY}" ]] && SSH_OPTS+=(-i "${ROUTER_SSH_KEY}")

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROFILES_FILE="${SCRIPT_DIR}/wifi_profiles.json"

USAGE="Usage: $0 apply <rate>[bps|kbps|mbps|gbps] [--both-directions] | apply --profile <name> | clear"

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
  else
    RATE_ARG="${2:-}"
    BOTH_DIRECTIONS=0
    [[ "${3:-}" == "--both-directions" ]] && BOTH_DIRECTIONS=1
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
# actually tests (tens to hundreds of kbit/s).
BURST_KBIT=$(( RATE_KBPS / 4 ))
[[ ${BURST_KBIT} -lt 8 ]] && BURST_KBIT=8

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
tc qdisc add dev ${IFB_IFACE} root tbf rate ${RATE_KBPS}kbit burst ${BURST_KBIT}kbit latency 400ms
"
fi

DIR_DESC="router->robot only"
[[ ${BOTH_DIRECTIONS} -eq 1 ]] && DIR_DESC="both directions"
echo "==> Limiting ${WIFI_IFACE} on ${ROUTER_IP} to ${RATE_KBPS}kbit (${DIR_DESC})"
remote "${apply_cmd}"
echo "Done."
