#!/usr/bin/env bash
# Compares sim vs tc IMPAIRMENT_MODE bandwidth usage on robot1's DDIL link.
# Run once with IMPAIRMENT_MODE=sim, once with IMPAIRMENT_MODE=tc, same
# BANDWIDTH_KBPS/RANDOM_SEED/SPAWN_PRESET otherwise, each time as:
#
#   sudo ./capture_ddil_overhead.sh sim   (or: tc)
#
# Start this BEFORE launching the experiment (or right after IMPAIRMENT_DELAY_S
# for tc mode, so discovery settle traffic isn't counted against the shaped
# window), let it run for a fixed duration, then Ctrl+C or let it time out.

set -euo pipefail
MODE="${1:?usage: capture_ddil_overhead.sh <sim|tc>}"
DURATION_S="${2:-60}"
OUTDIR="/tmp/ddil_captures"
mkdir -p "${OUTDIR}"

# For sim mode there's no netns -- ddil_proxy_node's token bucket runs in the
# main netns, so capture on whatever interface actually carries robot1's
# traffic (loopback if it's all in one container/host, or the real NIC if
# robots are on separate hosts). For tc mode, capture on the netns veth.
if [[ "${MODE}" == "tc" ]]; then
    IFACE="veth-r1"
else
    IFACE="lo"   # adjust if robot1 isn't co-located with the capture host
fi

PCAP="${OUTDIR}/robot1_${MODE}_$(date +%s).pcap"
echo "[capture] mode=${MODE} iface=${IFACE} duration=${DURATION_S}s -> ${PCAP}"
timeout "${DURATION_S}" tcpdump -i "${IFACE}" -w "${PCAP}" -s 0 udp &
TCPDUMP_PID=$!
wait "${TCPDUMP_PID}" || true

echo
echo "=== Total bytes/packets ==="
tshark -r "${PCAP}" -q -z io,stat,0

echo
echo "=== Per-protocol byte breakdown (RTPS submessage types) ==="
tshark -r "${PCAP}" -Y rtps -T fields -e rtps.sm.id 2>/dev/null | sort | uniq -c | sort -rn

echo
echo "=== Bytes spent on DDS discovery (SPDP/SEDP, port 7400/7410ish) vs data ==="
tshark -r "${PCAP}" -q -z conv,udp

echo
echo "=== Average packet size / header overhead ==="
tshark -r "${PCAP}" -T fields -e frame.len -e data.len 2>/dev/null | \
  awk '{total+=$1; if($2!="") payload+=$2; n++} END {
    printf "packets=%d total_bytes=%d avg_frame=%.1f payload_bytes=%d header_overhead_bytes=%d (%.1f%%)\n",
      n, total, total/n, payload, total-payload, 100*(total-payload)/total
  }'

echo
echo "pcap saved: ${PCAP}"
echo "Open in Wireshark with the RTPS dissector for a full submessage-level breakdown:"
echo "  wireshark ${PCAP}"
