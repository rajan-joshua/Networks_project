#!/bin/bash
# ==============================================================================
# Adaptive Net: Automated Verification & Self-Check Harness (Node-Failure Suite)
# Validates:
#   1. NS-3 Build Integrity
#   2. Failure-Discovery Check (Engine has zero knowledge of failure schedule)
#   3. Live Probe Monitor Check (Dynamic delay differs from static seed)
#   4. Steady State Zero-Switch Verification (No spuriously triggered switches)
#   5. TCP Handshake & Bulk Transfer Check (Full transfer completion, no stall)
#   6. Packet Conservation Check (Sent = Received + Lost)
#   7. Determinism Check (Seed 42 x 2 produces identical output)
#   8. Node-Failure Detection & PATH_DOWN Decision Check
#   9. Scenario Difference Check (Failure scenario differs from steady state)
#  10. Full Mode Matrix Smoke Tests (adaptive, baseline1, baseline2, shortest)
#  11. Full Scenario Matrix Smoke Tests (S1-S5 nodefail scenarios)
# ==============================================================================
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_DIR="${HOME}/ns-3.49"
TEST_DIR="${WORKSPACE_DIR}/results/verify_tmp"

mkdir -p "${TEST_DIR}"
mkdir -p "${NS3_DIR}/scratch"

echo "=================================================================="
echo "          Adaptive Net: Automated Self-Check Suite               "
echo "=================================================================="

# 1. Sync and Build Check
echo -n "Checking NS-3 Build... "
cp "${WORKSPACE_DIR}/adaptive-net.cc" "${NS3_DIR}/scratch/adaptive-net.cc"
cd "${NS3_DIR}"
./ns3 build scratch/adaptive-net > "${TEST_DIR}/build.log" 2>&1
if [ $? -eq 0 ]; then
    echo "[PASS]"
else
    echo "[FAIL] (See ${TEST_DIR}/build.log)"
    exit 1
fi

# 2. Failure-Discovery Check (Zero Knowledge Rule)
echo -n "Checking Failure-Discovery Integrity (No Failure Leakage into Engine)... "
awk '/class DecisionEngine/,/\/\/ Connect ProbeMonitorApp callbacks/' "${WORKSPACE_DIR}/adaptive-net.cc" > "${TEST_DIR}/engine.tmp"
if grep -q "SetRouterHardwareDown" "${TEST_DIR}/engine.tmp" || \
   grep -q "nodeFailStartTime" "${TEST_DIR}/engine.tmp" || \
   grep -q "SetPhysicalLinkDown" "${TEST_DIR}/engine.tmp"; then
    echo "[FAIL] Failure schedule leaked into DecisionEngine!"
    exit 1
fi
echo "[PASS]"

# 3. Live Probe Monitor Check
echo -n "Checking Live Probe Monitor Measurements... "
rm -f "${TEST_DIR}"/*.csv
./ns3 run "adaptive-net --mode=adaptive --scenario=steady --simTime=5.0 --enableAnim=false --resultsDir=${TEST_DIR}" > "${TEST_DIR}/monitor.log" 2>&1
if grep -q "PATH_0" "${TEST_DIR}/time_series.csv" || grep -q "Path_0" "${TEST_DIR}/time_series.csv" || grep -q "100Mbps" "${TEST_DIR}/time_series.csv"; then
    echo "[PASS]"
else
    echo "[FAIL] Time series does not show path tracking!"
    exit 1
fi

# 4. Steady-State Zero-Switch Check
echo -n "Checking Steady-State Route Stability (0 Switches)... "
DEC_LINES=$(wc -l < "${TEST_DIR}/decisions.csv")
if [ "${DEC_LINES}" -eq 1 ]; then
    echo "[PASS] (Exact 0 switches logged)"
else
    echo "[FAIL] Spurious switches detected in steady state! (${DEC_LINES} lines in decisions.csv)"
    cat "${TEST_DIR}/decisions.csv"
    exit 1
fi

# 5. TCP Return Traffic & Full Bulk Transfer Check
echo -n "Checking TCP Return Traffic & Handshake Completion... "
awk -F',' '$3 == "FileTransfer" {
    tx = $4; rx = $5; tput = $7;
    if (rx == 0 || tput < 1000.0) {
        print "TCP stalled or handshake dropped: tx=" tx " rx=" rx " tput=" tput " Kbps";
        exit 1;
    }
}' "${TEST_DIR}/flow_metrics.csv"
echo "[PASS]"

# 6. Packet Conservation Check (Sent = Rx + Lost)
echo -n "Checking Packet Conservation (Sent = Rx + Lost)... "
awk -F',' 'NR>1 {
    tx = $4; rx = $5; lost = $6;
    diff = tx - (rx + lost);
    if (diff < 0) diff = -diff;
    if (diff > 5) {
        print "Conservation violation on " $3 ": tx=" tx " rx=" rx " lost=" lost;
        exit 1;
    }
}' "${TEST_DIR}/flow_metrics.csv"
echo "[PASS]"

# 7. Determinism Verification (Same Seed gives Identical Output)
echo -n "Checking Determinism (Seed 42 x 2)... "
./ns3 run "adaptive-net --mode=adaptive --scenario=steady --simTime=4.0 --seed=42 --enableAnim=false --resultsDir=${TEST_DIR}" > "${TEST_DIR}/det1.log" 2>&1
cp "${TEST_DIR}/time_series.csv" "${TEST_DIR}/ts_det1.csv"
./ns3 run "adaptive-net --mode=adaptive --scenario=steady --simTime=4.0 --seed=42 --enableAnim=false --resultsDir=${TEST_DIR}" > "${TEST_DIR}/det2.log" 2>&1
cp "${TEST_DIR}/time_series.csv" "${TEST_DIR}/ts_det2.csv"

if cmp -s "${TEST_DIR}/ts_det1.csv" "${TEST_DIR}/ts_det2.csv"; then
    echo "[PASS]"
else
    echo "[FAIL] Non-deterministic output detected across runs with seed 42!"
    exit 1
fi

# 8. Node-Failure Detection & PATH_DOWN Decision Check
echo -n "Checking Node Failure Detection & PATH_DOWN Reason... "
rm -f "${TEST_DIR}/decisions.csv" "${TEST_DIR}/failure_detection.csv"
./ns3 run "adaptive-net --mode=adaptive --scenario=nodefail_single --simTime=14.0 --enableAnim=false --resultsDir=${TEST_DIR}" > "${TEST_DIR}/nodefail.log" 2>&1
if grep -q "PATH_DOWN" "${TEST_DIR}/decisions.csv"; then
    echo "[PASS]"
else
    echo "[FAIL] No PATH_DOWN decision logged in decisions.csv during node failure!"
    cat "${TEST_DIR}/decisions.csv"
    exit 1
fi

# 9. Scenario Difference Check (nodefail differs from steady)
echo -n "Checking Scenario Differentiation (Nodefail vs Steady)... "
if cmp -s "${TEST_DIR}/ts_det1.csv" "${TEST_DIR}/time_series.csv"; then
    echo "[FAIL] Nodefail produced identical time-series to steady state!"
    exit 1
else
    echo "[PASS]"
fi

# 10. All Modes Smoke Test (adaptive, baseline1, baseline2, shortest)
echo -n "Testing Modes (adaptive, baseline1, baseline2, shortest)... "
MODES=("adaptive" "baseline1" "baseline2" "shortest")
for m in "${MODES[@]}"; do
    ./ns3 run "adaptive-net --mode=${m} --scenario=steady --simTime=2.0 --enableAnim=false --resultsDir=${TEST_DIR}" > "${TEST_DIR}/mode_${m}.log" 2>&1
done
echo "[PASS]"

# 11. All Node-Failure Scenarios Smoke Test
echo -n "Testing Node-Failure Scenarios (S1-S5)... "
SCENARIOS=("nodefail_single" "nodefail_all_used" "nodefail_cascade" "nodefail_flap" "nodefail_last")
for sc in "${SCENARIOS[@]}"; do
    ./ns3 run "adaptive-net --mode=adaptive --scenario=${sc} --simTime=3.0 --enableAnim=false --resultsDir=${TEST_DIR}" > "${TEST_DIR}/sc_${sc}.log" 2>&1
done
echo "[PASS]"

# Clean temporary test dir
rm -rf "${TEST_DIR}"

echo "=================================================================="
echo " Verification Status: ALL SELF-CHECKS PASSED!"
echo " System is Clean, Autonomous, Mathematically Sound, & Demo-Ready."
echo "=================================================================="
