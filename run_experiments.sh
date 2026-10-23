#!/bin/bash
# ==============================================================================
# Adaptive Net: Multi-Seed Experiment Suite (Node-Failure Architecture)
# Evaluates Modes (adaptive, baseline1, baseline2, shortest) across
# Scenarios (steady, nodefail_single, nodefail_all_used,
#            nodefail_cascade, nodefail_flap, nodefail_last) with 5 seeds.
# ==============================================================================
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NS3_DIR="${HOME}/ns-3.49"
RESULTS_DIR="${WORKSPACE_DIR}/results"
PLOTS_DIR="${WORKSPACE_DIR}/plots"

echo "=================================================================="
echo "    Adaptive Net: Comprehensive Experiment Execution Suite        "
echo "=================================================================="
echo "Workspace:   ${WORKSPACE_DIR}"
echo "NS-3 Tree:   ${NS3_DIR}"
echo "Results Dir: ${RESULTS_DIR}"

mkdir -p "${RESULTS_DIR}"
mkdir -p "${PLOTS_DIR}"
mkdir -p "${NS3_DIR}/scratch"

# Sync primary source code into ns-3 scratch
cp "${WORKSPACE_DIR}/adaptive-net.cc" "${NS3_DIR}/scratch/adaptive-net.cc"

cd "${NS3_DIR}"

# Build scratch target
echo ">> Building scratch/adaptive-net..."
./ns3 build scratch/adaptive-net

# Clean prior run files
rm -f "${RESULTS_DIR}"/*.csv "${RESULTS_DIR}"/*.json
rm -f flow_metrics.csv summary_metrics.csv decisions.csv time_series.csv failure_detection.csv

SEEDS=(1 2 3 4 5)

echo ""
echo "=== Step 1: Evaluating All Routing Modes (Scenario: steady, 5 Seeds) ==="
MODES=("adaptive" "baseline1" "baseline2" "shortest")
for mode in "${MODES[@]}"; do
    for seed in "${SEEDS[@]}"; do
        echo ">> [Mode: ${mode} | Scenario: steady | Seed: ${seed}] Running..."
        ./ns3 run "adaptive-net --mode=${mode} --scenario=steady --simTime=30.0 --seed=${seed} --resultsDir=${RESULTS_DIR}"
        if [ "${seed}" -eq 1 ]; then
            cp "${RESULTS_DIR}/decisions.csv" "${RESULTS_DIR}/decisions_${mode}_steady.csv" 2>/dev/null || true
            cp "${RESULTS_DIR}/time_series.csv" "${RESULTS_DIR}/time_series_${mode}_steady.csv" 2>/dev/null || true
        fi
    done
done

echo ""
echo "=== Step 2: Evaluating Node Failure Scenarios (Mode: adaptive, 5 Seeds) ==="
SCENARIOS=("nodefail_single" "nodefail_all_used" "nodefail_cascade" "nodefail_flap" "nodefail_last")
for sc in "${SCENARIOS[@]}"; do
    for seed in "${SEEDS[@]}"; do
        echo ">> [Mode: adaptive | Scenario: ${sc} | Seed: ${seed}] Running..."
        ./ns3 run "adaptive-net --mode=adaptive --scenario=${sc} --simTime=30.0 --seed=${seed} --resultsDir=${RESULTS_DIR}"
        if [ "${seed}" -eq 1 ]; then
            cp "${RESULTS_DIR}/decisions.csv" "${RESULTS_DIR}/decisions_adaptive_${sc}.csv" 2>/dev/null || true
            cp "${RESULTS_DIR}/time_series.csv" "${RESULTS_DIR}/time_series_adaptive_${sc}.csv" 2>/dev/null || true
        fi
    done
done

echo ""
echo "=== Step 3: Benchmarking Baselines under Node Failure (Scenario: nodefail_single, 5 Seeds) ==="
BASE_MODES=("baseline1" "baseline2" "shortest")
for mode in "${BASE_MODES[@]}"; do
    for seed in "${SEEDS[@]}"; do
        echo ">> [Mode: ${mode} | Scenario: nodefail_single | Seed: ${seed}] Running..."
        ./ns3 run "adaptive-net --mode=${mode} --scenario=nodefail_single --simTime=30.0 --seed=${seed} --resultsDir=${RESULTS_DIR}"
        if [ "${seed}" -eq 1 ]; then
            cp "${RESULTS_DIR}/decisions.csv" "${RESULTS_DIR}/decisions_${mode}_nodefail_single.csv" 2>/dev/null || true
            cp "${RESULTS_DIR}/time_series.csv" "${RESULTS_DIR}/time_series_${mode}_nodefail_single.csv" 2>/dev/null || true
        fi
    done
done

# Copy primary NetAnim XML to workspace
if [ -f "adaptive_net.xml" ]; then
    cp adaptive_net.xml "${WORKSPACE_DIR}/adaptive_net.xml"
fi

echo ""
echo "=== Step 4: Generating Publication-Quality Figures ==="
cd "${WORKSPACE_DIR}"
if python3 -c "import numpy, pandas, matplotlib" &>/dev/null; then
    python3 plot_results.py
else
    echo "[Info] Python visualization packages not installed inside WSL environment."
    echo "[Info] Run 'python plot_results.py' from Windows PowerShell to generate plots."
fi

echo ""
echo "=================================================================="
echo " Experiment Suite Completed Successfully!"
echo " Empirical data stored in: ${RESULTS_DIR}"
echo " Figures saved in:         ${PLOTS_DIR}"
echo "=================================================================="
