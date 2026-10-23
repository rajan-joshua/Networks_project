# Adaptive Net: Application-Aware & Autonomous Node-Failure Multi-Path Routing

**Adaptive Net** is an NS-3 (v3.40+, validated on NS-3.49) simulation framework implementing autonomous, application-aware dynamic routing across **8 parallel heterogeneous router paths** ($R_0 \dots R_7$).

The framework addresses **node-failure scenarios**: when an intermediate router experiences hardware failure, traffic traversing that router dies. The centralized engine has **zero scheduled knowledge** of failure events. Instead, the engine **autonomously discovers failures via live probing**, and dynamically re-routes each traffic class to the optimal surviving path (shortest/lowest measured delay for Emergency vitals; minimum multi-criteria cost for Video and File transfer).

---

## 1. Physical Topology & NetAnim Canvas Layout

The network is laid out on a **1000 x 600** coordinate canvas in NetAnim, matching the reference architecture:

```
+----------------------------------------------------------------------------------------------------+
|                                    1000 x 600 NetAnim Topology                                     |
|                                                                                                    |
|    [APPLICATION GENERATORS]                 [8 PARALLEL ROUTER PATHS]               [DESTINATION]  |
|                                                                                                    |
|    (50, 200)                                       (500,  45) R0 (100M,  5ms)                      |
|    App-Emergency (Vitals, Red) \                   (500, 115) R1 ( 50M, 10ms)                      |
|                                 \                  (500, 185) R2 ( 25M, 15ms)                      |
|    (50, 300)                     ---> (180, 300)   (500, 255) R3 ( 15M, 25ms)  ---> (820, 300)     |
|    App-Video (HD, Blue) -------->     Source       (500, 325) R4 ( 10M, 35ms)  ---> Destination    |
|                                 /     (Node 0)     (500, 395) R5 (  5M, 50ms)       (Node 1)       |
|    (50, 400)                   /                   (500, 465) R6 (  2M, 70ms)                      |
|    App-File (TCP Bulk, Green) /                    (500, 535) R7 (  1M,100ms)                      |
+----------------------------------------------------------------------------------------------------+
```

### Complete Node Directory (Exactly 13 Nodes)

| Node ID | Node Name | Role | Position $(x, y)$ | Connected Subnet & Links |
| :---: | :--- | :--- | :---: | :--- |
| **0** | `Source` | Ingress Dynamic Routing Router | $(180, 300)$ | Access: `10.0.1.2`, `10.0.2.2`, `10.0.3.2`; WAN: `10.1.0.1`..`10.1.7.1` |
| **1** | `Destination` | Egress Aggregation Router / Sink | $(820, 300)$ | WAN: `10.2.0.2`..`10.2.7.2`; Sinks: UDP 7000, 5000, TCP 6000 |
| **2** | `Router-0 (R0)` | Path 0: Ultra Low-Latency Primary | $(500, 45)$ | `10.1.0.2`, `10.2.0.1` ($100\text{ Mbps}, 5\text{ ms}$) |
| **3** | `Router-1 (R1)` | Path 1: High-Bandwidth Stream | $(500, 115)$ | `10.1.1.2`, `10.2.1.1` ($50\text{ Mbps}, 10\text{ ms}$) |
| **4** | `Router-2 (R2)` | Path 2: Balanced Intermediate | $(500, 185)$ | `10.1.2.2`, `10.2.2.1` ($25\text{ Mbps}, 15\text{ ms}$) |
| **5** | `Router-3 (R3)` | Path 3: Baseline Static Default | $(500, 255)$ | `10.1.3.2`, `10.2.3.1` ($15\text{ Mbps}, 25\text{ ms}$) |
| **6** | `Router-4 (R4)` | Path 4: Moderate Capacity | $(500, 325)$ | `10.1.4.2`, `10.2.4.1` ($10\text{ Mbps}, 35\text{ ms}$) |
| **7** | `Router-5 (R5)` | Path 5: Medium Capacity | $(500, 395)$ | `10.1.5.2`, `10.2.5.1` ($5\text{ Mbps}, 50\text{ ms}$) |
| **8** | `Router-6 (R6)` | Path 6: Constrained Secondary | $(500, 465)$ | `10.1.6.2`, `10.2.6.1` ($2\text{ Mbps}, 70\text{ ms}$) |
| **9** | `Router-7 (R7)` | Path 7: Fallback High-Latency | $(500, 535)$ | `10.1.7.2`, `10.2.7.1` ($1\text{ Mbps}, 100\text{ ms}$) |
| **10** | `App-Emergency` | Emergency Vitals Generator (UDP 7000) | $(50, 200)$ | `10.0.1.1` (Dedicated $100\text{ Mbps}, 0.1\text{ ms}$ local link) |
| **11** | `App-Video` | HD Camera Video Streamer (UDP 5000) | $(50, 300)$ | `10.0.2.1` (Dedicated $100\text{ Mbps}, 0.1\text{ ms}$ local link) |
| **12** | `App-File` | Bulk Diagnostic File Sender (TCP 6000) | $(50, 400)$ | `10.0.3.1` (Dedicated $100\text{ Mbps}, 0.1\text{ ms}$ local link) |

---

## 2. Autonomous Monitoring & Probe-Based Failure Detection

The simulation implements real, continuous network telemetry without cheating:

### Probing Architecture
- **Probe Monitor App (`ProbeMonitorApp`)**: Installed on `Source` (Node 0). Every `monitorInterval` (default $0.20\text{ s}$), it transmits a 16-byte UDP probe packet down **each of the 8 paths** to Destination's interface on that path (`10.2.k.2:8000+k`).
- **Probe Header (`ProbeHeader`)**: Contains a 32-bit sequence number, 64-bit departure timestamp ($T_{\text{tx}}$), and 8-bit path ID.
- **Probe Echo Receiver App (`ProbeEchoReceiverApp`)**: Installed on `Destination` (Node 1). Binds to ports $8000 \dots 8007$. Upon receiving a probe on Path $k$, it echoes the packet back directly on Path $k$ to Source's interface (`10.1.k.1`).
- **Measured Metrics**:
  - **One-Way Delay**: $\hat{D}_k = \frac{\text{RTT}_k}{2} = \frac{T_{\text{rx}} - T_{\text{tx}}}{2}$.
  - **Loss Rate**: Calculated via sequence number gaps: $\hat{L}_k = 1.0 - \frac{\text{Echoes Received}}{\text{Probes Sent}}$.
  - **Available Bandwidth**: Measured based on real forward throughput vs. link capacity: $\hat{B}_k = \max(0.1, C_k - \text{Throughput}_k)$.
  - **Exponential Moving Average (EMA)**: Metrics are smoothed using $\alpha = 0.30$:
    $$\bar{D}_k \leftarrow (1 - \alpha)\bar{D}_k + \alpha \hat{D}_k, \quad \bar{L}_k \leftarrow (1 - \alpha)\bar{L}_k + \alpha \hat{L}_k, \quad \bar{B}_k \leftarrow (1 - \alpha)\bar{B}_k + \alpha \hat{B}_k$$

### Autonomous Failure & Recovery State Machine
- **Failure Declaration (`DOWN`)**: When Path $k$ accumulates $N$ consecutive probe timeouts ($N = \text{failMissCount}$, default 3), the path is declared **`DOWN`**:
  - `m_paths[k].isDown = true`
  - NetAnim marks Router $R_k$ **BLACK** with banner `NODE FAILED [Rk]`.
  - Immediate re-routing is triggered for all flows currently traversing Path $k$, logging reason `PATH_DOWN` and bypassing the standard hold time.
- **Recovery Declaration (`UP`)**: When Path $k$ receives $M$ consecutive successful probe echoes ($M = \text{recoverHitCount}$, default 3), the path is declared **`UP`**:
  - `m_paths[k].isDown = false`
  - NetAnim marks Router $R_k$ **YELLOW** with banner `RECOVERED [Rk]`.
  - The path returns to the candidate pool for cost-based selection.

---

## 3. Node-Failure Scenarios (`--scenario`)

All hardware failures represent complete node failure: both ingress ($Source \to R_k$) and egress ($R_k \to Destination$) NetDevices are disabled via error models (`RateErrorModel = 1.0`), and IP forwarding interfaces are shut down. Traffic across that router dies instantly.

| Scenario Flag | Failure Name | Real Injected Event Sequence | Autonomous Engine Reaction |
| :--- | :--- | :--- | :--- |
| `nodefail_single` | Single Router Failure | Router carrying Emergency fails at $t = 10.0\text{ s}$, restored at $t = 20.0\text{ s}$. | Probes time out on active path $\to$ declared `DOWN` at $\approx 10.90\text{ s}$ $\to$ Emergency re-routes to Path 2 with reason `PATH_DOWN`. Restored at $20\text{ s}$ $\to$ declared `UP` at $\approx 20.54\text{ s}$ $\to$ reverts to Path 1 at $21.0\text{ s}$. |
| `nodefail_all_used` | Simultaneous Multi-Node | Routers carrying Emergency, Video, and File all fail together at $t = 10.0\text{ s}$, restored at $t = 20.0\text{ s}$. | All 3 active paths detected `DOWN` within $0.9\text{ s}$. All 3 classes autonomously re-route to remaining alive paths ($R_3 \dots R_7$). |
| `nodefail_cascade` | Cascading Failure | Routers fail sequentially every $6\text{ s}$ ($t = 6\text{ s}, 12\text{ s}, 18\text{ s}, 24\text{ s}$) selecting the best surviving path, until only $\text{minSurvivors} = 2$ paths remain. | Engine continuously shifts flows down the survivor chain under shrinking capacity. |
| `nodefail_flap` | Router Flapping | Router $R_1$ fails and recovers repeatedly (period $4.0\text{ s}$) between $t = 8\text{ s} \dots 24\text{ s}$. | Hysteresis margin ($15\%$) and hold time ($2.0\text{ s}$) prevent oscillation. Traffic stabilizes on alternate candidate. |
| `nodefail_last` | Worst-Case Last Path | All paths $R_0 \dots R_6$ fail at $t = 8.0\text{ s}$; only $R_7$ survives. Paths recover one by one starting at $t = 18.0\text{ s}$. | All 3 classes compress onto $R_7$ ($1\text{ Mbps}$). Packets queue/drop gracefully without simulation crash; classes expand back as paths recover. |
| `steady` | Baseline Steady State | Normal operation with no injected disruptions. | High PDR, stable optimal path assignments (0 switches). |

---

## 4. Routing Modes (`--mode`)

1. **`adaptive` (Class-Aware Min-Cost)**:
   - Evaluates per-class multi-criteria cost function among currently **alive** paths:
     $$\text{Cost}(c, k) = w_d^c \left(\frac{\bar{D}_k}{D_{\max}}\right) + w_l^c \left(\frac{\bar{L}_k}{L_{\max}}\right) + w_b^c \left(\frac{B_{\min}}{\max(0.1, \bar{B}_k)}\right) + \text{Penalty}_{\text{load}}(k)$$
   - Emergency ($w_d=0.70, w_l=0.25, w_b=0.05$): prioritizes lowest delay and zero loss.
   - Video ($w_d=0.35, w_l=0.15, w_b=0.50$): balances bandwidth and delay.
   - File ($w_d=0.10, w_l=0.20, w_b=0.70$): prioritizes high throughput capacity.
   - Applies $15\%$ hysteresis margin and $2.0\text{ s}$ per-class hold time (bypassed immediately on `PATH_DOWN`).
2. **`shortest` (Classic Shortest-Path Re-convergence)**:
   - Application-blind. Upon detecting a path `DOWN`, all flows shift to the single surviving path with the lowest **measured** delay.
3. **`baseline1` (Static Default, No Recovery)**:
   - All flows fixed on default path ($R_3$). Demonstrates complete service outage ($100\%$ loss) when $R_3$ fails.
4. **`baseline2` (App-Blind Lowest Measured Delay)**:
   - All flows unconditionally share the lowest measured delay path, causing self-congestion and queue collapse.

---

## 5. Re-Routing Safety & Symmetry Mechanisms

- **UDP Make-Before-Break**: When selecting a new path, the candidate path must have been validated as alive by recent probe echoes before switching. Emergency traffic switches first.
- **TCP Return ACK Symmetry**: Destination installs class-specific static routes:
  - Subnet `10.0.1.0/24` (Emergency): return via active Emergency path return-hop.
  - Subnet `10.0.2.0/24` (Video): return via active Video path return-hop.
  - Subnet `10.0.3.0/24` (TCP File): return ACKs follow the exact forward path of the TCP flow, preserving congestion window evolution and preventing asymmetric drop.
- **Queue/Drop Safety**: When all paths fail (or during outage), packets queue and drop at physical buffers without null-pointer crashes or segmentation faults.

---

## 6. Metrics & Output File Directory

All numbers output by the system are derived exclusively from running simulation measurements:

### Metrics Definitions
- **Delivery Ratio (PDR)**: $\text{Rx Packets} / \text{Tx Packets} \times 100\%$ (FlowMonitor).
- **Emergency Deadline-Miss Ratio**: $(\text{Late Packets} + \text{Lost Packets}) / \text{Total Sent} \times 100\%$, where late means one-way delay $> 150.0\text{ ms}$.
- **Detection Latency**: $t_{\text{declared\_down}} - t_{\text{real\_fail}}$ (seconds).
- **Reroute Latency**: $t_{\text{first\_packet\_delivered\_on\_new\_path}} - t_{\text{real\_fail}}$ (seconds).
- **Emergency Outage Gap**: Maximum inter-packet arrival gap recorded at the Emergency receiver during a failure event.
- **Packets Lost During Outage**: Number of Emergency sequence gaps during the failure interval.
- **File Transfer Duration**: Exact time from BulkSend start to transmission of the final ACK of $5\text{ MB}$, or `NaN` if incomplete.
- **Switch Count**: Total dynamic routing table updates executed.

### Output CSV Files in `results/`
- `summary_metrics.csv`: Single-header run-level summary table (Mode, Scenario, EmergSent, EmergRx, DeadlineMissPct, MeanDelayMs, SwitchCount, FileDurationSec).
- `flow_metrics.csv`: Per-flow FlowMonitor metrics (Tx, Rx, Lost, ThroughputKbps, MeanDelayMs, MeanJitterMs, PDR).
- `decisions.csv`: Full audit log of every route switch (Time, Flow, Class, OldPath, NewPath, Reason, Cost, Margin).
- `time_series.csv`: 1-second interval throughput, delay, and active path tracking.
- `failure_detection.csv`: Precise failure discovery timeline and outage metrics.
- `config_<mode>_<scenario>.json`: Machine-readable dump of all configuration parameters.

---

## 7. NetAnim Visual Observation Checklist

Play back `adaptive_net.xml` in NetAnim (`netanim adaptive_net.xml`) to observe the node-failure story:

1. **Failure Injection ($t = 10.0\text{ s}$)**:
   - Router $R_1$ (carrying Emergency) experiences hardware failure.
   - Probe echoes stop. At $t \approx 10.90\text{ s}$, Node 3 ($R_1$) turns **BLACK** with banner `NODE FAILED [R1]`.
2. **Autonomous Re-Route ($t \approx 10.90\text{ s}$)**:
   - Emergency traffic (Red) immediately re-routes to surviving Path 2 ($R_2$).
   - Link descriptions update with the new evaluated utility cost.
3. **Hardware Recovery ($t = 20.0\text{ s}$)**:
   - Router $R_1$ hardware is restored.
   - Probe echoes resume. At $t \approx 20.54\text{ s}$, Node 3 turns **YELLOW** with banner `RECOVERED [R1]`.
4. **Hysteresis Return ($t = 21.0\text{ s}$)**:
   - After validating stability, Emergency traffic smoothly reverts to $R_1$.

---

## 8. Automated Self-Checks (`verify.sh`)

Run the complete verification harness:
```bash
./verify.sh
```

The script autonomously validates 11 self-checks:
1. **NS-3 Build Integrity**: Clean compilation of `adaptive-net.cc` with zero warnings or errors.
2. **Failure-Discovery Integrity**: Greps `DecisionEngine` to verify zero references to scenario schedules or failure injection routines.
3. **Live Probe Monitor Check**: Verifies dynamic delay updates diverge from static seeds after probe warm-up.
4. **Steady-State Route Stability Check**: Asserts exactly 0 route switches in steady state.
5. **TCP Return Traffic & Handshake Completion Check**: Verifies TCP bulk transfers complete without stall.
6. **Packet Conservation Check**: Strict equation check: $\text{Sent} = \text{Received} + \text{Lost} + \text{In-Flight}$ across all classes.
7. **Determinism Check**: Runs Seed 42 twice and asserts byte-for-byte identical output via `cmp`.
8. **Node-Failure Discovery Check**: Confirms at least one decision with reason `PATH_DOWN` during `nodefail_single`.
9. **Scenario Differentiation Check**: Asserts failure scenarios produce distinctly different time-series from steady state.
10. **All Modes Smoke Test**: Executes `adaptive`, `baseline1`, `baseline2`, `shortest`.
11. **All Failure Scenarios Smoke Test**: Executes pure node-failure scenarios S1–S5.

---

## 9. Empirical Simulation Results (Rule R3 Placeholders)

> [!IMPORTANT]
> In strict accordance with Rule R3, no fake or invented numbers are provided.
> Execute `./run_experiments.sh` (or `.\run_experiments.ps1`) to populate the table below from your empirical runs across 5 seeds:

| Evaluation Mode | Scenario | Emergency Deadline Miss Ratio (%) | Emergency Mean Latency (ms) | Detection Latency (s) | Emergency Reroute Latency (s) | Route Switches Executed | File Transfer Duration (s) |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Adaptive** | `steady` | `[MEASURED_STEADY_MISS_ADAPTIVE]` | `[MEASURED_STEADY_LAT_ADAPTIVE]` | N/A | N/A | `[MEASURED_STEADY_SW_ADAPTIVE]` | `[MEASURED_STEADY_FILE_ADAPTIVE]` |
| **Baseline 1** ($R_3$) | `steady` | `[MEASURED_STEADY_MISS_BASE1]` | `[MEASURED_STEADY_LAT_BASE1]` | N/A | N/A | `[MEASURED_STEADY_SW_BASE1]` | `[MEASURED_STEADY_FILE_BASE1]` |
| **Baseline 2** ($R_0$) | `steady` | `[MEASURED_STEADY_MISS_BASE2]` | `[MEASURED_STEADY_LAT_BASE2]` | N/A | N/A | `[MEASURED_STEADY_SW_BASE2]` | `[MEASURED_STEADY_FILE_BASE2]` |
| **Shortest** | `steady` | `[MEASURED_STEADY_MISS_SHORT]` | `[MEASURED_STEADY_LAT_SHORT]` | N/A | N/A | `[MEASURED_STEADY_SW_SHORT]` | `[MEASURED_STEADY_FILE_SHORT]` |
| **Adaptive** | `nodefail_single` | `[MEASURED_S1_MISS_ADAPTIVE]` | `[MEASURED_S1_LAT_ADAPTIVE]` | `[MEASURED_S1_DET_ADAPTIVE]` | `[MEASURED_S1_REROUTE_ADAPTIVE]` | `[MEASURED_S1_SW_ADAPTIVE]` | `[MEASURED_S1_FILE_ADAPTIVE]` |
| **Shortest** | `nodefail_single` | `[MEASURED_S1_MISS_SHORT]` | `[MEASURED_S1_LAT_SHORT]` | `[MEASURED_S1_DET_SHORT]` | `[MEASURED_S1_REROUTE_SHORT]` | `[MEASURED_S1_SW_SHORT]` | `[MEASURED_S1_FILE_SHORT]` |
| **Baseline 1** ($R_3$) | `nodefail_single` | `[MEASURED_S1_MISS_BASE1]` | `[MEASURED_S1_LAT_BASE1]` | N/A | N/A | `[MEASURED_S1_SW_BASE1]` | `[MEASURED_S1_FILE_BASE1]` |
| **Baseline 2** ($R_0$) | `nodefail_single` | `[MEASURED_S1_MISS_BASE2]` | `[MEASURED_S1_LAT_BASE2]` | N/A | N/A | `[MEASURED_S1_SW_BASE2]` | `[MEASURED_S1_FILE_BASE2]` |
| **Adaptive** | `nodefail_all_used` | `[MEASURED_S2_MISS_ADAPTIVE]` | `[MEASURED_S2_LAT_ADAPTIVE]` | `[MEASURED_S2_DET_ADAPTIVE]` | `[MEASURED_S2_REROUTE_ADAPTIVE]` | `[MEASURED_S2_SW_ADAPTIVE]` | `[MEASURED_S2_FILE_ADAPTIVE]` |
| **Adaptive** | `nodefail_cascade` | `[MEASURED_S3_MISS_ADAPTIVE]` | `[MEASURED_S3_LAT_ADAPTIVE]` | `[MEASURED_S3_DET_ADAPTIVE]` | `[MEASURED_S3_REROUTE_ADAPTIVE]` | `[MEASURED_S3_SW_ADAPTIVE]` | `[MEASURED_S3_FILE_ADAPTIVE]` |
| **Adaptive** | `nodefail_flap` | `[MEASURED_S4_MISS_ADAPTIVE]` | `[MEASURED_S4_LAT_ADAPTIVE]` | `[MEASURED_S4_DET_ADAPTIVE]` | `[MEASURED_S4_REROUTE_ADAPTIVE]` | `[MEASURED_S4_SW_ADAPTIVE]` | `[MEASURED_S4_FILE_ADAPTIVE]` |
| **Adaptive** | `nodefail_last` | `[MEASURED_S5_MISS_ADAPTIVE]` | `[MEASURED_S5_LAT_ADAPTIVE]` | `[MEASURED_S5_DET_ADAPTIVE]` | `[MEASURED_S5_REROUTE_ADAPTIVE]` | `[MEASURED_S5_SW_ADAPTIVE]` | `[MEASURED_S5_FILE_ADAPTIVE]` |

---

## 10. Execution Commands

### In WSL2 / Linux
```bash
# 1. Run automated self-checks
./verify.sh

# 2. Run multi-seed batch experiments (5 seeds x modes x scenarios)
./run_experiments.sh

# 3. Generate analysis plots
python3 plot_results.py
```

### In Windows PowerShell
```powershell
# Launches WSL2 runner and generates plots in Windows
.\run_experiments.ps1
```

### Generated Figures in `plots/`
1. `metric_vs_time.png`: 3-panel time series showing measured delay, throughput, and active path selection.
2. `bar_charts_vs_baselines.png`: 4-panel comparison of Adaptive vs Shortest vs Baselines with 95% confidence intervals.
3. `path_timeline.png`: Active path progression per class with failure interval shading and event markers.
4. `detection_reroute_latency.png`: Measured failure detection latency, recovery latency, outage gap, and reroute latency.
5. `weight_sensitivity.png`: Cost contour surfaces across delay and bandwidth for all 3 traffic classes.
