/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/**
 * =============================================================================
 * Adaptive Net: Application-Aware & Network-Aware Multi-Path Simulation
 * Target: NS-3.40+ (Validated on NS-3.49)
 *
 * Core Concept:
 *   Application -> Traffic Classification (ToS + Port) -> Requirement Identification
 *   -> Multi-Criteria Utility Cost Function -> Dynamic Path Selection across
 *   8 Parallel Heterogeneous Router Paths (R0..R7).
 *
 * Key Features:
 *   - Autonomous Node-Failure Discovery via Live Active Probes (Zero Knowledge)
 *   - Compatible-Shortest Route Selection on Failure with Per-Class Eligibility
 *   - Ipv4ListRouting: Class-Steering Layer + Ipv4StaticRouting Local Fallback
 *   - Multi-File Sequential TCP Transfer Queue with Retransmission Tracking
 *   - Scaled NetAnim Animation Profile & XML Size Guard
 *   - Clean Architecture: Exactly 13 Nodes (0:Source, 1:Dest, 2..9:R0..R7, 10..12:Apps)
 *     (Cross-traffic node completely removed).
 * =============================================================================
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/mobility-module.h"
#include "ns3/traffic-control-module.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cmath>
#include <map>
#include <limits>
#include <functional>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AdaptiveNetRefactor");

// =============================================================================
// Traffic Classes & Definitions
// =============================================================================
enum TrafficClass {
    CLASS_EMERGENCY = 0,
    CLASS_VIDEO     = 1,
    CLASS_FILE      = 2,
    CLASS_DEFAULT   = 3
};

static std::string TrafficClassToString(TrafficClass tc) {
    switch (tc) {
        case CLASS_EMERGENCY: return "Emergency";
        case CLASS_VIDEO:     return "Video";
        case CLASS_FILE:      return "FileTransfer";
        default:              return "Default";
    }
}

struct ClassPolicy {
    double weightDelay;
    double weightLoss;
    double weightBandwidth;
};

// =============================================================================
// Comprehensive Configuration Structure (No Hardcoding)
// =============================================================================
struct AdaptiveNetConfig {
    // General Simulation Settings
    std::string mode;
    std::string scenario;
    double simTime;
    uint32_t seed;
    uint32_t runIndex;
    bool enableAnim;
    std::string animFile;
    std::string resultsDir;

    // Node Coordinates (~1000 x 600 NetAnim Canvas)
    Vector posAppEmerg;
    Vector posAppVideo;
    Vector posAppFile;
    Vector posSource;
    Vector posRouters[8];
    Vector posDestination;

    // Layout Separation Constraints
    double minNodeGap;
    double routerFanoutGap;

    // 8 Parallel Path Heterogeneous Parameters (Source - Rk - Destination)
    std::string pathRates[8];
    std::string pathDelays[8];
    double pathLossRates[8];
    std::string pathQueues[8];

    // Local Access Bus Parameters
    std::string accessBusRate;
    std::string accessBusDelay;

    // Monitoring & Probing Parameters
    double monitorIntervalSec;
    double probeIntervalSec;
    uint32_t probePacketSize;
    uint16_t probePort;
    uint32_t failMissCount;
    uint32_t recoverHitCount;
    double emaAlpha;
    double hysteresisMargin;
    double holdTimeSec;
    double loadPenaltyWeight;

    // Traffic Class QoS Policy Weights
    ClassPolicy policyEmergency;
    ClassPolicy policyVideo;
    ClassPolicy policyFile;

    // Application Parameters & ToS / DSCP Settings
    uint16_t emergPort;
    double emergIntervalMs;
    uint32_t emergPacketSize;
    double emergDeadlineMs;
    uint32_t emergTos;

    uint16_t videoPort;
    std::string videoDataRate;
    uint32_t videoPacketSize;
    uint32_t videoTos;

    uint16_t filePort;
    uint32_t fileTos;
    uint32_t fileCount;
    std::string fileSizesBytes;
    double fileGapSec;
    bool isStress;

    // NetAnim Profile Configuration
    double animStartTime;
    double animStopTime;
    std::string animVideoRate;
    double animProbeIntervalSec;
    bool animPacketMeta;
    bool measureBw;
    uint32_t congDropThreshold;
    bool enableSpill;
    std::string profile;
    bool realLoss;
    double spillHigh;
    bool animLoad;
    double spillLow;
    double bufSampleSec;
    double congUtilThreshold;
    std::string burstRate;
    double burstStart;
    double burstEnd;
    double maxXmlMB;

    // Route Eligibility Thresholds (Compatible Path on Failure)
    double emergMaxDelayEligibleMs;
    double emergMaxLossEligible;
    double videoHeadroom;
    double fileMaxLossEligible;
    double fileMinRateEligibleMbps;

    // Scenario Injected Event Timers
    double nodeFailStartTime;
    double nodeFailEndTime;
    double cascadePeriodSec;
    double flapPeriodSec;
    uint32_t minSurvivors;

    AdaptiveNetConfig()
        : mode("adaptive"),
          scenario("steady"),
          simTime(30.0),
          seed(1),
          runIndex(0),
          enableAnim(false),
          animFile("adaptive_net.xml"),
          resultsDir("results"),
          posAppEmerg(50.0, 200.0, 0.0),
          posAppVideo(50.0, 300.0, 0.0),
          posAppFile(50.0, 400.0, 0.0),
          posSource(330.0, 300.0, 0.0),
          posDestination(820.0, 300.0, 0.0),
          minNodeGap(60.0),
          routerFanoutGap(60.0),
          accessBusRate("100Mbps"),
          accessBusDelay("0.1ms"),
          monitorIntervalSec(0.5),
          probeIntervalSec(0.2),
          probePacketSize(64),
          probePort(8888),
          failMissCount(3),
          recoverHitCount(2),
          emaAlpha(0.30),
          hysteresisMargin(0.15),
          holdTimeSec(2.0),
          loadPenaltyWeight(0.60),
          emergPort(7000),
          emergIntervalMs(100.0),
          emergPacketSize(128),
          emergDeadlineMs(150.0),
          emergTos(0xB8),      // DSCP EF (Expedited Forwarding, 46 << 2 = 184)
          videoPort(5000),
          videoDataRate("2.5Mbps"),
          videoPacketSize(1400),
          videoTos(0x88),      // DSCP AF41 (34 << 2 = 136)
          filePort(6000),
          fileTos(0x28),       // DSCP AF11 (10 << 2 = 40)
          fileCount(1),
          fileSizesBytes("5000000"),
          fileGapSec(1.0),
          isStress(false),
          animStartTime(0.0),
          animStopTime(15.0),
          animVideoRate("250Kbps"),
          animProbeIntervalSec(0.2),   // keep fast failure detection in anim runs
          animPacketMeta(false),       // packet text labels clutter NetAnim
          measureBw(false),
          congDropThreshold(3),
          enableSpill(true),
          profile("default"),
          realLoss(false),
          animLoad(true),
          spillHigh(1.00),   // spill only when the buffer is completely FULL
          spillLow(0.50),
          bufSampleSec(0.1),
          congUtilThreshold(0.95),
          burstRate("60Mbps"),
          burstStart(8.0),
          burstEnd(13.0),
          maxXmlMB(15.0),
          emergMaxDelayEligibleMs(75.0), // deadline / 2 = 75 ms
          emergMaxLossEligible(0.02),    // 2% max loss
          videoHeadroom(1.20),           // 20% bandwidth headroom
          fileMaxLossEligible(0.05),     // 5% max loss
          fileMinRateEligibleMbps(1.0),  // 1 Mbps min bandwidth
          nodeFailStartTime(10.0),
          nodeFailEndTime(20.0),
          cascadePeriodSec(4.0),
          flapPeriodSec(3.0),
          minSurvivors(2)
    {
        // 8 Router Positions fanned vertically at x = 500
        for (int k = 0; k < 8; ++k) {
            posRouters[k] = Vector(500.0, 45.0 + k * 70.0, 0.0);
        }

        // Heterogeneous 8-Path Configuration
        pathRates[0] = "100Mbps"; pathDelays[0] = "5ms";   pathLossRates[0] = 0.0001; pathQueues[0] = "150p";
        pathRates[1] = "50Mbps";  pathDelays[1] = "10ms";  pathLossRates[1] = 0.0002; pathQueues[1] = "150p";
        pathRates[2] = "25Mbps";  pathDelays[2] = "15ms";  pathLossRates[2] = 0.0005; pathQueues[2] = "100p";
        pathRates[3] = "15Mbps";  pathDelays[3] = "25ms";  pathLossRates[3] = 0.0010; pathQueues[3] = "100p";
        pathRates[4] = "10Mbps";  pathDelays[4] = "35ms";  pathLossRates[4] = 0.0020; pathQueues[4] = "80p";
        pathRates[5] = "5Mbps";   pathDelays[5] = "50ms";  pathLossRates[5] = 0.0050; pathQueues[5] = "60p";
        pathRates[6] = "2Mbps";   pathDelays[6] = "70ms";  pathLossRates[6] = 0.0100; pathQueues[6] = "50p";
        pathRates[7] = "1Mbps";   pathDelays[7] = "100ms"; pathLossRates[7] = 0.0200; pathQueues[7] = "40p";

        // Weights
        policyEmergency = {0.70, 0.25, 0.05};
        policyVideo     = {0.35, 0.15, 0.50};
        policyFile      = {0.10, 0.55, 0.35};   // File = reliability first (problem statement)
    }

    void BindCommandLine(CommandLine &cmd) {
        cmd.AddValue("mode", "Operational routing mode (adaptive, baseline1, baseline2, shortest)", mode);
        cmd.AddValue("scenario", "Simulation scenario (steady, nodefail_single, nodefail_all_used, nodefail_cascade, nodefail_flap, nodefail_last, congestion)", scenario);
        cmd.AddValue("simTime", "Simulation execution duration in seconds", simTime);
        cmd.AddValue("seed", "RNG seed value", seed);
        cmd.AddValue("runIndex", "Experiment run index", runIndex);
        cmd.AddValue("enableAnim", "Enable or disable NetAnim trace generation", enableAnim);
        cmd.AddValue("animFile", "NetAnim XML trace filename", animFile);
        cmd.AddValue("resultsDir", "Output directory for CSV and JSON metric files", resultsDir);

        cmd.AddValue("monitorInterval", "Monitoring period in seconds", monitorIntervalSec);
        cmd.AddValue("probeInterval", "Probe transmission interval in seconds", probeIntervalSec);
        cmd.AddValue("failMissCount", "Consecutive probe misses to declare path DOWN", failMissCount);
        cmd.AddValue("recoverHitCount", "Consecutive probe successes to declare path UP", recoverHitCount);
        cmd.AddValue("emaAlpha", "Exponential Moving Average smoothing alpha", emaAlpha);
        cmd.AddValue("hysteresisMargin", "Utility improvement threshold margin", hysteresisMargin);
        cmd.AddValue("holdTime", "Route switch hold time in seconds", holdTimeSec);
        cmd.AddValue("loadPenalty", "Penalty factor for paths carrying multiple classes", loadPenaltyWeight);

        cmd.AddValue("emergTos", "IPv4 ToS byte for Emergency traffic", emergTos);
        cmd.AddValue("videoTos", "IPv4 ToS byte for Video traffic", videoTos);
        cmd.AddValue("fileTos", "IPv4 ToS byte for File traffic", fileTos);

        cmd.AddValue("animStartTime", "NetAnim capture start time", animStartTime);
        cmd.AddValue("animStopTime", "NetAnim capture stop time", animStopTime);
        cmd.AddValue("animVideoRate", "NetAnim scaled video data rate", animVideoRate);
        cmd.AddValue("animProbeInterval", "NetAnim scaled probe interval", animProbeIntervalSec);
        cmd.AddValue("animPacketMeta", "Show packet metadata text in NetAnim (cluttered)", animPacketMeta);
        cmd.AddValue("measureBw", "Feed live per-path throughput into available-bandwidth estimate", measureBw);
        cmd.AddValue("profile", "Topology profile: default | tradeoff (no single router is best at everything)", profile);
        cmd.AddValue("realLoss", "Apply each path's loss rate as a real packet-error model", realLoss);
        cmd.AddValue("enableSpill", "Send packets to a standby route while the primary buffer is nearly full", enableSpill);
        cmd.AddValue("animLoad", "In animation runs add two short load bursts so router buffers visibly fill", animLoad);
        cmd.AddValue("spillHigh", "Buffer occupancy (0..1) that starts spill-over to a standby route", spillHigh);
        cmd.AddValue("spillLow", "Buffer occupancy (0..1) at which traffic returns to the primary route", spillLow);
        cmd.AddValue("congDrops", "Queue drops per monitor interval that mark a path CONGESTED", congDropThreshold);
        cmd.AddValue("congUtil", "Link utilisation (0..1) that marks a path CONGESTED", congUtilThreshold);
        cmd.AddValue("burstRate", "Extra video-class load rate in scenario 'congestion'", burstRate);
        cmd.AddValue("burstStart", "Burst start time (scenario 'congestion')", burstStart);
        cmd.AddValue("burstEnd", "Burst end time (scenario 'congestion')", burstEnd);
        cmd.AddValue("maxXmlMB", "Maximum allowed NetAnim XML file size in MB", maxXmlMB);

        cmd.AddValue("emergMaxDelayEligible", "Max measured delay for Emergency path eligibility", emergMaxDelayEligibleMs);
        cmd.AddValue("emergMaxLossEligible", "Max measured loss for Emergency path eligibility", emergMaxLossEligible);
        cmd.AddValue("videoHeadroom", "Bandwidth headroom factor for Video eligibility", videoHeadroom);
        cmd.AddValue("fileMaxLossEligible", "Max measured loss for File path eligibility", fileMaxLossEligible);
        cmd.AddValue("fileMinRateEligible", "Min measured rate in Mbps for File eligibility", fileMinRateEligibleMbps);

        cmd.AddValue("fileCount", "Number of sequential files to transfer over TCP", fileCount);
        cmd.AddValue("fileSizes", "Comma-separated file transfer sizes in bytes", fileSizesBytes);
        cmd.AddValue("fileGap", "Pause gap between sequential files in seconds", fileGapSec);
        cmd.AddValue("stress", "Enable stress profile with high rate & multi-file", isStress);

        cmd.AddValue("emergWd", "Emergency delay weight", policyEmergency.weightDelay);
        cmd.AddValue("emergWl", "Emergency loss weight", policyEmergency.weightLoss);
        cmd.AddValue("emergWb", "Emergency bandwidth weight", policyEmergency.weightBandwidth);

        cmd.AddValue("videoWd", "Video delay weight", policyVideo.weightDelay);
        cmd.AddValue("videoWl", "Video loss weight", policyVideo.weightLoss);
        cmd.AddValue("videoWb", "Video bandwidth weight", policyVideo.weightBandwidth);

        cmd.AddValue("fileWd", "File delay weight", policyFile.weightDelay);
        cmd.AddValue("fileWl", "File loss weight", policyFile.weightLoss);
        cmd.AddValue("fileWb", "File bandwidth weight", policyFile.weightBandwidth);

        cmd.AddValue("nodeFailStart", "Node failure start time in seconds", nodeFailStartTime);
        cmd.AddValue("nodeFailEnd", "Node failure restoration time in seconds", nodeFailEndTime);
        cmd.AddValue("cascadePeriod", "Cascade interval between successive node failures", cascadePeriodSec);
        cmd.AddValue("flapPeriod", "Flapping cycle period in seconds", flapPeriodSec);
        cmd.AddValue("minSurvivors", "Minimum survivor paths in cascade scenario", minSurvivors);
    }

    void ApplyProfiles() {
        if (profile == "tradeoff") {
            // fast-but-thin, fat-but-slow, fast-but-lossy ... so each class has a different best router
            const char *r[8] = {"10Mbps","100Mbps","50Mbps","25Mbps","15Mbps","8Mbps","5Mbps","2Mbps"};
            const char *d[8] = {"5ms","30ms","20ms","15ms","40ms","25ms","60ms","90ms"};
            double l[8] = {0.0001,0.0005,0.0200,0.0002,0.0001,0.0100,0.0005,0.0010};
            const char *q[8] = {"100p","150p","100p","100p","80p","60p","50p","40p"};
            for (int k = 0; k < 8; ++k) { pathRates[k]=r[k]; pathDelays[k]=d[k]; pathLossRates[k]=l[k]; pathQueues[k]=q[k]; }
        }
        if (isStress) {
            emergIntervalMs = 20.0; // 50 pkts/s
            videoDataRate = "5Mbps";
            fileCount = 3;
            fileSizesBytes = "1000000,2000000,3000000";
            fileGapSec = 0.5;
        }

        // P3: Scaled animation profile for --anim runs
        if (enableAnim) {
            videoDataRate = animVideoRate;
            probeIntervalSec = animProbeIntervalSec;
            if (fileSizesBytes == "5000000") {
                fileSizesBytes = "2000000"; // 2 MB file in anim trace: its TCP window visibly fills the R2 buffer
            }
        }
    }

    void Print(std::ostream &os) const {
        os << "==================================================================\n";
        os << "       Adaptive Net: Application-Aware 8-Path Multi-Path Engine   \n";
        os << "==================================================================\n";
        os << " Mode:                 " << mode << "\n";
        os << " Scenario:             " << scenario << "\n";
        os << " Duration:             " << simTime << " s | Seed: " << seed << "\n";
        os << " NetAnim Output:       " << (enableAnim ? animFile : "Disabled") << "\n";
        os << " Access Bus:           " << accessBusRate << ", " << accessBusDelay << "\n";
        os << " Monitoring Interval:  " << monitorIntervalSec << " s | Probe Interval: " << probeIntervalSec << " s\n";
        os << " Failure Detection:    Misses=" << failMissCount << " (DOWN) | Hits=" << recoverHitCount << " (UP)\n";
        os << " Hysteresis Margin:    " << hysteresisMargin * 100.0 << "% | Hold Time: " << holdTimeSec << " s\n";
        os << " ToS / DSCP Settings:  Emergency=0x" << std::hex << emergTos
           << " (DSCP " << std::dec << (emergTos >> 2) << "), Video=0x" << std::hex << videoTos
           << " (DSCP " << std::dec << (videoTos >> 2) << "), File=0x" << std::hex << fileTos
           << " (DSCP " << std::dec << (fileTos >> 2) << ")\n";
        os << " Emergency Telemetry:  1 pkt / " << emergIntervalMs << " ms (" << emergPacketSize
           << " B) | Deadline: " << emergDeadlineMs << " ms\n";
        os << " Video Stream:         " << videoDataRate << " (" << videoPacketSize << " B packets)\n";
        os << " File Transfer Queue:  " << fileCount << " file(s) [" << fileSizesBytes << " B]\n";
        os << " Node Failure Window:  " << nodeFailStartTime << "s - " << nodeFailEndTime << "s\n";
        os << "------------------------------------------------------------------\n";
        os << " Path Directory (Source - Rk - Destination):\n";
        for (int k = 0; k < 8; ++k) {
            os << "   Path " << k << " (R" << k << "): " << std::left << std::setw(8) << pathRates[k]
               << std::setw(7) << pathDelays[k]
               << "Loss: " << std::setw(8) << pathLossRates[k] * 100.0 << "% "
               << "Queue: " << pathQueues[k] << "\n";
        }
        os << "==================================================================\n";
    }

    void WriteJson(const std::string &filepath) const {
        std::ofstream jf(filepath);
        if (!jf.is_open()) return;
        jf << "{\n";
        jf << "  \"mode\": \"" << mode << "\",\n";
        jf << "  \"scenario\": \"" << scenario << "\",\n";
        jf << "  \"simTime\": " << simTime << ",\n";
        jf << "  \"seed\": " << seed << ",\n";
        jf << "  \"runIndex\": " << runIndex << ",\n";
        jf << "  \"emergTos\": " << emergTos << ",\n";
        jf << "  \"videoTos\": " << videoTos << ",\n";
        jf << "  \"fileTos\": " << fileTos << ",\n";
        jf << "  \"fileCount\": " << fileCount << ",\n";
        jf << "  \"fileSizesBytes\": \"" << fileSizesBytes << "\",\n";
        jf << "  \"monitorIntervalSec\": " << monitorIntervalSec << ",\n";
        jf << "  \"probeIntervalSec\": " << probeIntervalSec << ",\n";
        jf << "  \"failMissCount\": " << failMissCount << ",\n";
        jf << "  \"recoverHitCount\": " << recoverHitCount << ",\n";
        jf << "  \"emaAlpha\": " << emaAlpha << ",\n";
        jf << "  \"hysteresisMargin\": " << hysteresisMargin << ",\n";
        jf << "  \"holdTimeSec\": " << holdTimeSec << ",\n";
        jf << "  \"emergDeadlineMs\": " << emergDeadlineMs << ",\n";
        jf << "  \"videoDataRate\": \"" << videoDataRate << "\",\n";
        jf << "  \"nodeFailStartTime\": " << nodeFailStartTime << ",\n";
        jf << "  \"nodeFailEndTime\": " << nodeFailEndTime << "\n";
        jf << "}\n";
        jf.close();
    }
};

// =============================================================================
// Helper Function: Check If File Exists And Is Not Empty
// =============================================================================
static bool FileExistsAndNotEmpty(const std::string &path) {
    std::ifstream f(path, std::ios::binary);
    if (!f.is_open()) return false;
    f.seekg(0, std::ios::end);
    return f.tellg() > 0;
}

// =============================================================================
// Internal Path Descriptor (8 Parallel PointToPoint Paths)
// =============================================================================
struct InternalPath {
    uint32_t id;
    std::string name;
    double baseDelayMs;
    double baseBandwidthMbps;
    double baseLossRate;

    // Dynamic Live Measurements updated by Probes
    double smoothedDelayMs;
    double smoothedLossRate;
    double smoothedBandwidthMbps;
    bool isDown;

    // Interface Addressing
    Ipv4Address nextHopIp;      // Rk ingress IP from Source
    Ipv4Address srcIp;          // Source interface IP on Path k
    Ipv4Address destPathIp;     // Destination interface IP on Path k
    Ptr<NetDevice> outputDevice;// Source NetDevice to Rk

    // Return Hop Addressing for Destination
    Ipv4Address returnHopIp;    // Rk egress IP towards Destination
    uint32_t returnInterface;   // Destination interface ID on Path k

    // Bandwidth measurement accumulators
    uint64_t intervalTxBytes = 0;
    uint64_t utilTxBytes = 0;     // congestion monitor (independent of measureBw)
    uint32_t intervalDrops = 0;   // queue overflow drops this interval
    double   utilPct = 0.0;
    double   bufPct = 0.0;
    Ptr<QueueDisc> qdisc;          // the router-side buffer we visualise
    uint64_t offeredBytes = 0;     // bytes offered to the buffer this interval
    double   offeredMbps = 0.0;
    bool     spilling = false;
    double   lastSpillAt = -10.0;
    uint32_t spillMask = 0;        // bit0 emergency, bit1 video, bit2 file
    uint32_t spillTarget = 99;
    uint64_t spillPkts = 0;
    bool     congested = false;
    uint32_t cleanIntervals = 0;

    double ComputeCost(const ClassPolicy &pol,
                       double minD, double maxD,
                       double minB, double maxB,
                       double minL, double maxL,
                       uint32_t classLoadOnPath,
                       double loadPenaltyWeight) const {
        if (isDown) return 1e9;

        // Normalized Delay
        double normD = 0.0;
        if (maxD > minD) normD = (smoothedDelayMs - minD) / (maxD - minD);
        else normD = smoothedDelayMs / 100.0;

        // Normalized Loss
        double normL = 0.0;
        if (maxL > minL) normL = (smoothedLossRate - minL) / (maxL - minL);
        else normL = smoothedLossRate;

        // Normalized Bandwidth Cost
        double normB = 0.0;
        if (maxB > minB) normB = (maxB - smoothedBandwidthMbps) / (maxB - minB);
        else normB = 1.0 / std::max(0.1, smoothedBandwidthMbps);

        double baseCost = pol.weightDelay * normD +
                          pol.weightLoss * normL +
                          pol.weightBandwidth * normB;

        double loadPenalty = classLoadOnPath * loadPenaltyWeight;
        return baseCost + loadPenalty + (congested ? 3.0 : 0.0);
    }
};

// =============================================================================
// Probe Header (16-Byte Precision UDP Probe)
// =============================================================================
class ProbeHeader : public Header {
public:
    uint32_t m_seq;
    uint64_t m_timestampNs;
    uint8_t  m_pathId;

    ProbeHeader() : m_seq(0), m_timestampNs(0), m_pathId(0) {}

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::ProbeHeader")
            .SetParent<Header>()
            .SetGroupName("AdaptiveNet")
            .AddConstructor<ProbeHeader>();
        return tid;
    }

    virtual TypeId GetInstanceTypeId(void) const override { return GetTypeId(); }
    virtual uint32_t GetSerializedSize(void) const override { return 13; }

    virtual void Serialize(Buffer::Iterator start) const override {
        start.WriteHtonU32(m_seq);
        start.WriteHtonU64(m_timestampNs);
        start.WriteU8(m_pathId);
    }

    virtual uint32_t Deserialize(Buffer::Iterator start) override {
        m_seq = start.ReadNtohU32();
        m_timestampNs = start.ReadNtohU64();
        m_pathId = start.ReadU8();
        return GetSerializedSize();
    }

    virtual void Print(std::ostream &os) const override {
        os << "probe seq=" << m_seq << " ts=" << m_timestampNs << " path=" << (uint32_t)m_pathId;
    }
};

// =============================================================================
// Global Performance & Emergency Stats
// =============================================================================
struct EmergencyStats {
    uint32_t sent = 0;
    uint32_t received = 0;
    uint32_t deadlineMisses = 0;
    double maxDelayMs = 0.0;
    double sumDelayMs = 0.0;
    std::map<uint32_t, bool> seenSeq;
} g_emergencyStats;

struct SystemMetrics {
    uint32_t switchCount = 0;
    double fileCompletionTimeSec = std::numeric_limits<double>::quiet_NaN();
} g_systemMetrics;

// =============================================================================
// Failure Event & Detection Logger
// =============================================================================
struct FailureRecord {
    std::string scenario;
    std::string mode;
    uint32_t pathId;
    double realFailTime;
    double realRestoreTime;
    double declaredDownTime;
    double declaredUpTime;
    double firstDeliveredTime[3]; // per class
    double maxInterPacketGapMs;
    uint32_t lostDuringOutage;
    bool isActive;
};

class FailureLogger {
public:
    FailureLogger() {}

    void Init(const std::string &filepath, const std::string &scenario, const std::string &mode) {
        m_filepath = filepath;
        m_scenario = scenario;
        m_mode = mode;

        bool exists = FileExistsAndNotEmpty(m_filepath);
        m_file.open(m_filepath, std::ios::app);
        if (!exists) {
            m_file << "Scenario,Mode,PathId,RealFailTime,DeclaredDownTime,DetectLatencySec,"
                   << "RealRestoreTime,DeclaredUpTime,RecoverLatencySec,EmergOutageGapSec,"
                   << "LostDuringOutage,EmergRerouteLatencySec\n";
        }
    }

    ~FailureLogger() {
        Flush();
        if (m_file.is_open()) m_file.close();
    }

    void OnRealFailure(uint32_t pathId, double t) {
        FailureRecord rec;
        rec.scenario = m_scenario;
        rec.mode = m_mode;
        rec.pathId = pathId;
        rec.realFailTime = t;
        rec.realRestoreTime = std::numeric_limits<double>::quiet_NaN();
        rec.declaredDownTime = std::numeric_limits<double>::quiet_NaN();
        rec.declaredUpTime = std::numeric_limits<double>::quiet_NaN();
        for (int i = 0; i < 3; ++i) rec.firstDeliveredTime[i] = std::numeric_limits<double>::quiet_NaN();
        rec.maxInterPacketGapMs = 0.0;
        rec.lostDuringOutage = 0;
        rec.isActive = true;
        auto old = m_records.find(pathId);
        if (old != m_records.end()) m_done.push_back(old->second);
        m_records[pathId] = rec;
    }

    void OnDeclaredDown(uint32_t pathId, double t) {
        if (m_records.find(pathId) != m_records.end()) {
            m_records[pathId].declaredDownTime = t;
        }
    }

    void OnRealRestore(uint32_t pathId, double t) {
        if (m_records.find(pathId) != m_records.end()) {
            m_records[pathId].realRestoreTime = t;
        }
    }

    void OnDeclaredUp(uint32_t pathId, double t) {
        if (m_records.find(pathId) != m_records.end()) {
            m_records[pathId].declaredUpTime = t;
        }
    }

    void OnClassPacketDelivered(TrafficClass tc, uint32_t pathId, double t) {
        for (auto &kv : m_records) {
            if (kv.second.isActive && std::isnan(kv.second.firstDeliveredTime[tc])) {
                if (t >= kv.second.declaredDownTime) {
                    kv.second.firstDeliveredTime[tc] = t;
                }
            }
        }
    }

    // Called for every emergency inter-arrival gap. The gap is charged to every
    // injected failure whose time falls inside it (fixes hard-coded path id 1).
    void RecordEmergencyOutageGap(double tPrev, double tNow, uint32_t lostPkts) {
        const double tol = 0.15;
        for (auto &kv : m_records) {
            FailureRecord &r = kv.second;
            if (tPrev <= r.realFailTime + tol && tNow > r.realFailTime) {
                r.maxInterPacketGapMs = std::max(r.maxInterPacketGapMs, (tNow - tPrev) * 1000.0);
                r.lostDuringOutage += lostPkts;
            }
        }
    }

    void Flush() {
        if (!m_file.is_open()) return;
        std::vector<FailureRecord> all = m_done;
        for (auto &kv : m_records) all.push_back(kv.second);
        for (const FailureRecord &r : all) {
            double detectLatency = (!std::isnan(r.declaredDownTime) && !std::isnan(r.realFailTime))
                                   ? (r.declaredDownTime - r.realFailTime) : std::numeric_limits<double>::quiet_NaN();
            double recoverLatency = (!std::isnan(r.declaredUpTime) && !std::isnan(r.realRestoreTime))
                                    ? (r.declaredUpTime - r.realRestoreTime) : std::numeric_limits<double>::quiet_NaN();
            double rerouteLatency = (!std::isnan(r.firstDeliveredTime[CLASS_EMERGENCY]) && !std::isnan(r.realFailTime))
                                    ? (r.firstDeliveredTime[CLASS_EMERGENCY] - r.realFailTime) : std::numeric_limits<double>::quiet_NaN();

            m_file << r.scenario << "," << r.mode << "," << r.pathId << ","
                   << std::fixed << std::setprecision(3)
                   << r.realFailTime << ","
                   << (std::isnan(r.declaredDownTime) ? -1.0 : r.declaredDownTime) << ","
                   << (std::isnan(detectLatency) ? -1.0 : detectLatency) << ","
                   << (std::isnan(r.realRestoreTime) ? -1.0 : r.realRestoreTime) << ","
                   << (std::isnan(r.declaredUpTime) ? -1.0 : r.declaredUpTime) << ","
                   << (std::isnan(recoverLatency) ? -1.0 : recoverLatency) << ","
                   << (r.maxInterPacketGapMs / 1000.0) << ","
                   << r.lostDuringOutage << ","
                   << (std::isnan(rerouteLatency) ? -1.0 : rerouteLatency) << "\n";
        }
        m_file.flush();
        m_records.clear();
        m_done.clear();
    }

private:
    std::string m_filepath;
    std::string m_scenario;
    std::string m_mode;
    std::ofstream m_file;
    std::map<uint32_t, FailureRecord> m_records;
    std::vector<FailureRecord> m_done;
} g_failureLogger;

// Forward declaration of DecisionEngine
class DecisionEngine;

// =============================================================================
// Adaptive Routing Protocol (Per-Class Dynamic Egress Steering)
// P1 & P5 Fixes: Local access device return forwarding + ToS priority classification
// =============================================================================
class AdaptiveRoutingProtocol : public Ipv4RoutingProtocol {
public:
    struct AccessSubnetRoute {
        Ipv4Address network;
        Ipv4Mask mask;
        Ptr<NetDevice> outputDevice;
        Ipv4Address gateway;
    };

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("ns3::AdaptiveRoutingProtocol")
            .SetParent<Ipv4RoutingProtocol>()
            .SetGroupName("Internet")
            .AddConstructor<AdaptiveRoutingProtocol>();
        return tid;
    }

    AdaptiveRoutingProtocol() : m_ipv4(nullptr) {}
    virtual ~AdaptiveRoutingProtocol() {}

    void SetClassRoute(TrafficClass tc, Ptr<Ipv4Route> route) {
        m_routes[tc] = route;
    }

    uint64_t TakeClassBytes(TrafficClass tc) { uint64_t b = m_classBytes[tc]; m_classBytes[tc] = 0; return b; }

    void SetSpillCallback(std::function<Ptr<Ipv4Route>(TrafficClass, Ptr<Ipv4Route>)> cb) { m_spill = cb; }

    void SetDefaultRoute(Ptr<Ipv4Route> route) {
        m_defaultRoute = route;
    }

    // Fix P1: Register internal access subnets with their matching devices and gateways
    void AddAccessRoute(Ipv4Address network, Ipv4Mask mask, Ptr<NetDevice> dev, Ipv4Address gw) {
        m_accessRoutes.push_back({network, mask, dev, gw});
    }

    void AddPathRoute(Ipv4Address destPathIp, Ipv4Address nextHopIp, Ptr<NetDevice> dev, Ipv4Address srcIp) {
        Ptr<Ipv4Route> r = Create<Ipv4Route>();
        r->SetDestination(destPathIp);
        r->SetGateway(nextHopIp);
        r->SetOutputDevice(dev);
        r->SetSource(srcIp);
        m_destToPathRoute[destPathIp] = r;
    }

    // Fix P5: Classify by ToS first, port second. Remove arbitrary size heuristic.
    TrafficClass ClassifyPacket(Ptr<const Packet> p, const Ipv4Header &header) const {
        uint8_t tos = header.GetTos();
        uint8_t dscp = (tos >> 2);

        // Priority 1: ToS / DSCP Classification
        if (dscp == 46 || tos == 0xB8) return CLASS_EMERGENCY; // DSCP EF (Expedited Forwarding)
        if (dscp == 34 || tos == 0x88) return CLASS_VIDEO;     // DSCP AF41 (Assured Forwarding 41)
        if (dscp == 10 || tos == 0x28) return CLASS_FILE;      // DSCP AF11 (Assured Forwarding 11)

        // Priority 2: Transport Protocol & Port
        if (header.GetProtocol() == 6) { // TCP is Bulk File Transfer
            return CLASS_FILE;
        }
        if (header.GetProtocol() == 17) { // UDP
            if (p && p->GetSize() >= 8) {
                UdpHeader udp;
                p->PeekHeader(udp);
                uint16_t dPort = udp.GetDestinationPort();
                uint16_t sPort = udp.GetSourcePort();
                if (dPort == 7000 || sPort == 7000) return CLASS_EMERGENCY;
                if (dPort == 5000 || sPort == 5000) return CLASS_VIDEO;
                if (dPort == 6000 || sPort == 6000) return CLASS_FILE;
                if (dPort >= 8000 && dPort <= 8007) return CLASS_DEFAULT; // Probes
            }
        }
        return CLASS_DEFAULT;
    }

    virtual Ptr<Ipv4Route> RouteOutput(Ptr<Packet> p, const Ipv4Header &header,
                                       Ptr<NetDevice> oif, Socket::SocketErrno &sockerr) override {
        // Direct route for probes and path-specific destinations
        auto pIt = m_destToPathRoute.find(header.GetDestination());
        if (pIt != m_destToPathRoute.end()) {
            if (!oif || pIt->second->GetOutputDevice() == oif) {
                sockerr = Socket::ERROR_NOTERROR;
                return pIt->second;
            }
        }

        TrafficClass tc = ClassifyPacket(p, header);
        auto it = m_routes.find(tc);
        if (it != m_routes.end() && it->second != nullptr) {
            if (!oif || it->second->GetOutputDevice() == oif) {
                sockerr = Socket::ERROR_NOTERROR;
                Ptr<Ipv4Route> outRoute = Create<Ipv4Route>();
                outRoute->SetDestination(header.GetDestination());
                outRoute->SetGateway(it->second->GetGateway());
                outRoute->SetOutputDevice(it->second->GetOutputDevice());
                outRoute->SetSource(it->second->GetSource());
                return outRoute;
            }
        }
        if (m_defaultRoute && (!oif || m_defaultRoute->GetOutputDevice() == oif)) {
            sockerr = Socket::ERROR_NOTERROR;
            Ptr<Ipv4Route> outRoute = Create<Ipv4Route>();
            outRoute->SetDestination(header.GetDestination());
            outRoute->SetGateway(m_defaultRoute->GetGateway());
            outRoute->SetOutputDevice(m_defaultRoute->GetOutputDevice());
            outRoute->SetSource(m_defaultRoute->GetSource());
            return outRoute;
        }
        sockerr = Socket::ERROR_NOROUTETOHOST;
        return nullptr;
    }

    // Fix P1: Forward return traffic to internal app nodes directly out of matching access device
    virtual bool RouteInput(Ptr<const Packet> p, const Ipv4Header &header,
                            Ptr<const NetDevice> idev, const UnicastForwardCallback &ucb,
                            const MulticastForwardCallback &mcb, const LocalDeliverCallback &lcb,
                            const ErrorCallback &ecb) override {
        uint32_t iif = m_ipv4->GetInterfaceForDevice(idev);
        if (m_ipv4->IsDestinationAddress(header.GetDestination(), iif)) {
            if (!lcb.IsNull()) {
                lcb(p, header, iif);
                return true;
            }
            return false;
        }

        // Check if destination is on an internal access subnet (App nodes: 10.0.1.0/24, 10.0.2.0/24, 10.0.3.0/24)
        for (const auto& acc : m_accessRoutes) {
            if (acc.mask.IsMatch(header.GetDestination(), acc.network)) {
                if (!ucb.IsNull()) {
                    Ptr<Ipv4Route> accRoute = Create<Ipv4Route>();
                    accRoute->SetDestination(header.GetDestination());
                    accRoute->SetGateway(acc.gateway);
                    accRoute->SetOutputDevice(acc.outputDevice);
                    uint32_t oif = m_ipv4->GetInterfaceForDevice(acc.outputDevice);
                    accRoute->SetSource(m_ipv4->GetAddress(oif, 0).GetLocal());
                    ucb(accRoute, p, header);
                    return true;
                }
                return false;
            }
        }

        // Outbound forwarding towards Destination: steer by TrafficClass
        TrafficClass tc = ClassifyPacket(p, header);
        m_classBytes[tc] += p->GetSize();
        Ptr<Ipv4Route> route = nullptr;
        auto it = m_routes.find(tc);
        if (it != m_routes.end() && it->second != nullptr) {
            route = it->second;
        } else if (m_defaultRoute) {
            route = m_defaultRoute;
        }

        if (route && m_spill && tc != CLASS_FILE) { // never reorder the TCP file flow
            Ptr<Ipv4Route> alt = m_spill(tc, route);
            if (alt) route = alt;
        }

        if (route && !ucb.IsNull()) {
            Ptr<Ipv4Route> fwdRoute = Create<Ipv4Route>();
            fwdRoute->SetDestination(header.GetDestination());
            fwdRoute->SetGateway(route->GetGateway());
            fwdRoute->SetOutputDevice(route->GetOutputDevice());
            fwdRoute->SetSource(route->GetSource());
            ucb(fwdRoute, p, header);
            return true;
        }

        // Fallback to Ipv4StaticRouting in Ipv4ListRouting
        return false;
    }

    virtual void NotifyInterfaceUp(uint32_t interface) override {}
    virtual void NotifyInterfaceDown(uint32_t interface) override {}
    virtual void NotifyAddAddress(uint32_t interface, Ipv4InterfaceAddress address) override {}
    virtual void NotifyRemoveAddress(uint32_t interface, Ipv4InterfaceAddress address) override {}
    virtual void SetIpv4(Ptr<Ipv4> ipv4) override { m_ipv4 = ipv4; }
    virtual void PrintRoutingTable(Ptr<OutputStreamWrapper> stream, Time::Unit unit) const override {}

private:
    Ptr<Ipv4> m_ipv4;
    std::map<TrafficClass, Ptr<Ipv4Route>> m_routes;
    Ptr<Ipv4Route> m_defaultRoute;
    std::vector<AccessSubnetRoute> m_accessRoutes;
    std::map<Ipv4Address, Ptr<Ipv4Route>> m_destToPathRoute;
    std::function<Ptr<Ipv4Route>(TrafficClass, Ptr<Ipv4Route>)> m_spill;
    uint64_t m_classBytes[4] = {0, 0, 0, 0};
};

class AdaptiveRoutingHelper : public Ipv4RoutingHelper {
public:
    AdaptiveRoutingHelper(Ptr<AdaptiveRoutingProtocol> proto) : m_proto(proto) {}
    virtual Ipv4RoutingHelper* Copy(void) const override { return new AdaptiveRoutingHelper(*this); }
    virtual Ptr<Ipv4RoutingProtocol> Create(Ptr<Node> node) const override { return m_proto; }
private:
    Ptr<AdaptiveRoutingProtocol> m_proto;
};

// =============================================================================
// Probe Echo Receiver Application (Destination Echoes Back on Same Path)
// =============================================================================
class ProbeEchoReceiverApp : public Application {
public:
    ProbeEchoReceiverApp() : m_port(8888) {}
    void Setup(uint16_t port) { m_port = port; }

private:
    virtual void StartApplication(void) override {
        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
        m_socket->Bind(local);
        m_socket->SetRecvCallback(MakeCallback(&ProbeEchoReceiverApp::HandleRead, this));
    }

    virtual void StopApplication(void) override {
        if (m_socket) m_socket->Close();
    }

    void HandleRead(Ptr<Socket> socket) {
        Ptr<Packet> packet;
        Address from;
        while ((packet = socket->RecvFrom(from))) {
            socket->SendTo(packet, 0, from);
        }
    }

    uint16_t m_port;
    Ptr<Socket> m_socket;
};

// =============================================================================
// Probe Sender & Monitor Application (Source Probes All 8 Paths)
// =============================================================================
class ProbeMonitorApp : public Application {
public:
    ProbeMonitorApp(const AdaptiveNetConfig &cfg,
                    std::vector<InternalPath> &paths,
                    DecisionEngine *engine)
        : m_cfg(cfg), m_paths(paths), m_engine(engine), m_running(false)
    {
        for (uint32_t k = 0; k < 8; ++k) {
            m_probeSeq[k] = 0;
            m_consecutiveLosses[k] = 0;
            m_consecutiveHits[k] = 0;
            m_probesSentWindow[k] = 0;
            m_probesLostWindow[k] = 0;
        }
    }

    void SetupSockets(Ptr<Node> sourceNode) {
        for (uint32_t k = 0; k < m_paths.size(); ++k) {
            Ptr<Socket> sock = Socket::CreateSocket(sourceNode, UdpSocketFactory::GetTypeId());
            InetSocketAddress local = InetSocketAddress(m_paths[k].srcIp, 0);
            sock->Bind(local);
            sock->SetRecvCallback(MakeCallback(&ProbeMonitorApp::HandleProbeEcho, this));
            m_sockets[k] = sock;
        }
    }

private:
    virtual void StartApplication(void) override {
        m_running = true;
        ScheduleNextProbe();
    }

    virtual void StopApplication(void) override {
        m_running = false;
        if (m_probeEvent.IsPending()) Simulator::Cancel(m_probeEvent);
        for (auto &kv : m_sockets) {
            if (kv.second) kv.second->Close();
        }
    }

    void ScheduleNextProbe() {
        if (!m_running) return;
        m_probeEvent = Simulator::Schedule(Seconds(m_cfg.probeIntervalSec), &ProbeMonitorApp::SendProbes, this);
    }

    void SendProbes() {
        if (!m_running) return;
        uint64_t nowNs = Simulator::Now().GetNanoSeconds();

        // 1. Check for unacknowledged probes (timeouts)
        for (uint32_t k = 0; k < m_paths.size(); ++k) {
            auto it = m_unackedProbes[k].begin();
            while (it != m_unackedProbes[k].end()) {
                double ageSec = (nowNs - it->second) / 1e9;
                // Timeout must exceed the path's real RTT (2 links each way => 4 x link delay),
                // otherwise slow paths (R6/R7) are falsely declared DOWN.
                double rttBaseSec = 4.0 * m_paths[k].baseDelayMs / 1000.0;
                double timeoutSec = std::max(m_cfg.probeIntervalSec * 1.8, rttBaseSec * 1.5 + 0.05);
                if (ageSec > timeoutSec) {
                    OnProbeLost(k);
                    it = m_unackedProbes[k].erase(it);
                } else {
                    ++it;
                }
            }
        }

        // 2. Transmit next probe down EACH of the 8 paths
        for (uint32_t k = 0; k < m_paths.size(); ++k) {
            uint32_t seq = ++m_probeSeq[k];
            ProbeHeader ph;
            ph.m_pathId = k;
            ph.m_seq = seq;
            ph.m_timestampNs = nowNs;

            uint32_t payload = (m_cfg.probePacketSize > 16) ? (m_cfg.probePacketSize - 16) : 1;
            Ptr<Packet> p = Create<Packet>(payload);
            p->AddHeader(ph);

            m_unackedProbes[k][seq] = nowNs;
            m_probesSentWindow[k]++;

            InetSocketAddress destAddr(m_paths[k].destPathIp, m_cfg.probePort);
            m_sockets[k]->SendTo(p, 0, destAddr);
        }

        ScheduleNextProbe();
    }

    void HandleProbeEcho(Ptr<Socket> socket) {
        Ptr<Packet> packet;
        Address from;
        while ((packet = socket->RecvFrom(from))) {
            ProbeHeader ph;
            if (packet->RemoveHeader(ph) > 0) {
                uint32_t k = ph.m_pathId;
                if (k < m_paths.size()) {
                    uint64_t nowNs = Simulator::Now().GetNanoSeconds();
                    auto it = m_unackedProbes[k].find(ph.m_seq);
                    if (it != m_unackedProbes[k].end()) {
                        double rttMs = (nowNs - ph.m_timestampNs) / 1e6;
                        double delaySample = rttMs / 2.0;
                        m_unackedProbes[k].erase(it);

                        // Live EMA Delay Update
                        m_paths[k].smoothedDelayMs = m_cfg.emaAlpha * delaySample +
                                                    (1.0 - m_cfg.emaAlpha) * m_paths[k].smoothedDelayMs;

                        OnProbeSuccess(k);
                    }
                }
            }
        }
    }

    void OnProbeLost(uint32_t pathId);
    void OnProbeSuccess(uint32_t pathId);

    const AdaptiveNetConfig &m_cfg;
    std::vector<InternalPath> &m_paths;
    DecisionEngine *m_engine;
    bool m_running;
    EventId m_probeEvent;

    std::map<uint32_t, Ptr<Socket>> m_sockets;
    uint32_t m_probeSeq[8];
    uint32_t m_consecutiveLosses[8];
    uint32_t m_consecutiveHits[8];
    uint32_t m_probesSentWindow[8];
    uint32_t m_probesLostWindow[8];
    std::map<uint32_t, std::map<uint32_t, uint64_t>> m_unackedProbes;
};

// =============================================================================
// Centralized Decision Engine (Compatible-Shortest Route Failover & Hysteresis)
// =============================================================================
class DecisionEngine {
public:
    DecisionEngine(const AdaptiveNetConfig &cfg,
                   Ptr<Node> source,
                   Ptr<Node> destination,
                   Ptr<AdaptiveRoutingProtocol> sourceRouting,
                   Ptr<Ipv4StaticRouting> destStatic,
                   std::vector<InternalPath> &paths,
                   AnimationInterface *anim)
        : m_cfg(cfg),
          m_source(source),
          m_destination(destination),
          m_sourceRouting(sourceRouting),
          m_destStatic(destStatic),
          m_paths(paths),
          m_anim(anim)
    {
        m_policies[CLASS_EMERGENCY] = m_cfg.policyEmergency;
        m_policies[CLASS_VIDEO]     = m_cfg.policyVideo;
        m_policies[CLASS_FILE]      = m_cfg.policyFile;

        // Initial default assignments (R0=Emergency, R1=Video, R2=File)
        m_activePathIdx[CLASS_EMERGENCY] = 0;
        m_activePathIdx[CLASS_VIDEO]     = 1;
        m_activePathIdx[CLASS_FILE]      = 2;

        for (int c = 0; c < 3; ++c) {
            m_lastSwitchTime[(TrafficClass)c] = Seconds(0.0);
        }

        {
            std::string cp = m_cfg.resultsDir + "/congestion_events.csv";
            bool ex = FileExistsAndNotEmpty(cp);
            m_congFile.open(cp, std::ios::app);
            if (!ex) m_congFile << "Time,Path,UtilPct,Drops,ClassesOnPath,Action\n";
        }
        std::string decPath = m_cfg.resultsDir + "/decisions.csv";
        bool exists = FileExistsAndNotEmpty(decPath);
        m_decisionsFile.open(decPath, std::ios::app);
        if (!exists) {
            m_decisionsFile << "Time,Class,OldPath,NewPath,Reason,CostBefore,CostAfter\n";
        }
    }

    ~DecisionEngine() {
        if (m_decisionsFile.is_open()) m_decisionsFile.close();
    }

    void StartMonitoring() {
        // Initial setup of routing tables
        for (auto &kv : m_activePathIdx) {
            TrafficClass tc = kv.first;
            uint32_t pIdx = kv.second;
            Ptr<Ipv4Route> r = Create<Ipv4Route>();
            r->SetDestination(m_paths[pIdx].destPathIp);
            r->SetGateway(m_paths[pIdx].nextHopIp);
            r->SetOutputDevice(m_paths[pIdx].outputDevice);
            r->SetSource(m_paths[pIdx].srcIp);
            m_sourceRouting->SetClassRoute(tc, r);

            UpdateDestinationReturnRoute(tc, pIdx);
        }
        m_sourceRouting->SetSpillCallback([this](TrafficClass tc, Ptr<Ipv4Route> pr) { return SpillRoute(tc, pr); });
        if (m_anim) m_bufCounter = m_anim->AddNodeCounter("Router buffer occupancy %", AnimationInterface::DOUBLE_COUNTER);
        UpdateNetAnimHighlights();
        Simulator::Schedule(Seconds(m_cfg.monitorIntervalSec), &DecisionEngine::SampleAndEvaluate, this);
        Simulator::Schedule(Seconds(m_cfg.bufSampleSec), &DecisionEngine::BufferSample, this);
    }

    // ---------------- Buffer visualisation & spill-over routing ----------------
    double Occupancy(uint32_t k) const {
        const Ptr<QueueDisc> &q = m_paths[k].qdisc;
        if (!q) return 0.0;
        double lim = q->GetMaxSize().GetValue();
        return lim > 0 ? std::min(1.0, q->GetNPackets() / lim) : 0.0;
    }

    uint32_t PathOfRoute(Ptr<Ipv4Route> r) const {
        for (uint32_t k = 0; k < m_paths.size(); ++k)
            if (m_paths[k].outputDevice == r->GetOutputDevice()) return k;
        return 99;
    }

    bool IsIdle(uint32_t k) const {   // standby = carries no class and no recent spill
        for (auto &kv : m_activePathIdx) if (kv.second == k) return false;
        return (Simulator::Now().GetSeconds() - m_paths[k].lastSpillAt) > 1.0;
    }

    // Called for every packet of Emergency/Video before it is queued. While the primary
    // router's buffer is nearly full, the packet is sent via the lowest-latency STANDBY
    // router whose own buffer is still empty enough.
    Ptr<Ipv4Route> SpillRoute(TrafficClass tc, Ptr<Ipv4Route> primary) {
        if (!m_cfg.enableSpill || m_cfg.mode != "adaptive") return nullptr;
        uint32_t k = PathOfRoute(primary);
        if (k >= m_paths.size() || m_paths[k].isDown) return nullptr;
        double occ = Occupancy(k);
        double now = Simulator::Now().GetSeconds();
        if (!m_paths[k].spilling && occ >= m_cfg.spillHigh - 1e-9) {
            m_paths[k].spilling = true;
            m_paths[k].spillPkts = 0;
        } else if (m_paths[k].spilling && occ <= m_cfg.spillLow) {
            m_paths[k].spilling = false;
            if (m_spillAnnounced[k]) {
                m_spillAnnounced[k] = false;
                std::cout << "[BUFFER] t=" << std::fixed << std::setprecision(2) << now << "s R" << k
                          << " buffer drained -> back on primary (spill episode finished)\n";
            }
            return nullptr;
        }
        if (!m_paths[k].spilling) return nullptr;

        // choose target: alive, not congested, eligible, buffer < 50%; prefer idle standby, lowest delay
        int best = -1; bool bestIdle = false;
        for (uint32_t i = 0; i < m_paths.size(); ++i) {
            if (i == k || m_paths[i].isDown || m_paths[i].congested) continue;
            if (!IsPathEligible(tc, i) || Occupancy(i) >= 0.5) continue;
            bool idle = IsIdle(i) || m_paths[i].spillMask != 0;   // already-used spill target counts as standby
            if (best < 0 || (idle && !bestIdle) ||
                (idle == bestIdle && m_paths[i].smoothedDelayMs < m_paths[best].smoothedDelayMs)) {
                best = i; bestIdle = idle;
            }
        }
        if (best < 0) return nullptr;
        InternalPath &t = m_paths[best];
        t.lastSpillAt = now;
        t.spillMask |= (1u << tc);
        m_paths[k].spillPkts++;
        m_paths[k].spillTarget = best;
        m_spillTotal++;
        if (!m_spillAnnounced[k] && now - m_lastAnnounce[k] > 2.0) {
            m_spillAnnounced[k] = true;
            m_lastAnnounce[k] = now;
            std::ostringstream o;
            o << "R" << k << " buffer " << std::fixed << std::setprecision(0) << occ * 100 << "% full -> "
              << TrafficClassToString(tc) << " overflow spills to STANDBY R" << best << " (lowest latency, "
              << std::setprecision(1) << t.smoothedDelayMs << " ms)";
            Notify(o.str());
            m_congFile << std::fixed << std::setprecision(2) << now << ",R" << k << "," << m_paths[k].utilPct
                       << ",0," << ClassesOn(k) << ",SPILL to R" << best << "\n";
            m_congFile.flush();
        }
        Ptr<Ipv4Route> r = Create<Ipv4Route>();
        r->SetDestination(t.destPathIp);
        r->SetGateway(t.nextHopIp);
        r->SetOutputDevice(t.outputDevice);
        r->SetSource(t.srcIp);
        return r;
    }

    void BufferSample() {
        double now = Simulator::Now().GetSeconds();
        if (!m_bufFile.is_open()) {
            m_bufFile.open(m_cfg.resultsDir + "/buffer_trace.csv");
            m_bufFile << "Time,R0,R1,R2,R3,R4,R5,R6,R7\n";
        }
        m_bufFile << std::fixed << std::setprecision(2) << now;
        for (uint32_t k = 0; k < m_paths.size(); ++k) {
            double occ = Occupancy(k);
            m_paths[k].bufPct = occ * 100.0;
            m_bufFile << "," << std::setprecision(0) << m_paths[k].bufPct << std::setprecision(2);
            if (now - m_paths[k].lastSpillAt > 1.0) m_paths[k].spillMask = 0;
            if (m_anim && std::fabs(m_paths[k].bufPct - m_lastCounter[k]) >= 5.0) {
                m_anim->UpdateNodeCounter(m_bufCounter, RouterNode(k)->GetId(), m_paths[k].bufPct);
                m_lastCounter[k] = m_paths[k].bufPct;
            }
        }
        m_bufFile << "\n";
        UpdateNetAnimHighlights();
        Simulator::Schedule(Seconds(m_cfg.bufSampleSec), &DecisionEngine::BufferSample, this);
    }

    void UpdateDestinationReturnRoute(TrafficClass tc, uint32_t pathId) {
        Ipv4Address appSubnet;
        if (tc == CLASS_EMERGENCY) appSubnet = Ipv4Address("10.0.1.0");
        else if (tc == CLASS_VIDEO) appSubnet = Ipv4Address("10.0.2.0");
        else if (tc == CLASS_FILE) appSubnet = Ipv4Address("10.0.3.0");
        else return;

        for (uint32_t r = 0; r < m_destStatic->GetNRoutes(); ++r) {
            Ipv4RoutingTableEntry entry = m_destStatic->GetRoute(r);
            if (entry.GetDestNetwork() == appSubnet) {
                m_destStatic->RemoveRoute(r);
                break;
            }
        }
        m_destStatic->AddNetworkRouteTo(appSubnet, Ipv4Mask("255.255.255.0"),
                                       m_paths[pathId].returnHopIp,
                                       m_paths[pathId].returnInterface, 1);
    }

    uint32_t GetActivePath(TrafficClass tc) const {
        auto it = m_activePathIdx.find(tc);
        return (it != m_activePathIdx.end()) ? it->second : 0;
    }

    // Eligibility Checking for Compatible-Shortest Failover
    bool IsPathEligible(TrafficClass tc, uint32_t pathId) const {
        if (m_paths[pathId].isDown || m_paths[pathId].congested) return false;

        if (tc == CLASS_EMERGENCY) {
            double maxAllowedDelay = m_cfg.emergDeadlineMs / 2.0;
            return (m_paths[pathId].smoothedDelayMs <= maxAllowedDelay &&
                    m_paths[pathId].smoothedLossRate <= m_cfg.emergMaxLossEligible);
        } else if (tc == CLASS_VIDEO) {
            double videoRateMbps = DataRate(m_cfg.videoDataRate).GetBitRate() / 1e6;
            double reqBandwidth = videoRateMbps * m_cfg.videoHeadroom;
            return (m_paths[pathId].smoothedBandwidthMbps >= reqBandwidth);
        } else if (tc == CLASS_FILE) {
            return (m_paths[pathId].smoothedLossRate <= m_cfg.fileMaxLossEligible &&
                    m_paths[pathId].smoothedBandwidthMbps >= m_cfg.fileMinRateEligibleMbps);
        }
        return true;
    }

    uint32_t SelectBestFromCandidates(TrafficClass tc, const std::vector<uint32_t>& candidates) {
        double minD = 1e9, maxD = -1e9;
        double minB = 1e9, maxB = -1e9;
        double minL = 1e9, maxL = -1e9;
        for (uint32_t p : candidates) {
            minD = std::min(minD, m_paths[p].smoothedDelayMs);
            maxD = std::max(maxD, m_paths[p].smoothedDelayMs);
            minB = std::min(minB, m_paths[p].smoothedBandwidthMbps);
            maxB = std::max(maxB, m_paths[p].smoothedBandwidthMbps);
            minL = std::min(minL, m_paths[p].smoothedLossRate);
            maxL = std::max(maxL, m_paths[p].smoothedLossRate);
        }

        const ClassPolicy& pol = m_policies[tc];
        uint32_t bestIdx = candidates[0];
        double minCost = 1e9;

        for (uint32_t p : candidates) {
            uint32_t candidateLoad = 0;
            for (auto& kv : m_activePathIdx) {
                if (kv.first != tc && kv.second == p) candidateLoad++;
            }
            double cost = m_paths[p].ComputeCost(pol, minD, maxD, minB, maxB, minL, maxL,
                                                 candidateLoad, m_cfg.loadPenaltyWeight);
            if (cost < minCost) {
                minCost = cost;
                bestIdx = p;
            }
        }
        return bestIdx;
    }

    double ComputePathCost(TrafficClass tc, uint32_t pIdx) {
        double minD = 1e9, maxD = -1e9;
        double minB = 1e9, maxB = -1e9;
        double minL = 1e9, maxL = -1e9;
        for (const auto& p : m_paths) {
            if (p.isDown) continue;
            minD = std::min(minD, p.smoothedDelayMs);
            maxD = std::max(maxD, p.smoothedDelayMs);
            minB = std::min(minB, p.smoothedBandwidthMbps);
            maxB = std::max(maxB, p.smoothedBandwidthMbps);
            minL = std::min(minL, p.smoothedLossRate);
            maxL = std::max(maxL, p.smoothedLossRate);
        }
        uint32_t load = 0;
        for (auto& kv : m_activePathIdx) {
            if (kv.first != tc && kv.second == pIdx) load++;
        }
        return m_paths[pIdx].ComputeCost(m_policies[tc], minD, maxD, minB, maxB, minL, maxL,
                                         load, m_cfg.loadPenaltyWeight);
    }

    Ptr<Node> RouterNode(uint32_t k) const {
        return m_paths[k].outputDevice->GetChannel()->GetDevice(1)->GetNode();
    }

    // Only writes to NetAnim when colour/label really changed (keeps XML small).
    void SetRouterLook(uint32_t k, uint8_t r, uint8_t g, uint8_t b, const std::string &label) {
        if (!m_anim) return;
        uint32_t key = (uint32_t(r) << 16) | (uint32_t(g) << 8) | b;
        auto it = m_look.find(k);
        if (it != m_look.end() && it->second.first == key && it->second.second == label) return;
        Ptr<Node> n = RouterNode(k);
        m_anim->UpdateNodeColor(n, r, g, b);
        m_anim->UpdateNodeDescription(n, label);
        m_look[k] = std::make_pair(key, label);
    }

    void OnPathFailureDiscovered(uint32_t pathId) {
        std::cout << "[DecisionEngine] t=" << Simulator::Now().GetSeconds() << "s path R" << pathId << " declared DOWN (probe timeouts)\n";
        m_paths[pathId].isDown = true;
        g_failureLogger.OnDeclaredDown(pathId, Simulator::Now().GetSeconds());
        SetRouterLook(pathId, 0, 0, 0, "NODE FAILED [R" + std::to_string(pathId) + "]");

        if (m_cfg.mode != "baseline1") {
            bool affected = false;
            for (auto &kv : m_activePathIdx) {
                if (kv.second == pathId) { affected = true; break; }
            }
            if (affected) ExecuteReRouteOnFailure(pathId);
        }
        UpdateNetAnimHighlights();
    }

    void OnPathRecoveryDiscovered(uint32_t pathId) {
        m_paths[pathId].isDown = false;
        m_recoveredAt[pathId] = Simulator::Now().GetSeconds();
        g_failureLogger.OnDeclaredUp(pathId, Simulator::Now().GetSeconds());
        UpdateNetAnimHighlights();
    }

    // Router colour = colour of the application using it:
    //   Emergency = RED, Video = BLUE, File = GREEN, failed = BLACK,
    //   just recovered = YELLOW (1.5 s), idle = GREY.
    // A router shared by several classes takes the highest-priority colour and
    // lists all classes in its label.
    void UpdateNetAnimHighlights() {
        if (!m_anim) return;
        double now = Simulator::Now().GetSeconds();
        for (uint32_t k = 0; k < 8; ++k) {
            std::string tag = "R" + std::to_string(k);
            if (m_paths[k].isDown) {
                SetRouterLook(k, 0, 0, 0, "NODE FAILED [" + tag + "]");
                continue;
            }
            std::string cg = m_paths[k].congested ? " !! CONGESTED" : "";
            bool e = (m_activePathIdx[CLASS_EMERGENCY] == k);
            bool v = (m_activePathIdx[CLASS_VIDEO] == k);
            bool f = (m_activePathIdx[CLASS_FILE] == k);
            std::string spillTag;
            if (m_paths[k].spillMask & 1u) { e = true; spillTag = " (spill)"; }
            if (m_paths[k].spillMask & 2u) { v = true; spillTag = " (spill)"; }
            if (e || v || f) {
                std::string who;
                if (e) who += "EMERGENCY";
                if (v) who += std::string(who.empty() ? "" : "+") + "VIDEO";
                if (f) who += std::string(who.empty() ? "" : "+") + "FILE";
                if (e)      SetRouterLook(k, 255, 0, 0, tag + " [" + who + "]" + spillTag + cg);
                else if (v) SetRouterLook(k, 0, 100, 255, tag + " [" + who + "]" + spillTag + cg);
                else        SetRouterLook(k, 0, 200, 0, tag + " [" + who + "]" + spillTag + cg);
            } else if (m_recoveredAt.count(k) && now - m_recoveredAt[k] < 1.5) {
                SetRouterLook(k, 255, 215, 0, "RECOVERED [" + tag + "]");
            } else {
                SetRouterLook(k, 180, 180, 180, tag + " [Standby]" + cg);
            }
        }
        UpdateSourceBanner();
        UpdateLinkLabels();
    }

    // ---- Congestion monitoring & notification ----------------------------------
    std::string ClassesOn(uint32_t k) const {
        std::string w;
        auto add = [&](TrafficClass c, const char *n) {
            auto it = m_activePathIdx.find(c);
            if (it != m_activePathIdx.end() && it->second == k) w += std::string(w.empty() ? "" : "+") + n;
        };
        add(CLASS_EMERGENCY, "EMERG"); add(CLASS_VIDEO, "VIDEO"); add(CLASS_FILE, "FILE");
        if (m_paths[k].spillMask & 1u) w += std::string(w.empty() ? "" : "+") + "~EMERG";
        if (m_paths[k].spillMask & 2u) w += std::string(w.empty() ? "" : "+") + "~VIDEO";
        return w.empty() ? "standby" : w;
    }

    void UpdateLinkLabels() {
        if (!m_anim) return;
        for (uint32_t k = 0; k < 8; ++k) {
            std::ostringstream o;
            int n = (int)std::lround(std::min(100.0, m_paths[k].bufPct) / 10.0);
            o << "R" << k << " " << ClassesOn(k) << " | buffer [" << std::string(n, '#') << std::string(10 - n, '-')
              << "] " << std::fixed << std::setprecision(0) << (n * 10) << "% | link "
              << m_paths[k].utilPct << "% of " << m_paths[k].baseBandwidthMbps << "Mbps";
            if (m_paths[k].spilling) o << " | SPILLING->R" << m_paths[k].spillTarget;
            if (m_paths[k].isDown) o.str("R" + std::to_string(k) + " DOWN");
            else if (m_paths[k].congested) o << " CONGESTED";
            std::string t = o.str();
            if (m_linkText[k] != t) {
                m_anim->UpdateLinkDescription(m_source, RouterNode(k), t);
                m_linkText[k] = t;
            }
        }
    }

    void UpdateSourceBanner() {
        if (!m_anim) return;
        bool alert = Simulator::Now().GetSeconds() < m_alertUntil;
        std::string t = alert ? ("Source Router | ALERT: " + m_alertText) : "Source Router";
        if (t == m_lookSource) return;
        m_anim->UpdateNodeDescription(m_source, t);
        if (alert) m_anim->UpdateNodeColor(m_source, 255, 140, 0);
        else       m_anim->UpdateNodeColor(m_source, 0, 0, 255);
        m_lookSource = t;
    }

    void Notify(const std::string &msg) {
        std::cout << "[CONGESTION] t=" << std::fixed << std::setprecision(2) << Simulator::Now().GetSeconds()
                  << "s " << msg << "\n";
        m_alertText = msg;
        m_alertUntil = Simulator::Now().GetSeconds() + 4.0;
    }

    void EvaluateCongestion() {
        for (uint32_t k = 0; k < m_paths.size(); ++k) {
            InternalPath &p = m_paths[k];
            double util = (p.utilTxBytes * 8.0) / (m_cfg.monitorIntervalSec * 1e6 * p.baseBandwidthMbps);
            p.utilPct = std::min(100.0, util * 100.0);
            uint32_t drops = p.intervalDrops;
            p.offeredMbps = (p.offeredBytes * 8.0) / (m_cfg.monitorIntervalSec * 1e6);
            p.offeredBytes = 0;
            p.utilTxBytes = 0; p.intervalDrops = 0;
            if (p.isDown) { p.congested = false; continue; }

            bool onlyFile = (ClassesOn(k) == "FILE");   // TCP probing its own link is normal
            if (onlyFile) { p.congested = false; continue; }
            bool bad = (drops >= m_cfg.congDropThreshold) || (util >= m_cfg.congUtilThreshold && drops > 0);
            if (bad) {
                p.cleanIntervals = 0;
                if (!p.congested) {
                    p.congested = true;
                    std::string who = ClassesOn(k);
                    std::string action = "none";
                    if (m_cfg.mode == "adaptive") action = RerouteForCongestion(k);
                    std::ostringstream o;
                    o << "R" << k << " congested (" << std::fixed << std::setprecision(0) << p.utilPct
                      << "% util, " << drops << " drops) carrying " << who << " -> " << action;
                    Notify(o.str());
                    m_congFile << std::fixed << std::setprecision(2) << Simulator::Now().GetSeconds() << ",R" << k
                               << "," << p.utilPct << "," << drops << "," << who << "," << action << "\n";
                    m_congFile.flush();
                }
            } else if (p.congested && util < 0.8 && ++p.cleanIntervals >= 2) {
                p.congested = false;
                std::cout << "[CONGESTION] t=" << Simulator::Now().GetSeconds() << "s R" << k << " congestion cleared\n";
            }
        }
    }

    // Move every class (except a lone TCP file flow that only congests itself) to the
    // best compatible, non-congested, alive path.
    std::string RerouteForCongestion(uint32_t k) {
        std::ostringstream moved;
        std::vector<TrafficClass> order = {CLASS_EMERGENCY, CLASS_VIDEO, CLASS_FILE};
        int onPath = 0;
        for (TrafficClass tc : order) if (m_activePathIdx[tc] == k) onPath++;
        for (TrafficClass tc : order) {
            if (m_activePathIdx[tc] != k) continue;
            if (tc == CLASS_FILE && onPath == 1) continue; // TCP self-congestion: keep, no flapping
            std::vector<uint32_t> cand;
            for (uint32_t i = 0; i < m_paths.size(); ++i)
                if (i != k && !m_paths[i].isDown && IsPathEligible(tc, i) &&
                    HasCapacity(tc, i)) // enough capacity for the class's offered load
                    cand.push_back(i);
            if (cand.empty()) continue;
            std::vector<uint32_t> pool;                      // prefer idle STANDBY routers
            for (uint32_t i : cand) if (IsIdle(i)) pool.push_back(i);
            if (pool.empty()) pool = cand;
            uint32_t best = pool[0];
            if (tc == CLASS_EMERGENCY) {
                for (uint32_t i : pool) if (m_paths[i].smoothedDelayMs < m_paths[best].smoothedDelayMs) best = i;
            } else best = SelectBestFromCandidates(tc, pool);
            PerformSwitch(tc, k, best, 0, 0, "CONGESTION");
            moved << (moved.str().empty() ? "" : ", ") << TrafficClassToString(tc) << " moved R" << k << "->R" << best;
        }
        return moved.str().empty() ? "no move needed" : moved.str();
    }

    bool HasCapacity(TrafficClass tc, uint32_t i) const {
        if (tc == CLASS_FILE) return true;                     // TCP adapts to whatever it gets
        return m_paths[i].baseBandwidthMbps >= 1.1 * m_classRateMbps[tc];
    }

    void SampleAndEvaluate() {
        for (int c = 0; c < 3; ++c) {
            double r = (m_sourceRouting->TakeClassBytes((TrafficClass)c) * 8.0) / (m_cfg.monitorIntervalSec * 1e6);
            m_classRateMbps[c] = 0.5 * r + 0.5 * m_classRateMbps[c];
        }
        EvaluateCongestion();
        for (auto &p : m_paths) {
            if (p.isDown) {
                p.smoothedBandwidthMbps = 0.0;
                continue;
            }
            double elapsedSec = m_cfg.monitorIntervalSec;
            double tputMbps = m_cfg.measureBw ? (p.intervalTxBytes * 8.0) / (elapsedSec * 1e6) : 0.0;
            p.intervalTxBytes = 0;

            double capacityMbps = p.baseBandwidthMbps;
            double availBwSample = std::max(0.1, capacityMbps - tputMbps);
            p.smoothedBandwidthMbps = m_cfg.emaAlpha * availBwSample + (1.0 - m_cfg.emaAlpha) * p.smoothedBandwidthMbps;
        }

        if (m_cfg.mode == "adaptive") {
            ExecuteAdaptiveDecisions();
        } else if (m_cfg.mode == "baseline2") {
            ExecuteBaseline2Decisions();
        } else if (m_cfg.mode == "shortest") {
            ExecuteShortestPathDecisions();
        }

        UpdateNetAnimHighlights();
        Simulator::Schedule(Seconds(m_cfg.monitorIntervalSec), &DecisionEngine::SampleAndEvaluate, this);
    }

    // New Behavior: Compatible-Shortest Route on Failure
    void ExecuteReRouteOnFailure(uint32_t failedPathId) {
        std::vector<TrafficClass> priorityOrder = {CLASS_EMERGENCY, CLASS_VIDEO, CLASS_FILE};

        std::vector<uint32_t> alivePaths;
        for (uint32_t i = 0; i < m_paths.size(); ++i) {
            if (!m_paths[i].isDown) alivePaths.push_back(i);
        }

        if (alivePaths.empty()) {
            std::cout << "[DecisionEngine] ALL 8 PATHS DOWN! Queueing/dropping traffic safely.\n";
            return;
        }

        for (TrafficClass tc : priorityOrder) {
            if (m_activePathIdx[tc] == failedPathId) {
                std::vector<uint32_t> eligiblePaths;
                for (uint32_t p : alivePaths) {
                    if (IsPathEligible(tc, p)) eligiblePaths.push_back(p);
                }

                std::string reason = "PATH_DOWN";
                const std::vector<uint32_t>& candidatePool = (!eligiblePaths.empty()) ? eligiblePaths : alivePaths;
                if (eligiblePaths.empty()) {
                    reason = "NO_COMPATIBLE_PATH";
                }

                uint32_t bestIdx = candidatePool[0];
                if (tc == CLASS_EMERGENCY || m_cfg.mode == "shortest" || m_cfg.mode == "baseline2") {
                    double minD = 1e9;
                    for (uint32_t p : candidatePool) {
                        if (m_paths[p].smoothedDelayMs < minD) {
                            minD = m_paths[p].smoothedDelayMs;
                            bestIdx = p;
                        }
                    }
                } else {
                    bestIdx = SelectBestFromCandidates(tc, candidatePool);
                }

                double oldCost = 1e9;
                double newCost = (tc == CLASS_EMERGENCY) ? m_paths[bestIdx].smoothedDelayMs : ComputePathCost(tc, bestIdx);
                PerformSwitch(tc, failedPathId, bestIdx, oldCost, newCost, reason);
            }
        }
        UpdateNetAnimHighlights();
    }

    uint32_t SelectBestPathForClass(TrafficClass tc) {
        std::vector<uint32_t> alivePaths;
        for (uint32_t i = 0; i < m_paths.size(); ++i) {
            if (!m_paths[i].isDown) alivePaths.push_back(i);
        }
        if (alivePaths.empty()) return 0;
        std::vector<uint32_t> clear;
        for (uint32_t i : alivePaths) if (!m_paths[i].congested && HasCapacity(tc, i)) clear.push_back(i);
        if (clear.empty()) for (uint32_t i : alivePaths) if (HasCapacity(tc, i)) clear.push_back(i);
        return SelectBestFromCandidates(tc, clear.empty() ? alivePaths : clear);
    }

    void ExecuteAdaptiveDecisions() {
        Time now = Simulator::Now();
        std::vector<TrafficClass> priorityOrder = {CLASS_EMERGENCY, CLASS_VIDEO, CLASS_FILE};

        for (TrafficClass tc : priorityOrder) {
            bool holdExpired = (now - m_lastSwitchTime[tc] >= Seconds(m_cfg.holdTimeSec));
            uint32_t curIdx = m_activePathIdx[tc];
            double curCost = ComputePathCost(tc, curIdx);

            uint32_t bestIdx = SelectBestPathForClass(tc);
            double minCost = ComputePathCost(tc, bestIdx);

            if (bestIdx != curIdx) {
                bool shouldSwitch = false;
                if (m_paths[curIdx].isDown) {
                    shouldSwitch = true;
                } else if (holdExpired && minCost < curCost * (1.0 - m_cfg.hysteresisMargin)) {
                    shouldSwitch = true;
                }

                if (shouldSwitch) {
                    std::string reason = m_paths[curIdx].isDown ? "PATH_DOWN" : "UtilityOptimization";
                    PerformSwitch(tc, curIdx, bestIdx, curCost, minCost, reason);
                }
            }
        }
    }

    void ExecuteBaseline2Decisions() {
        uint32_t lowestDelayIdx = 0;
        double minD = 1e9;
        for (uint32_t i = 0; i < m_paths.size(); ++i) {
            if (m_paths[i].isDown) continue;
            if (m_paths[i].smoothedDelayMs < minD) {
                minD = m_paths[i].smoothedDelayMs;
                lowestDelayIdx = i;
            }
        }

        for (auto& kv : m_policies) {
            TrafficClass tc = kv.first;
            uint32_t curIdx = m_activePathIdx[tc];
            if (curIdx != lowestDelayIdx && (m_paths[curIdx].isDown || minD < m_paths[curIdx].smoothedDelayMs * 0.85)) {
                std::string reason = m_paths[curIdx].isDown ? "PATH_DOWN" : "DelayOnlyBaseline2";
                PerformSwitch(tc, curIdx, lowestDelayIdx, 0, 0, reason);
            }
        }
    }

    void ExecuteShortestPathDecisions() {
        uint32_t shortestIdx = 0;
        double minD = 1e9;
        for (uint32_t i = 0; i < m_paths.size(); ++i) {
            if (m_paths[i].isDown) continue;
            if (m_paths[i].smoothedDelayMs < minD) {
                minD = m_paths[i].smoothedDelayMs;
                shortestIdx = i;
            }
        }

        for (auto& kv : m_policies) {
            TrafficClass tc = kv.first;
            uint32_t curIdx = m_activePathIdx[tc];
            if (m_paths[curIdx].isDown) {
                PerformSwitch(tc, curIdx, shortestIdx, 0, 0, "PATH_DOWN");
            }
        }
    }

    void PerformSwitch(TrafficClass tc, uint32_t oldIdx, uint32_t newIdx,
                        double oldCost, double newCost, std::string reason) {
        m_activePathIdx[tc] = newIdx;
        m_lastSwitchTime[tc] = Simulator::Now();
        UpdateNetAnimHighlights();
        g_systemMetrics.switchCount++;

        // Update egress route on Source Router
        Ptr<Ipv4Route> r = Create<Ipv4Route>();
        r->SetDestination(m_paths[newIdx].destPathIp);
        r->SetGateway(m_paths[newIdx].nextHopIp);
        r->SetOutputDevice(m_paths[newIdx].outputDevice);
        r->SetSource(m_paths[newIdx].srcIp);
        m_sourceRouting->SetClassRoute(tc, r);

        // Update symmetric return route for this class subnet on Destination
        UpdateDestinationReturnRoute(tc, newIdx);

        m_decisionsFile << std::fixed << std::setprecision(2)
                        << Simulator::Now().GetSeconds() << ","
                        << TrafficClassToString(tc) << ","
                        << m_paths[oldIdx].name << ","
                        << m_paths[newIdx].name << ","
                        << reason << ","
                        << oldCost << "," << newCost << "\n";
        m_decisionsFile.flush();
    }

    const AdaptiveNetConfig &m_cfg;
    Ptr<Node> m_source;
    Ptr<Node> m_destination;
    Ptr<AdaptiveRoutingProtocol> m_sourceRouting;
    Ptr<Ipv4StaticRouting> m_destStatic;
    std::vector<InternalPath>& m_paths;
    AnimationInterface* m_anim;

    std::map<TrafficClass, Time> m_lastSwitchTime;
    std::map<TrafficClass, ClassPolicy> m_policies;
    std::map<TrafficClass, uint32_t> m_activePathIdx;
    std::map<uint32_t, std::pair<uint32_t, std::string>> m_look;
    std::map<uint32_t, double> m_recoveredAt;
    std::map<uint32_t, std::string> m_linkText;
    std::string m_lookSource;
    std::string m_alertText;
    double m_alertUntil = -1.0;
    std::ofstream m_congFile;
    std::ofstream m_bufFile;
    uint32_t m_bufCounter = 0;
    double m_lastCounter[8] = {-100,-100,-100,-100,-100,-100,-100,-100};
    bool m_spillAnnounced[8] = {false,false,false,false,false,false,false,false};
    uint64_t m_spillTotal = 0;
    double m_lastAnnounce[8] = {-10,-10,-10,-10,-10,-10,-10,-10};
    double m_classRateMbps[4] = {0,0,0,0};
    std::ofstream m_decisionsFile;
};

// Connect ProbeMonitorApp callbacks to DecisionEngine
void ProbeMonitorApp::OnProbeLost(uint32_t pathId) {
    m_consecutiveLosses[pathId]++;
    m_consecutiveHits[pathId] = 0;
    m_probesLostWindow[pathId]++;

    if (m_consecutiveLosses[pathId] >= m_cfg.failMissCount && !m_paths[pathId].isDown) {
        m_engine->OnPathFailureDiscovered(pathId);
    }
}

void ProbeMonitorApp::OnProbeSuccess(uint32_t pathId) {
    m_consecutiveHits[pathId]++;
    m_consecutiveLosses[pathId] = 0;

    if (m_consecutiveHits[pathId] >= m_cfg.recoverHitCount && m_paths[pathId].isDown) {
        m_engine->OnPathRecoveryDiscovered(pathId);
    }
}

// =============================================================================
// Emergency Sender Application (UDP Port 7000)
// =============================================================================
class EmergencySenderApp : public Application {
public:
    EmergencySenderApp() : m_running(false), m_seq(0), m_interval(MilliSeconds(100)), m_packetSize(100) {}

    void Setup(Ptr<Socket> socket, Address sinkAddress, Time interval, uint32_t packetSize) {
        m_socket = socket;
        m_sinkAddress = sinkAddress;
        m_interval = interval;
        m_packetSize = packetSize;
    }

private:
    virtual void StartApplication(void) override {
        m_running = true;
        m_socket->Connect(m_sinkAddress);
        SendPacket();
    }

    virtual void StopApplication(void) override {
        m_running = false;
        if (m_sendEvent.IsPending()) Simulator::Cancel(m_sendEvent);
    }

    void SendPacket(void) {
        if (!m_running) return;
        uint32_t seq = ++m_seq;
        uint64_t nowNs = Simulator::Now().GetNanoSeconds();

        uint32_t payload = (m_packetSize > 12) ? (m_packetSize - 12) : 1;
        Ptr<Packet> p = Create<Packet>(payload);

        // Write seq (4B) and nowNs (8B) into payload for exact latency tracking
        uint8_t buffer[12];
        buffer[0] = (seq >> 24) & 0xFF; buffer[1] = (seq >> 16) & 0xFF;
        buffer[2] = (seq >> 8) & 0xFF;  buffer[3] = seq & 0xFF;
        for (int i = 0; i < 8; ++i) {
            buffer[4 + i] = (nowNs >> (56 - 8 * i)) & 0xFF;
        }
        Ptr<Packet> packetWithHeader = Create<Packet>(buffer, 12);
        packetWithHeader->AddAtEnd(p);

        m_socket->Send(packetWithHeader);
        g_emergencyStats.sent++;

        m_sendEvent = Simulator::Schedule(m_interval, &EmergencySenderApp::SendPacket, this);
    }

    Ptr<Socket> m_socket;
    Address m_sinkAddress;
    bool m_running;
    uint32_t m_seq;
    Time m_interval;
    EventId m_sendEvent;
    uint32_t m_packetSize;
};

// =============================================================================
// Emergency Receiver Application (Deduplication, Latency, & Outage Tracking)
// =============================================================================
class EmergencyReceiverApp : public Application {
public:
    EmergencyReceiverApp() : m_port(7000), m_deadlineMs(150.0), m_prevArrivalNs(0), m_prevSeq(0) {}

    void Setup(uint16_t port, double deadlineMs) {
        m_port = port;
        m_deadlineMs = deadlineMs;
    }

    uint64_t m_intervalRxBytes = 0;
    uint32_t m_intervalRxPackets = 0;
    double m_intervalDelaySumMs = 0.0;

private:
    virtual void StartApplication(void) override {
        m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
        InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), m_port);
        m_socket->Bind(local);
        m_socket->SetRecvCallback(MakeCallback(&EmergencyReceiverApp::HandleRead, this));
    }

    virtual void StopApplication(void) override {
        if (m_socket) m_socket->Close();
    }

    void HandleRead(Ptr<Socket> socket) {
        Ptr<Packet> packet;
        Address from;
        while ((packet = socket->RecvFrom(from))) {
            uint64_t nowNs = Simulator::Now().GetNanoSeconds();
            m_intervalRxBytes += packet->GetSize();
            m_intervalRxPackets++;

            if (packet->GetSize() >= 12) {
                uint8_t buffer[12];
                packet->CopyData(buffer, 12);
                uint32_t seq = (buffer[0] << 24) | (buffer[1] << 16) | (buffer[2] << 8) | buffer[3];
                uint64_t txNs = 0;
                for (int i = 0; i < 8; ++i) {
                    txNs = (txNs << 8) | buffer[4 + i];
                }

                if (g_emergencyStats.seenSeq.find(seq) == g_emergencyStats.seenSeq.end()) {
                    g_emergencyStats.seenSeq[seq] = true;
                    g_emergencyStats.received++;

                    double delayMs = (nowNs > txNs) ? ((nowNs - txNs) / 1e6) : 0.0;
                    g_emergencyStats.sumDelayMs += delayMs;
                    m_intervalDelaySumMs += delayMs;
                    g_emergencyStats.maxDelayMs = std::max(g_emergencyStats.maxDelayMs, delayMs);

                    if (delayMs > m_deadlineMs) {
                        g_emergencyStats.deadlineMisses++;
                    }

                    // Track inter-packet arrival gaps during outages
                    if (m_prevArrivalNs > 0) {
                        uint32_t lostPkts = (seq > m_prevSeq + 1) ? (seq - m_prevSeq - 1) : 0;
                        g_failureLogger.RecordEmergencyOutageGap(m_prevArrivalNs / 1e9, nowNs / 1e9, lostPkts);
                    }
                    m_prevArrivalNs = nowNs;
                    m_prevSeq = seq;

                    g_failureLogger.OnClassPacketDelivered(CLASS_EMERGENCY, 1, Simulator::Now().GetSeconds());
                }
            }
        }
    }

    uint16_t m_port;
    double m_deadlineMs;
    Ptr<Socket> m_socket;
    uint64_t m_prevArrivalNs;
    uint32_t m_prevSeq;
};

// =============================================================================
// Multi-File TCP Transfer Manager (Sequential File Transfers over BulkSend)
// =============================================================================
struct FileTransferItem {
    uint32_t index;
    uint64_t targetBytes;
    uint64_t rxBytes;
    double startTimeSec;
    double completionTimeSec;
    double durationSec;
    uint32_t retransmissions;
    uint32_t pathId;
    bool completed;
    bool resumedAfterFailure;
};

class FileTransferManager {
public:
    FileTransferManager(Ptr<Node> senderNode, Address remoteAddr, const std::vector<uint64_t>& sizes,
                        double gapSec, uint32_t filePort, uint32_t fileTos,
                        const std::string& resultsDir, DecisionEngine* engine)
        : m_senderNode(senderNode), m_remoteAddr(remoteAddr), m_sizes(sizes),
          m_gapSec(gapSec), m_port(filePort), m_tos(fileTos),
          m_resultsDir(resultsDir), m_engine(engine),
          m_currentIndex(0), m_prevSinkTotal(0), m_retransmissions(0)
    {
        for (uint32_t i = 0; i < m_sizes.size(); ++i) {
            FileTransferItem item;
            item.index = i;
            item.targetBytes = m_sizes[i];
            item.rxBytes = 0;
            item.startTimeSec = 0.0;
            item.completionTimeSec = std::numeric_limits<double>::quiet_NaN();
            item.durationSec = std::numeric_limits<double>::quiet_NaN();
            item.retransmissions = 0;
            item.pathId = 2;
            item.completed = false;
            item.resumedAfterFailure = false;
            m_items.push_back(item);
        }
    }

    void Setup(Ptr<PacketSink> sink) {
        m_sink = sink;
    }

    void StartFirstFile(double startSec) {
        Simulator::Schedule(Seconds(startSec), &FileTransferManager::LaunchCurrentFile, this);
    }

    void LaunchCurrentFile() {
        if (m_currentIndex >= m_items.size()) return;

        double now = Simulator::Now().GetSeconds();
        m_items[m_currentIndex].startTimeSec = now;
        m_items[m_currentIndex].pathId = m_engine ? m_engine->GetActivePath(CLASS_FILE) : 2;
        m_retransmissions = 0;
        m_prevSinkTotal = m_sink->GetTotalRx();

        std::cout << "[FileTransferManager] Starting File " << (m_currentIndex + 1) << " of " << m_items.size()
                  << " (" << m_items[m_currentIndex].targetBytes << " Bytes) at t = " << now << " s on Path "
                  << m_items[m_currentIndex].pathId << "\n";

        BulkSendHelper fileHelper("ns3::TcpSocketFactory", m_remoteAddr);
        fileHelper.SetAttribute("MaxBytes", UintegerValue(m_items[m_currentIndex].targetBytes));
        fileHelper.SetAttribute("SendSize", UintegerValue(1400));
        fileHelper.SetAttribute("Tos", UintegerValue(m_tos));
        m_currentApp = fileHelper.Install(m_senderNode);
        m_currentApp.Start(Seconds(0.0));

        m_currentApp.Get(0)->TraceConnectWithoutContext("TcpRetransmission",
            MakeCallback(&FileTransferManager::OnRetransmission, this));

        Simulator::Schedule(MilliSeconds(20), &FileTransferManager::CheckCurrentFileProgress, this);
    }

    void OnRetransmission(const Ptr<const Packet> packet,
                          const TcpHeader& header,
                          const Address& localAddr,
                          const Address& peerAddr,
                          const Ptr<const TcpSocketBase> socket) {
        m_retransmissions++;
    }

    void CheckCurrentFileProgress() {
        if (m_currentIndex >= m_items.size()) return;

        uint64_t curSinkTotal = m_sink->GetTotalRx();
        uint64_t bytesReceivedThisFile = (curSinkTotal >= m_prevSinkTotal) ? (curSinkTotal - m_prevSinkTotal) : 0;
        m_items[m_currentIndex].rxBytes = bytesReceivedThisFile;
        m_items[m_currentIndex].retransmissions = m_retransmissions;

        uint32_t curPath = m_engine ? m_engine->GetActivePath(CLASS_FILE) : m_items[m_currentIndex].pathId;
        if (curPath != m_items[m_currentIndex].pathId) {
            m_items[m_currentIndex].resumedAfterFailure = true;
            m_items[m_currentIndex].pathId = curPath;
        }

        if (bytesReceivedThisFile >= m_items[m_currentIndex].targetBytes) {
            double now = Simulator::Now().GetSeconds();
            m_items[m_currentIndex].completionTimeSec = now;
            m_items[m_currentIndex].durationSec = now - m_items[m_currentIndex].startTimeSec;
            m_items[m_currentIndex].completed = true;

            std::cout << "[FileTransferManager] File " << (m_currentIndex + 1) << " Completed at t = "
                      << now << " s (Duration: " << m_items[m_currentIndex].durationSec
                      << " s, Retransmissions: " << m_retransmissions << ")\n";

            if (m_currentIndex == m_items.size() - 1) {
                g_systemMetrics.fileCompletionTimeSec = now - m_items[0].startTimeSec;
            }

            m_currentApp.Stop(Seconds(0.0));
            m_currentIndex++;

            if (m_currentIndex < m_items.size()) {
                Simulator::Schedule(Seconds(m_gapSec), &FileTransferManager::LaunchCurrentFile, this);
            }
            return;
        }

        Simulator::Schedule(MilliSeconds(20), &FileTransferManager::CheckCurrentFileProgress, this);
    }

    void WriteResults() {
        std::string path = m_resultsDir + "/file_transfers.csv";
        std::ofstream f(path);
        f << "FileIndex,TargetBytes,RxBytes,StartTimeSec,CompletionTimeSec,DurationSec,Retransmissions,PathId,Completed,Resumed\n";
        for (const auto& it : m_items) {
            f << it.index << "," << it.targetBytes << "," << it.rxBytes << ","
              << std::fixed << std::setprecision(3) << it.startTimeSec << ","
              << (std::isnan(it.completionTimeSec) ? -1.0 : it.completionTimeSec) << ","
              << (std::isnan(it.durationSec) ? -1.0 : it.durationSec) << ","
              << it.retransmissions << "," << it.pathId << ","
              << (it.completed ? 1 : 0) << "," << (it.resumedAfterFailure ? 1 : 0) << "\n";
        }
        f.close();
    }

private:
    Ptr<Node> m_senderNode;
    Address m_remoteAddr;
    std::vector<uint64_t> m_sizes;
    double m_gapSec;
    uint32_t m_port;
    uint32_t m_tos;
    std::string m_resultsDir;
    DecisionEngine* m_engine;

    Ptr<PacketSink> m_sink;
    ApplicationContainer m_currentApp;
    uint32_t m_currentIndex;
    uint64_t m_prevSinkTotal;
    uint32_t m_retransmissions;
    std::vector<FileTransferItem> m_items;
};

// =============================================================================
// Hardware Failure Injection Routine
// =============================================================================
static double g_restoreLoss[8] = {0,0,0,0,0,0,0,0};   // loss model re-applied after a router is restored
static void SetRouterHardwareDown(uint32_t pathId, bool isDown,
                                  NodeContainer &routerNodes,
                                  std::vector<NetDeviceContainer> &devSR,
                                  std::vector<NetDeviceContainer> &devRD,
                                  AnimationInterface *anim) {
    Ptr<Node> rNode = routerNodes.Get(pathId);
    Ptr<Ipv4> rIpv4 = rNode->GetObject<Ipv4>();

    if (isDown) {
        Ptr<RateErrorModel> em1 = CreateObject<RateErrorModel>();
        em1->SetAttribute("ErrorRate", DoubleValue(1.0));
        devSR[pathId].Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(em1));

        Ptr<RateErrorModel> em2 = CreateObject<RateErrorModel>();
        em2->SetAttribute("ErrorRate", DoubleValue(1.0));
        devRD[pathId].Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(em2));

        for (uint32_t ifIdx = 0; ifIdx < rIpv4->GetNInterfaces(); ++ifIdx) {
            rIpv4->SetDown(ifIdx);
        }
    } else {
        Ptr<RateErrorModel> emClean = CreateObject<RateErrorModel>();
        emClean->SetAttribute("ErrorRate", DoubleValue(0.0));
        Ptr<RateErrorModel> emLoss = CreateObject<RateErrorModel>();
        emLoss->SetAttribute("ErrorRate", DoubleValue(g_restoreLoss[pathId]));
        emLoss->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
        devSR[pathId].Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(emLoss));
        devRD[pathId].Get(0)->SetAttribute("ReceiveErrorModel", PointerValue(emClean));

        for (uint32_t ifIdx = 0; ifIdx < rIpv4->GetNInterfaces(); ++ifIdx) {
            rIpv4->SetUp(ifIdx);
        }
    }
}

static void OnPathTx(InternalPath *p, Ptr<const Packet> pkt) { p->intervalTxBytes += pkt->GetSize(); p->utilTxBytes += pkt->GetSize(); }
static void OnPathDrop(InternalPath *p, Ptr<const Packet> pkt) { p->intervalDrops++; }
static void OnQdiscDrop(InternalPath *p, Ptr<const QueueDiscItem> item) { p->intervalDrops++; }
static void OnQdiscDropBefore(InternalPath *p, Ptr<const QueueDiscItem> item, const char *reason) { p->intervalDrops++; p->offeredBytes += item->GetPacket()->GetSize(); }
static void OnQdiscEnqueue(InternalPath *p, Ptr<const QueueDiscItem> item) { p->offeredBytes += item->GetPacket()->GetSize(); }

// Video one-way delay tracking (packet UID is preserved end to end)
static std::map<uint64_t, double> g_videoTx;
static double g_videoDelaySum = 0.0;
static uint32_t g_videoDelayCnt = 0;
static void OnVideoTx(Ptr<const Packet> p) { g_videoTx[p->GetUid()] = Simulator::Now().GetSeconds(); }
static void OnVideoRx(Ptr<const Packet> p, const Address &) {
    auto it = g_videoTx.find(p->GetUid());
    if (it == g_videoTx.end()) return;
    g_videoDelaySum += (Simulator::Now().GetSeconds() - it->second) * 1000.0;
    g_videoDelayCnt++;
    g_videoTx.erase(it);
}

// Parse comma-separated file sizes
static std::vector<uint64_t> ParseFileSizes(const std::string &str, uint32_t count) {
    std::vector<uint64_t> sizes;
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, ',')) {
        try {
            if (!item.empty()) sizes.push_back(std::stoull(item));
        } catch (...) {}
    }
    while (sizes.size() < count) {
        sizes.push_back(sizes.empty() ? 5000000 : sizes.back());
    }
    return sizes;
}

// =============================================================================
// Application Lifecycle Audit Structure
// =============================================================================
struct AppAuditRecord {
    std::string nodeName;
    std::string appName;
    double startSec;
    double stopSec;
    bool isActive;
};

// =============================================================================
// Main Simulation Routine
// =============================================================================
int main(int argc, char *argv[]) {
    AdaptiveNetConfig config;
    CommandLine cmd(__FILE__);
    config.BindCommandLine(cmd);
    cmd.Parse(argc, argv);
    config.ApplyProfiles();

    // Enforce Seed & Run
    RngSeedManager::SetSeed(config.seed);
    RngSeedManager::SetRun(config.runIndex);

    // Print Configuration
    config.Print(std::cout);

    // Write JSON Config
    std::string jsonPath = config.resultsDir + "/config_" + config.mode + "_" + config.scenario + ".json";
    config.WriteJson(jsonPath);

    // Initialize Failure Logger
    std::string failPath = config.resultsDir + "/failure_detection.csv";
    g_failureLogger.Init(failPath, config.scenario, config.mode);

    // 1. Create Nodes in Exact Order: Node 0=Source, 1=Dest, 2..9=R0..R7, 10..12=Apps
    Ptr<Node> sourceNode = CreateObject<Node>(); // Node 0
    Ptr<Node> destNode   = CreateObject<Node>(); // Node 1

    NodeContainer routerNodes;
    routerNodes.Create(8);                       // Nodes 2..9

    NodeContainer appNodes;
    appNodes.Create(3);                          // Nodes 10..12
    Ptr<Node> appEmergNode = appNodes.Get(0);    // Node 10
    Ptr<Node> appVideoNode = appNodes.Get(1);    // Node 11
    Ptr<Node> appFileNode  = appNodes.Get(2);    // Node 12

    NodeContainer allNodes;
    allNodes.Add(sourceNode);
    allNodes.Add(destNode);
    allNodes.Add(routerNodes);
    allNodes.Add(appNodes);

    // 2. Install Mobility & Assign Layout Positions
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> posAlloc = CreateObject<ListPositionAllocator>();
    posAlloc->Add(config.posSource);
    posAlloc->Add(config.posDestination);
    for (int k = 0; k < 8; ++k) posAlloc->Add(config.posRouters[k]);
    posAlloc->Add(config.posAppEmerg);
    posAlloc->Add(config.posAppVideo);
    posAlloc->Add(config.posAppFile);

    mobility.SetPositionAllocator(posAlloc);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(allNodes);

    // Layout Separation Self-Checks
    for (uint32_t i = 0; i < allNodes.GetN(); ++i) {
        for (uint32_t j = i + 1; j < allNodes.GetN(); ++j) {
            Vector p1 = allNodes.Get(i)->GetObject<MobilityModel>()->GetPosition();
            Vector p2 = allNodes.Get(j)->GetObject<MobilityModel>()->GetPosition();
            double dist = ns3::CalculateDistance(p1, p2);
            NS_ASSERT_MSG(dist >= (config.minNodeGap - 1e-4),
                          "Node separation violation between Node " << i << " and Node " << j << ": " << dist);
        }
    }

    // 3. Install Internet Stacks (Fix P1: Ipv4ListRouting on sourceNode)
    InternetStackHelper internet;
    Ipv4ListRoutingHelper sourceListRouting;
    Ptr<AdaptiveRoutingProtocol> sourceRouting = CreateObject<AdaptiveRoutingProtocol>();
    AdaptiveRoutingHelper sourceRoutingHelper(sourceRouting);
    sourceListRouting.Add(sourceRoutingHelper, 10); // Priority 10: Class-steering
    Ipv4StaticRoutingHelper staticRoutingHelper;
    sourceListRouting.Add(staticRoutingHelper, 5);  // Priority 5: Local access fallback

    internet.SetRoutingHelper(sourceListRouting);
    internet.Install(sourceNode);

    InternetStackHelper internetStandard;
    internetStandard.Install(destNode);
    internetStandard.Install(routerNodes);
    internetStandard.Install(appEmergNode);
    internetStandard.Install(appVideoNode);
    internetStandard.Install(appFileNode);

    // 4. Configure Local Access Links to Source Router
    PointToPointHelper p2pAccess;
    p2pAccess.SetDeviceAttribute("DataRate", StringValue(config.accessBusRate));
    p2pAccess.SetChannelAttribute("Delay", StringValue(config.accessBusDelay));

    NetDeviceContainer devEmerg = p2pAccess.Install(appEmergNode, sourceNode);
    NetDeviceContainer devVideo = p2pAccess.Install(appVideoNode, sourceNode);
    NetDeviceContainer devFile  = p2pAccess.Install(appFileNode,  sourceNode);

    Ipv4AddressHelper ipHelper;
    ipHelper.SetBase("10.0.1.0", "255.255.255.0");
    Ipv4InterfaceContainer ifEmerg = ipHelper.Assign(devEmerg);
    sourceRouting->AddAccessRoute(Ipv4Address("10.0.1.0"), Ipv4Mask("255.255.255.0"),
                                  devEmerg.Get(1), ifEmerg.GetAddress(0));

    ipHelper.SetBase("10.0.2.0", "255.255.255.0");
    Ipv4InterfaceContainer ifVideo = ipHelper.Assign(devVideo);
    sourceRouting->AddAccessRoute(Ipv4Address("10.0.2.0"), Ipv4Mask("255.255.255.0"),
                                  devVideo.Get(1), ifVideo.GetAddress(0));

    ipHelper.SetBase("10.0.3.0", "255.255.255.0");
    Ipv4InterfaceContainer ifFile = ipHelper.Assign(devFile);
    sourceRouting->AddAccessRoute(Ipv4Address("10.0.3.0"), Ipv4Mask("255.255.255.0"),
                                  devFile.Get(1), ifFile.GetAddress(0));

    // Default static routing on App nodes towards Source Router
    Ptr<Ipv4StaticRouting> staticEmerg = staticRoutingHelper.GetStaticRouting(appEmergNode->GetObject<Ipv4>());
    staticEmerg->SetDefaultRoute(ifEmerg.GetAddress(1), 1);

    Ptr<Ipv4StaticRouting> staticVideo = staticRoutingHelper.GetStaticRouting(appVideoNode->GetObject<Ipv4>());
    staticVideo->SetDefaultRoute(ifVideo.GetAddress(1), 1);

    Ptr<Ipv4StaticRouting> staticFile = staticRoutingHelper.GetStaticRouting(appFileNode->GetObject<Ipv4>());
    staticFile->SetDefaultRoute(ifFile.GetAddress(1), 1);

    // 5. Configure 8 Heterogeneous Parallel Paths (Source - Rk - Destination)
    std::vector<NetDeviceContainer> devSource_R(8);
    std::vector<NetDeviceContainer> devR_Dest(8);
    std::vector<Ipv4InterfaceContainer> ifSource_R(8);
    std::vector<Ipv4InterfaceContainer> ifR_Dest(8);
    std::vector<InternalPath> paths(8);

    for (int k = 0; k < 8; ++k) {
        PointToPointHelper p2pPath;
        p2pPath.SetDeviceAttribute("DataRate", StringValue(config.pathRates[k]));
        p2pPath.SetChannelAttribute("Delay", StringValue(config.pathDelays[k]));
        p2pPath.SetQueue("ns3::DropTailQueue", "MaxSize", StringValue(config.pathQueues[k]));

        devSource_R[k] = p2pPath.Install(sourceNode, routerNodes.Get(k));
        devR_Dest[k]   = p2pPath.Install(routerNodes.Get(k), destNode);

        std::ostringstream subnetSR, subnetRD;
        subnetSR << "10.1." << k << ".0";
        subnetRD << "10.2." << k << ".0";

        {   // Router buffer = FIFO queue disc sized by pathQueues[k]; keep the device queue tiny so
            // that all queueing happens (and is visible) in this one buffer.
            TrafficControlHelper tch;
            tch.SetRootQueueDisc("ns3::FifoQueueDisc", "MaxSize", StringValue(config.pathQueues[k]));
            tch.Install(devSource_R[k].Get(0));
            Ptr<PointToPointNetDevice> pd = DynamicCast<PointToPointNetDevice>(devSource_R[k].Get(0));
            pd->GetQueue()->SetMaxSize(QueueSize("2p"));
        }
        if (config.realLoss) {
            Ptr<RateErrorModel> lossModel = CreateObject<RateErrorModel>();
            lossModel->SetAttribute("ErrorRate", DoubleValue(config.pathLossRates[k]));
            lossModel->SetAttribute("ErrorUnit", StringValue("ERROR_UNIT_PACKET"));
            devSource_R[k].Get(1)->SetAttribute("ReceiveErrorModel", PointerValue(lossModel));
            g_restoreLoss[k] = config.pathLossRates[k];
        }
        ipHelper.SetBase(subnetSR.str().c_str(), "255.255.255.0");
        ifSource_R[k] = ipHelper.Assign(devSource_R[k]);

        ipHelper.SetBase(subnetRD.str().c_str(), "255.255.255.0");
        ifR_Dest[k] = ipHelper.Assign(devR_Dest[k]);

        paths[k].id = k;
        paths[k].name = config.pathRates[k];
        paths[k].baseDelayMs = std::stod(config.pathDelays[k]);
        paths[k].baseBandwidthMbps = DataRate(config.pathRates[k]).GetBitRate() / 1e6;
        paths[k].baseLossRate = config.pathLossRates[k];

        paths[k].smoothedDelayMs = paths[k].baseDelayMs;
        paths[k].smoothedLossRate = paths[k].baseLossRate;
        paths[k].smoothedBandwidthMbps = paths[k].baseBandwidthMbps;
        paths[k].isDown = false;

        paths[k].nextHopIp = ifSource_R[k].GetAddress(1);
        paths[k].srcIp = ifSource_R[k].GetAddress(0);
        paths[k].destPathIp = ifR_Dest[k].GetAddress(1);
        paths[k].outputDevice = devSource_R[k].Get(0);

        paths[k].returnHopIp = ifR_Dest[k].GetAddress(0);
        paths[k].returnInterface = ifR_Dest[k].Get(1).first->GetInterfaceForDevice(devR_Dest[k].Get(1));

        // Router forwarding routing: route 10.2.0.0/16 network to Destination
        Ptr<Ipv4StaticRouting> rStatic = staticRoutingHelper.GetStaticRouting(routerNodes.Get(k)->GetObject<Ipv4>());
        rStatic->AddNetworkRouteTo(Ipv4Address("10.2.0.0"), Ipv4Mask("255.255.0.0"),
                                  ifR_Dest[k].GetAddress(1), 2, 1);
        rStatic->AddNetworkRouteTo(Ipv4Address("10.0.0.0"), Ipv4Mask("255.255.0.0"),
                                  ifSource_R[k].GetAddress(0), 1, 1);
    }

    // 6. Initialize NetAnim
    AnimationInterface *anim = nullptr;
    if (config.enableAnim) {
        anim = new AnimationInterface(config.animFile);
        anim->SetMaxPktsPerTraceFile(500000);
        anim->SetStartTime(Seconds(config.animStartTime));
        anim->SetStopTime(Seconds(config.animStopTime));
        anim->EnablePacketMetadata(config.animPacketMeta);

        anim->UpdateNodeDescription(sourceNode, "Source Router");
        anim->UpdateNodeColor(sourceNode, 0, 0, 255);
        anim->UpdateNodeDescription(destNode, "Destination Sink");
        anim->UpdateNodeColor(destNode, 0, 150, 0);

        for (int k = 0; k < 8; ++k) {
            anim->UpdateNodeDescription(routerNodes.Get(k), "R" + std::to_string(k));
            anim->UpdateNodeColor(routerNodes.Get(k), 180, 180, 180);
        }

        anim->UpdateNodeDescription(appEmergNode, "App-Emergency [7000]");
        anim->UpdateNodeColor(appEmergNode, 255, 0, 0);
        anim->UpdateNodeDescription(appVideoNode, "App-Video [5000]");
        anim->UpdateNodeColor(appVideoNode, 0, 100, 255);
        anim->UpdateNodeDescription(appFileNode, "App-File [6000]");
        anim->UpdateNodeColor(appFileNode, 0, 200, 0);
    }

    // 7. Application Lifecycle Audit List
    std::vector<AppAuditRecord> appAudit;

    // A. Emergency Application (UDP 7000, ToS = config.emergTos)
    Ptr<Socket> emergSocket = Socket::CreateSocket(appEmergNode, UdpSocketFactory::GetTypeId());
    emergSocket->SetIpTos(config.emergTos); // Fix P5: Explicit ToS

    Ptr<EmergencySenderApp> emergSender = CreateObject<EmergencySenderApp>();
    emergSender->Setup(emergSocket, InetSocketAddress(paths[0].destPathIp, config.emergPort),
                       MilliSeconds(config.emergIntervalMs), config.emergPacketSize);
    appEmergNode->AddApplication(emergSender);
    emergSender->SetStartTime(Seconds(0.5));
    emergSender->SetStopTime(Seconds(config.simTime));
    appAudit.push_back({"App-Emergency (10)", "EmergencySenderApp (7000)", 0.5, config.simTime, true});

    Ptr<EmergencyReceiverApp> emergReceiver = CreateObject<EmergencyReceiverApp>();
    emergReceiver->Setup(config.emergPort, config.emergDeadlineMs);
    destNode->AddApplication(emergReceiver);
    emergReceiver->SetStartTime(Seconds(0.0));
    emergReceiver->SetStopTime(Seconds(config.simTime));
    appAudit.push_back({"Destination (1)", "EmergencyReceiverApp (7000)", 0.0, config.simTime, true});

    // B. Video Application (OnOff UDP 5000, ToS = config.videoTos)
    OnOffHelper videoHelper("ns3::UdpSocketFactory", InetSocketAddress(paths[0].destPathIp, config.videoPort));
    videoHelper.SetAttribute("DataRate", StringValue(config.videoDataRate));
    videoHelper.SetAttribute("PacketSize", UintegerValue(config.videoPacketSize));
    videoHelper.SetAttribute("Tos", UintegerValue(config.videoTos)); // Fix P5: Explicit ToS
    videoHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    videoHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    ApplicationContainer videoApp = videoHelper.Install(appVideoNode);
    videoApp.Start(Seconds(0.5));
    videoApp.Stop(Seconds(config.simTime));
    videoApp.Get(0)->TraceConnectWithoutContext("Tx", MakeCallback(&OnVideoTx));
    appAudit.push_back({"App-Video (11)", "OnOff Video (5000)", 0.5, config.simTime, true});

    PacketSinkHelper videoSink("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), config.videoPort));
    ApplicationContainer videoSinkApp = videoSink.Install(destNode);
    videoSinkApp.Start(Seconds(0.0));
    videoSinkApp.Stop(Seconds(config.simTime));
    videoSinkApp.Get(0)->TraceConnectWithoutContext("Rx", MakeCallback(&OnVideoRx));
    appAudit.push_back({"Destination (1)", "PacketSink Video (5000)", 0.0, config.simTime, true});

    // C. File Transfer Sink on Destination (TCP 6000)
    PacketSinkHelper fileSink("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), config.filePort));
    ApplicationContainer fileSinkApp = fileSink.Install(destNode);
    fileSinkApp.Start(Seconds(0.0));
    fileSinkApp.Stop(Seconds(config.simTime));
    appAudit.push_back({"Destination (1)", "PacketSink File (6000)", 0.0, config.simTime, true});
    Ptr<PacketSink> fSinkPtr = DynamicCast<PacketSink>(fileSinkApp.Get(0));

    // 8. Initialize Centralized Decision Engine & Probe Monitor
    Ptr<Ipv4StaticRouting> destStatic = staticRoutingHelper.GetStaticRouting(destNode->GetObject<Ipv4>());
    for (int k = 0; k < 8; ++k) {
        sourceRouting->AddPathRoute(paths[k].destPathIp, paths[k].nextHopIp, paths[k].outputDevice, paths[k].srcIp);
        destStatic->AddNetworkRouteTo(Ipv4Address(("10.1." + std::to_string(k) + ".0").c_str()),
                                      Ipv4Mask("255.255.255.0"),
                                      paths[k].returnHopIp, paths[k].returnInterface, 1);
    }
    for (int k = 0; k < 8; ++k) {
        paths[k].outputDevice->TraceConnectWithoutContext("PhyTxEnd", MakeBoundCallback(&OnPathTx, &paths[k]));
        paths[k].outputDevice->TraceConnectWithoutContext("MacTxDrop", MakeBoundCallback(&OnPathDrop, &paths[k]));
        // ns-3 installs a default queue disc on every device: that is where congestion drops happen
        Ptr<TrafficControlLayer> tcl = sourceNode->GetObject<TrafficControlLayer>();
        Ptr<QueueDisc> qd = tcl ? tcl->GetRootQueueDiscOnDevice(paths[k].outputDevice) : nullptr;
        if (qd) {
            paths[k].qdisc = qd;
            qd->TraceConnectWithoutContext("Enqueue", MakeBoundCallback(&OnQdiscEnqueue, &paths[k]));
            qd->TraceConnectWithoutContext("Drop", MakeBoundCallback(&OnQdiscDrop, &paths[k]));
            qd->TraceConnectWithoutContext("DropBeforeEnqueue", MakeBoundCallback(&OnQdiscDropBefore, &paths[k]));
        }
    }
    DecisionEngine engine(config, sourceNode, destNode, sourceRouting, destStatic, paths, anim);
    Simulator::Schedule(Seconds(0.5), &DecisionEngine::StartMonitoring, &engine);

    // Install Probe Echo Receiver on Destination
    Ptr<ProbeEchoReceiverApp> probeEcho = CreateObject<ProbeEchoReceiverApp>();
    probeEcho->Setup(config.probePort);
    destNode->AddApplication(probeEcho);
    probeEcho->SetStartTime(Seconds(0.0));
    probeEcho->SetStopTime(Seconds(config.simTime));
    appAudit.push_back({"Destination (1)", "ProbeEchoReceiverApp (8888)", 0.0, config.simTime, true});

    // Install Probe Monitor on Source
    Ptr<ProbeMonitorApp> probeMonitor = CreateObject<ProbeMonitorApp>(config, paths, &engine);
    probeMonitor->SetupSockets(sourceNode);
    sourceNode->AddApplication(probeMonitor);
    probeMonitor->SetStartTime(Seconds(0.1));
    probeMonitor->SetStopTime(Seconds(config.simTime));
    appAudit.push_back({"Source (0)", "ProbeMonitorApp (8888)", 0.1, config.simTime, true});

    // Initialize Multi-File Sequential TCP Transfer Queue
    std::vector<uint64_t> fileSizes = ParseFileSizes(config.fileSizesBytes, config.fileCount);
    FileTransferManager fileManager(appFileNode, InetSocketAddress(paths[0].destPathIp, config.filePort),
                                    fileSizes, config.fileGapSec, config.filePort, config.fileTos,
                                    config.resultsDir, &engine);
    fileManager.Setup(fSinkPtr);
    fileManager.StartFirstFile(1.0);
    appAudit.push_back({"App-File (12)", "BulkSend Multi-File Queue", 1.0, config.simTime - 0.5, true});

    // Audit Table Printout & Assertion (Fix P2)
    std::cout << "\n==================================================================\n";
    std::cout << "               Application Lifecycle Audit Table                  \n";
    std::cout << "==================================================================\n";
    std::cout << std::left << std::setw(22) << "Node"
              << std::setw(30) << "Application"
              << std::setw(14) << "Start (s)"
              << std::setw(14) << "Stop (s)" << "\n";
    std::cout << "------------------------------------------------------------------\n";
    for (const auto& entry : appAudit) {
        std::cout << std::left << std::setw(22) << entry.nodeName
                  << std::setw(30) << entry.appName;
        if (!entry.isActive) {
            std::cout << std::setw(14) << "SILENT" << std::setw(14) << "SILENT\n";
        } else {
            std::cout << std::setw(14) << std::fixed << std::setprecision(2) << entry.startSec
                      << std::setw(14) << std::fixed << std::setprecision(2) << entry.stopSec << "\n";
        }
        NS_ASSERT_MSG(entry.startSec <= entry.stopSec || !entry.isActive,
                      "Application start time must precede stop time!");
    }
    std::cout << "==================================================================\n\n";

    // 9. Scenario Injections (Discovered by Probes Only)
    if (config.scenario == "steady") {
        // Steady state: No injected failures or disruptions; clean baseline run
        std::cout << "[Scenario: steady] Baseline operation with zero injected disruptions.\n";
    }
    else if (config.scenario == "nodefail_single") {
        Simulator::Schedule(Seconds(config.nodeFailStartTime), [&engine, &routerNodes, &devSource_R, &devR_Dest, anim, &config]() {
            uint32_t failPath = engine.GetActivePath(CLASS_EMERGENCY);
            std::cout << "[Event @ " << config.nodeFailStartTime << "s] S1: Router R"
                      << failPath << " Hardware Fails!\n";
            g_failureLogger.OnRealFailure(failPath, config.nodeFailStartTime);
            SetRouterHardwareDown(failPath, true, routerNodes, devSource_R, devR_Dest, anim);
        });

        Simulator::Schedule(Seconds(config.nodeFailEndTime), [&engine, &routerNodes, &devSource_R, &devR_Dest, anim, &config]() {
            std::cout << "[Event @ " << config.nodeFailEndTime << "s] S1: Router Restored\n";
            for (uint32_t k = 0; k < 8; ++k) {
                g_failureLogger.OnRealRestore(k, config.nodeFailEndTime);
                SetRouterHardwareDown(k, false, routerNodes, devSource_R, devR_Dest, anim);
            }
        });
    }
    else if (config.scenario == "nodefail_all_used") {
        Simulator::Schedule(Seconds(config.nodeFailStartTime), [&engine, &routerNodes, &devSource_R, &devR_Dest, anim, &config]() {
            std::vector<uint32_t> failedPaths;
            failedPaths.push_back(engine.GetActivePath(CLASS_EMERGENCY));
            failedPaths.push_back(engine.GetActivePath(CLASS_VIDEO));
            failedPaths.push_back(engine.GetActivePath(CLASS_FILE));

            std::cout << "[Event @ " << config.nodeFailStartTime
                      << "s] S2: Simultaneous Failure on All Used Routers!\n";
            for (uint32_t p : failedPaths) {
                g_failureLogger.OnRealFailure(p, config.nodeFailStartTime);
                SetRouterHardwareDown(p, true, routerNodes, devSource_R, devR_Dest, anim);
            }
        });

        Simulator::Schedule(Seconds(config.nodeFailEndTime), [&routerNodes, &devSource_R, &devR_Dest, anim, &config]() {
            std::cout << "[Event @ " << config.nodeFailEndTime << "s] S2: All Routers Restored\n";
            for (uint32_t k = 0; k < 8; ++k) {
                g_failureLogger.OnRealRestore(k, config.nodeFailEndTime);
                SetRouterHardwareDown(k, false, routerNodes, devSource_R, devR_Dest, anim);
            }
        });
    }
    else if (config.scenario == "nodefail_cascade") {
        for (uint32_t step = 0; step < (8 - config.minSurvivors); ++step) {
            double tFail = 6.0 + step * config.cascadePeriodSec;
            if (tFail >= 25.0) break; // recovery happens at 25 s
            Simulator::Schedule(Seconds(tFail), [&engine, &routerNodes, &devSource_R, &devR_Dest, anim, tFail]() {
                uint32_t failPath = engine.GetActivePath(CLASS_EMERGENCY);
                std::cout << "[Event @ " << tFail << "s] S3: Cascade Failure on Active Router R"
                          << failPath << "\n";
                g_failureLogger.OnRealFailure(failPath, tFail);
                SetRouterHardwareDown(failPath, true, routerNodes, devSource_R, devR_Dest, anim);
            });
        }
        Simulator::Schedule(Seconds(25.0), [&routerNodes, &devSource_R, &devR_Dest, anim]() {
            std::cout << "[Event @ 25.0s] S3: Cascade Recovery - All Routers Restored\n";
            for (uint32_t k = 0; k < 8; ++k) {
                g_failureLogger.OnRealRestore(k, 25.0);
                SetRouterHardwareDown(k, false, routerNodes, devSource_R, devR_Dest, anim);
            }
        });
    }
    else if (config.scenario == "nodefail_flap") {
        for (double t = 8.0; t < 24.0; t += (config.flapPeriodSec * 2)) {
            Simulator::Schedule(Seconds(t), [&routerNodes, &devSource_R, &devR_Dest, anim, t]() {
                std::cout << "[Event @ " << t << "s] S4: Router R1 Flap DOWN\n";
                g_failureLogger.OnRealFailure(1, t);
                SetRouterHardwareDown(1, true, routerNodes, devSource_R, devR_Dest, anim);
            });
            Simulator::Schedule(Seconds(t + config.flapPeriodSec), [&routerNodes, &devSource_R, &devR_Dest, anim, t, &config]() {
                double tRec = t + config.flapPeriodSec;
                std::cout << "[Event @ " << tRec << "s] S4: Router R1 Flap UP\n";
                g_failureLogger.OnRealRestore(1, tRec);
                SetRouterHardwareDown(1, false, routerNodes, devSource_R, devR_Dest, anim);
            });
        }
    }
    else if (config.scenario == "congestion") {
        // Extra video-class load saturates the Video router (R1, 50 Mbps) between burstStart and burstEnd.
        OnOffHelper burst("ns3::UdpSocketFactory", InetSocketAddress(paths[0].destPathIp, config.videoPort));
        burst.SetAttribute("DataRate", StringValue(config.burstRate));
        burst.SetAttribute("PacketSize", UintegerValue(1400));
        burst.SetAttribute("Tos", UintegerValue(config.videoTos));
        burst.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
        burst.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
        ApplicationContainer b = burst.Install(appVideoNode);
        b.Start(Seconds(config.burstStart));
        b.Stop(Seconds(config.burstEnd));
        std::cout << "[Scenario: congestion] Extra " << config.burstRate << " video-class load "
                  << config.burstStart << "s-" << config.burstEnd << "s\n";
    }
    else if (config.scenario == "nodefail_last") {
        Simulator::Schedule(Seconds(8.0), [&routerNodes, &devSource_R, &devR_Dest, anim]() {
            std::cout << "[Event @ 8.0s] S5: All Routers R0..R6 Fail! Only R7 Survives\n";
            for (uint32_t k = 0; k < 7; ++k) {
                g_failureLogger.OnRealFailure(k, 8.0);
                SetRouterHardwareDown(k, true, routerNodes, devSource_R, devR_Dest, anim);
            }
        });

        for (uint32_t k = 0; k < 7; ++k) {
            double tRec = 18.0 + k * 1.5;
            Simulator::Schedule(Seconds(tRec), [k, &routerNodes, &devSource_R, &devR_Dest, anim]() {
                std::cout << "[Event @ " << Simulator::Now().GetSeconds()
                          << "s] S5: Router R" << k << " Restored\n";
                g_failureLogger.OnRealRestore(k, Simulator::Now().GetSeconds());
                SetRouterHardwareDown(k, false, routerNodes, devSource_R, devR_Dest, anim);
            });
        }
    }

    // Animation helper: two short bursts of extra video-class load so the buffers visibly fill
    // (and spill to a standby router) in every animated scenario. Only in --enableAnim runs.
    if (config.enableAnim && config.animLoad && config.scenario != "congestion") {
        double starts[2] = {4.0, 14.0};
        for (int b = 0; b < 2; ++b) {
            OnOffHelper burst("ns3::UdpSocketFactory", InetSocketAddress(paths[0].destPathIp, config.videoPort));
            burst.SetAttribute("DataRate", StringValue(config.burstRate));
            burst.SetAttribute("PacketSize", UintegerValue(1400));
            burst.SetAttribute("Tos", UintegerValue(config.videoTos));
            burst.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
            burst.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
            ApplicationContainer bc = burst.Install(appVideoNode);
            bc.Start(Seconds(starts[b]));
            bc.Stop(Seconds(starts[b] + 2.0));
        }
        std::cout << "[Animation] extra " << config.burstRate << " video-class load at 4-6 s and 14-16 s (--animLoad=false to disable)\n";
    }

    // 10. Real Time-Series Metric Logger (Per-Interval Reset Counters)
    std::string tsPath = config.resultsDir + "/time_series.csv";
    std::ofstream timeSeriesFile(tsPath);
    timeSeriesFile << "Time,EmergencyTputKbps,EmergencyDelayMs,VideoTputKbps,VideoDelayMs,"
                   << "FileTputKbps,ActiveEmergPath,ActiveVideoPath,ActiveFilePath\n";

    Ptr<PacketSink> vSink = DynamicCast<PacketSink>(videoSinkApp.Get(0));
    static uint64_t prevVideoBytes = 0;
    static uint64_t prevFileBytes = 0;

    for (double t = 1.0; t < config.simTime; t += 1.0) {
        Simulator::Schedule(Seconds(t), [&timeSeriesFile, &paths, &engine, emergReceiver, vSink, fSinkPtr]() {
            double now = Simulator::Now().GetSeconds();

            double emergTputKbps = (emergReceiver->m_intervalRxBytes * 8.0) / 1000.0;
            double emergDelay = (emergReceiver->m_intervalRxPackets > 0)
                                ? (emergReceiver->m_intervalDelaySumMs / emergReceiver->m_intervalRxPackets)
                                : std::numeric_limits<double>::quiet_NaN();
            emergReceiver->m_intervalRxBytes = 0;
            emergReceiver->m_intervalRxPackets = 0;
            emergReceiver->m_intervalDelaySumMs = 0.0;

            uint64_t curVideoBytes = vSink->GetTotalRx();
            double videoTputKbps = ((curVideoBytes - prevVideoBytes) * 8.0) / 1000.0;
            prevVideoBytes = curVideoBytes;
            double vDelay = g_videoDelayCnt ? g_videoDelaySum / g_videoDelayCnt : std::numeric_limits<double>::quiet_NaN();
            g_videoDelaySum = 0.0; g_videoDelayCnt = 0;

            uint64_t curFileBytes = fSinkPtr->GetTotalRx();
            double fileTputKbps = ((curFileBytes - prevFileBytes) * 8.0) / 1000.0;
            prevFileBytes = curFileBytes;

            timeSeriesFile << std::fixed << std::setprecision(1) << now << ","
                           << std::setprecision(2) << emergTputKbps << ","
                           << emergDelay << ","
                           << videoTputKbps << ","
                           << vDelay << ","
                           << fileTputKbps << ","
                           << "Path_" << engine.GetActivePath(CLASS_EMERGENCY) << ","
                           << "Path_" << engine.GetActivePath(CLASS_VIDEO) << ","
                           << "Path_" << engine.GetActivePath(CLASS_FILE) << "\n";
        });
    }

    // 11. Enable FlowMonitor
    FlowMonitorHelper flowHelper;
    Ptr<FlowMonitor> monitor = flowHelper.InstallAll();

    // 12. Run Simulation
    Simulator::Stop(Seconds(config.simTime));
    std::cout << "[Simulator] Launching execution for " << config.simTime << " seconds...\n";
    Simulator::Run();

    // Write file transfer records
    fileManager.WriteResults();

    // 13. Performance Metrics Extraction
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowHelper.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    std::string flowMetricPath = config.resultsDir + "/flow_metrics.csv";
    bool flowExists = FileExistsAndNotEmpty(flowMetricPath);
    std::ofstream flowFile(flowMetricPath, std::ios::app);
    if (!flowExists) {
        flowFile << "Mode,Scenario,Class,TxPackets,RxPackets,LostPackets,ThroughputKbps,MeanDelayMs,MeanJitterMs,DeliveryRatio\n";
    }

    std::cout << "\n================ FlowMonitor Performance Results =================\n";
    std::cout << std::left << std::setw(15) << "Class"
              << std::setw(10) << "TxPkts"
              << std::setw(10) << "RxPkts"
              << std::setw(10) << "Lost"
              << std::setw(18) << "Throughput(Kbps)"
              << std::setw(14) << "MeanDelay(ms)"
              << std::setw(14) << "Jitter(ms)"
              << std::setw(10) << "PDR(%)" << "\n";
    std::cout << "------------------------------------------------------------------\n";

    for (auto it = stats.begin(); it != stats.end(); ++it) {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(it->first);
        std::string flowClass = "Other";
        if (t.destinationPort == config.emergPort) flowClass = "Emergency";
        else if (t.destinationPort == config.videoPort) flowClass = "Video";
        else if (t.destinationPort == config.filePort) flowClass = "FileTransfer";
        else continue;

        double duration = (it->second.timeLastRxPacket.GetSeconds() - it->second.timeFirstTxPacket.GetSeconds());
        double tputKbps = (duration > 0) ? (it->second.rxBytes * 8.0 / duration / 1024.0) : 0.0;
        double meanDelayMs = (it->second.rxPackets > 0) ? (it->second.delaySum.GetSeconds() / it->second.rxPackets * 1000.0) : 0.0;
        double meanJitterMs = (it->second.rxPackets > 1) ? (it->second.jitterSum.GetSeconds() / (it->second.rxPackets - 1) * 1000.0) : 0.0;
        double pdr = (it->second.txPackets > 0) ? (100.0 * it->second.rxPackets / it->second.txPackets) : 0.0;

        std::cout << std::left << std::setw(15) << flowClass
                  << std::setw(10) << it->second.txPackets
                  << std::setw(10) << it->second.rxPackets
                  << std::setw(10) << it->second.lostPackets
                  << std::setw(18) << std::fixed << std::setprecision(1) << tputKbps
                  << std::setw(14) << meanDelayMs
                  << std::setw(14) << meanJitterMs
                  << std::setw(10) << std::setprecision(2) << pdr << "\n";

        flowFile << config.mode << "," << config.scenario << "," << flowClass << ","
                 << it->second.txPackets << "," << it->second.rxPackets << "," << it->second.lostPackets << ","
                 << tputKbps << "," << meanDelayMs << "," << meanJitterMs << "," << pdr << "\n";
    }
    flowFile.close();

    // Summary Metrics
    uint32_t totalEmergLost = (g_emergencyStats.sent > g_emergencyStats.received)
                              ? (g_emergencyStats.sent - g_emergencyStats.received) : 0;
    uint32_t totalDeadlineViolations = g_emergencyStats.deadlineMisses + totalEmergLost;
    double missRatio = (g_emergencyStats.sent > 0) ? (100.0 * totalDeadlineViolations / g_emergencyStats.sent) : 0.0;
    double avgEmergDelay = (g_emergencyStats.received > 0) ? (g_emergencyStats.sumDelayMs / g_emergencyStats.received) : 0.0;

    std::string sumPath = config.resultsDir + "/summary_metrics.csv";
    bool sumExists = FileExistsAndNotEmpty(sumPath);
    std::ofstream sumFile(sumPath, std::ios::app);
    if (!sumExists) {
        sumFile << "Mode,Scenario,EmergSent,EmergRx,EmergViolations,DeadlineMissPct,MeanDelayMs,MaxDelayMs,"
                << "SwitchCount,FileDurationSec\n";
    }

    sumFile << config.mode << "," << config.scenario << ","
            << g_emergencyStats.sent << "," << g_emergencyStats.received << "," << totalDeadlineViolations << ","
            << std::fixed << std::setprecision(2) << missRatio << ","
            << avgEmergDelay << "," << g_emergencyStats.maxDelayMs << ","
            << g_systemMetrics.switchCount << ",";
    if (std::isnan(g_systemMetrics.fileCompletionTimeSec)) {
        sumFile << "NaN\n";
    } else {
        sumFile << g_systemMetrics.fileCompletionTimeSec << "\n";
    }
    sumFile.close();

    timeSeriesFile.close();
    g_failureLogger.Flush();

    // Fix P3: Check XML file size and warn if it exceeds maxXmlMB
    if (config.enableAnim) {
        std::ifstream xmlFile(config.animFile, std::ios::binary | std::ios::ate);
        if (xmlFile.is_open()) {
            double xmlSizeMB = static_cast<double>(xmlFile.tellg()) / (1024.0 * 1024.0);
            std::cout << "[NetAnim] XML Trace File Size: " << std::fixed << std::setprecision(2) << xmlSizeMB << " MB\n";
            if (xmlSizeMB > config.maxXmlMB) {
                std::cerr << "[WARNING] NetAnim XML size (" << xmlSizeMB << " MB) exceeds maxXmlMB (" << config.maxXmlMB << " MB)!\n";
            }
        }
    }

    Simulator::Destroy();
    if (anim) delete anim;

    std::cout << "\n[Execution Complete] Mode: " << config.mode << " | Scenario: " << config.scenario << "\n";
    std::cout << "Emergency Telemetry Deadline Miss Ratio: " << std::fixed << std::setprecision(2) << missRatio << " %\n";
    std::cout << "Dynamic Routing Switches Executed: " << g_systemMetrics.switchCount << "\n\n";

    return 0;
}