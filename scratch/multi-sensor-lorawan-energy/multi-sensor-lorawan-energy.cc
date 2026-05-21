
/*
 * This script simulates a LoRaWAN network with sensor energy consumption.
 *
 * Each end device cycles through: WakeUp -> Sensor Measurement -> LoRaWAN TX -> Sleep.
 * The Class A MAC manages all radio state transitions (TX, RX1, RX2, Sleep) autonomously.
 *
 * Energy models:
 *   - LoraRadioEnergyModel: radio (TX/RX/Standby/Sleep + per-state overhead charges)
 *   - MySimpleEnergyModel: sensor (y = a*x + b linear model with overhead charge)
 */

#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/command-line.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/file-helper.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/log.h"
#include "ns3/lora-energy-source-helper.h"
#include "ns3/lora-energy-source.h"
#include "ns3/lora-helper.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/my-simple-energy-model.h"
#include "ns3/names.h"
#include "ns3/node-container.h"
#include "ns3/packet.h"
#include "ns3/position-allocator.h"
#include "ns3/random-variable-stream.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <filesystem>
#include <sstream>

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("SensorLorawanEnergy");

// ---------------------------------------------------------------------------
// Global configuration (set via command line)
// ---------------------------------------------------------------------------
double initialEnergy = 20.0;    // Joules
double depletionTh = 0.1;       // fraction of initial energy
int numNodes = 1;
std::string outputPath = "multi-sensor-lorawan-energy-output";
std::string subfolder = "";

// Node current
double nodeWakeUpDuration = 0.14348;      // s
double nodeWakeUpCurrent = 0.03638;       // A
double nodeSleepCurrent = 0.000052;   // A (52 uA, entire node deep sleep)

// Sensor timing & current parameters
double measurementDuration = 0.01498;   // 14.98 ms
double measurementCurrent = 0.02149;   // 21.49 mA
double sensorOverhead = 0.0044376;    // 4437.6 uC
int numSample = 5;

// Radio current (A)
double radioSleepCurrent = 0.01722; //17.22 mA
// double radioStandbyCurrent = 0.2;
double radioTxCurrent = 0.1642; // 164.2 mA
double radioRxCurrent = 0.024236;   // 24.236 mA

// Radio overhead charges (Coulombs)
double radioTxOverhead = 0.0043887;  // 4388.7 uC
double radioRxOverhead = 0.0;
double standbyOverhead = 0.00018556;    // 185.56 uC


int spreadingFactor = 7;           // LoRa spreading factor (7-12)

// LoRaWAN region
std::string regionStr = "EU";


// ---------------------------------------------------------------------------
// Global state
// ---------------------------------------------------------------------------
std::map<Ptr<const Packet>, Ptr<SimpleEndDeviceLoraPhy>> g_packetToPhyMap;
std::map<uint32_t, std::pair<double, double>> g_nodeDepletionRecord;
std::set<uint32_t> g_depletedNodes;
std::map<uint32_t, Ptr<MySimpleEnergyModel>> g_sensorModels;
std::map<uint32_t, Ptr<LoraRadioEnergyModel>> g_radioEnergyModels;

// ---------------------------------------------------------------------------
// Forward declarations of sensor state machine functions
// ---------------------------------------------------------------------------
void NodeWakeUp(Ptr<Node> node,
                int packetSize,
                double period,
                double stopTime);
void StartSensorMeasurement(Ptr<Node> node,
                            int packetSize,
                            double period,
                            double stopTime);
void SendPacketViaMac(Ptr<Node> node,
                      int packetSize,
                      double period,
                      double stopTime);
void OnTxCycleCompleted(Ptr<Node> node,
                        int packetSize,
                        double period,
                        double stopTime);

// ---------------------------------------------------------------------------
// RemainingEnergy trace callback
// ---------------------------------------------------------------------------
void
RemainingEnergy(const Ptr<Node> node, double oldValue, double remainingEnergy)
{
    double timestamp = Simulator::Now().GetSeconds();
    NS_LOG_INFO(timestamp << "s Node " << node->GetId()
                          << " remaining energy: " << remainingEnergy << "J");

    if (remainingEnergy <= initialEnergy * depletionTh)
    {
        auto [it, inserted] = g_depletedNodes.insert(node->GetId());
        if (inserted)
        {
            g_nodeDepletionRecord[node->GetId()] = {timestamp, remainingEnergy};
            NS_LOG_INFO("Node " << node->GetId()
                                << " depleted at " << timestamp << "s");

            // Stop all energy consumption for this node
            auto sit = g_sensorModels.find(node->GetId());
            if (sit != g_sensorModels.end())
            {
                sit->second->SetCurrentA(0.0);
            }
            auto rit = g_radioEnergyModels.find(node->GetId());
            if (rit != g_radioEnergyModels.end())
            {
                rit->second->SetTxCurrentA(0.0);
                rit->second->SetRxCurrentA(0.0);
                rit->second->SetStandbyCurrentA(0.0);
                rit->second->SetSleepCurrentA(0.0);
                rit->second->SetTxOverheadCharge(0.0);
                rit->second->SetRxOverheadCharge(0.0);
                rit->second->SetStandbyOverheadCharge(0.0);
            }
        }

        if (g_depletedNodes.size() == static_cast<size_t>(numNodes))
        {
            NS_LOG_INFO("All nodes depleted at time " << timestamp << "s");
            Simulator::Stop(Seconds(0));
        }
    }
}

// ---------------------------------------------------------------------------
// Sensor state machine
// ---------------------------------------------------------------------------

void
NodeWakeUp(Ptr<Node> node,
           int packetSize,
           double period,
           double stopTime)
{
    NS_LOG_FUNCTION_NOARGS();
    if (g_depletedNodes.count(node->GetId()))
        return;

    auto it = g_sensorModels.find(node->GetId());
    if (it != g_sensorModels.end())
    {
        it->second->SetCurrentA(nodeWakeUpCurrent);
        NS_LOG_DEBUG("Node " << node->GetId() << " wake-up ("
                             << nodeWakeUpCurrent << " A)");
    }

    Simulator::Schedule(Seconds(nodeWakeUpDuration),
                        &StartSensorMeasurement,
                        node, packetSize, period, stopTime);
}

void
StartSensorMeasurement(Ptr<Node> node,
                       int packetSize,
                       double period,
                       double stopTime)
{
    NS_LOG_FUNCTION_NOARGS();
    if (g_depletedNodes.count(node->GetId()))
        return;

    auto it = g_sensorModels.find(node->GetId());
    if (it != g_sensorModels.end())
    {
        it->second->ApplyOverheadCharge();
        it->second->SetCurrentA(measurementCurrent);
        NS_LOG_DEBUG("Node " << node->GetId() << " measuring ("
                             << measurementCurrent << " A)");
    }

    Simulator::Schedule(Seconds(measurementDuration*numSample),
                        &SendPacketViaMac,
                        node, packetSize, period, stopTime);
}

void
SendPacketViaMac(Ptr<Node> node,
                 int packetSize,
                 double period,
                 double stopTime)
{
    NS_LOG_FUNCTION_NOARGS();
    if (g_depletedNodes.count(node->GetId()))
        return;

    // Turn off sensor (radio TX+RX1+RX2 will follow, managed by MAC)
    auto it = g_sensorModels.find(node->GetId());
    if (it != g_sensorModels.end())
    {
        it->second->SetCurrentA(0.0);
        NS_LOG_DEBUG("Node " << node->GetId() << " sensor off");
    }

    // Activate radio sleep current for the LoRa TX cycle
    auto rit = g_radioEnergyModels.find(node->GetId());
    if (rit != g_radioEnergyModels.end())
    {
        rit->second->SetSleepCurrentA(radioSleepCurrent);
    }

    // Send via LoRaWAN MAC. The TxCycleCompleted trace will trigger node sleep.
    Ptr<LoraNetDevice> loraDevice = DynamicCast<LoraNetDevice>(node->GetDevice(0));
    NS_ASSERT(loraDevice);
    Ptr<LorawanMac> mac = loraDevice->GetMac();
    NS_ASSERT(mac);

    // WORKAROUND: ns-3 LoRaWAN MAC does not append the 4-byte MIC.
    // Add 4 bytes to the application payload so the PHY frame size matches
    // the real LoRaWAN standard (MHDR + FHDR + FPort + Payload + MIC).
    Ptr<Packet> packet = Create<Packet>(packetSize + 4);
    NS_LOG_DEBUG("Node " << node->GetId() << " sending packet via MAC");
    mac->Send(packet);
}

// Called when the MAC's TX+RX1+RX2 cycle completes (TxCycleCompleted trace)
void
OnTxCycleCompleted(Ptr<Node> node,
                   int packetSize,
                   double period,
                   double stopTime)
{
    NS_LOG_FUNCTION_NOARGS();

    NS_LOG_INFO(Simulator::Now().GetSeconds()
                << "s Node " << node->GetId() << " LoRaWAN cycle complete");

    // Deactivate radio sleep current (LoRa cycle is done)
    auto rit = g_radioEnergyModels.find(node->GetId());
    if (rit != g_radioEnergyModels.end())
    {
        rit->second->SetSleepCurrentA(0.0);
    }

    // Set node to deep sleep current
    auto it = g_sensorModels.find(node->GetId());
    if (it != g_sensorModels.end())
    {
        it->second->SetCurrentA(nodeSleepCurrent);
        NS_LOG_DEBUG("Node " << node->GetId() << " node sleep ("
                             << nodeSleepCurrent << " A)");
    }

    // Do not schedule next cycle if node is depleted
    if (g_depletedNodes.count(node->GetId()))
        return;

    // Schedule next cycle
    double nextWakeTime = Simulator::Now().GetSeconds() + period;
    if (nextWakeTime < stopTime)
    {
        Simulator::Schedule(Seconds(period),
                            &NodeWakeUp,
                            node, packetSize, period, stopTime);
    }
    else
    {
        NS_LOG_DEBUG("Node " << node->GetId()
                             << " final cycle, sleeping until end of simulation");
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

void
PrintNodePositions(NodeContainer nodes)
{
    for (auto n = nodes.Begin(); n != nodes.End(); ++n)
    {
        Ptr<MobilityModel> mobility = (*n)->GetObject<ConstantPositionMobilityModel>();
        NS_ASSERT(mobility);
        Vector pos = mobility->GetPosition();
        NS_LOG_DEBUG("Node " << (*n)->GetId() << " position: " << pos);
    }
}

void
SaveNodeConfigToFile(NodeContainer endNodes,
                     NodeContainer gateways,
                     Ptr<LoraChannel> channel,
                     std::map<uint32_t, double> nodeTxStartTimes)
{
    std::filesystem::path outDir(outputPath);
    std::error_code ec;
    std::filesystem::create_directories(outDir, ec);
    if (ec)
    {
        NS_LOG_ERROR("Could not create output directory: " << outDir);
        return;
    }

    std::ofstream file((outDir / "node-config.csv").string());
    file << "NodeId,PosX,PosY,TxStartTime,DataRate" << std::endl;

    for (auto n = endNodes.Begin(); n != endNodes.End(); ++n)
    {
        Ptr<Node> node = *n;
        Ptr<MobilityModel> mobility = node->GetObject<ConstantPositionMobilityModel>();
        NS_ASSERT(mobility);
        Vector pos = mobility->GetPosition();

        Ptr<LoraNetDevice> loraDevice = DynamicCast<LoraNetDevice>(node->GetDevice(0));
        NS_ASSERT(loraDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            DynamicCast<ClassAEndDeviceLorawanMac>(loraDevice->GetMac());
        NS_ASSERT(mac);

        file << node->GetId() << ","
             << pos.x << "," << pos.y << ","
             << nodeTxStartTimes[node->GetId()] << ","
             << static_cast<int>(mac->GetDataRate())
             << std::endl;
    }
    file.close();
}

std::map<uint32_t, double>
AssignRandomStartTimes(NodeContainer nodes, double period)
{
    Ptr<UniformRandomVariable> rng = CreateObject<UniformRandomVariable>();
    rng->SetAttribute("Min", DoubleValue(0.0));
    rng->SetAttribute("Max", DoubleValue(period));

    std::map<uint32_t, double> times;
    for (auto n = nodes.Begin(); n != nodes.End(); ++n)
    {
        times[(*n)->GetId()] = rng->GetValue();
    }
    return times;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int
main(int argc, char* argv[])
{
    // Logging
    LogComponentEnable("SensorLorawanEnergy", LOG_LEVEL_ALL);
    LogComponentEnable("ClassAEndDeviceLorawanMac", LOG_LEVEL_ALL);
    LogComponentEnable("EndDeviceLoraPhy", LOG_LEVEL_ALL);
    LogComponentEnable("LoraPhy", LOG_LEVEL_ALL);
    LogComponentEnableAll(LOG_PREFIX_FUNC);
    LogComponentEnableAll(LOG_PREFIX_NODE);
    LogComponentEnableAll(LOG_PREFIX_TIME);

    // Command line
    int runNum = 1;
    int packetSize = 15;
    double period = 600.0;
    double stopTime = 600.0;
    double radius = 100.0;
    bool verify = false;

    CommandLine cmd(__FILE__);
    cmd.AddValue("runNum", "RNG run number", runNum);
    cmd.AddValue("packetSize", "Packet size (bytes)", packetSize);
    cmd.AddValue("period", "Period between measurements (s)", period);
    cmd.AddValue("stopTime", "Simulation stop time (s)", stopTime);
    cmd.AddValue("numNodes", "Number of end devices", numNodes);
    cmd.AddValue("radius", "Deployment radius (m)", radius);
    cmd.AddValue("initialEnergy", "Initial energy per node (J)", initialEnergy);
    cmd.AddValue("depletionTh", "Depletion threshold fraction", depletionTh);
    cmd.AddValue("nodeWakeUpDuration", "Node wake-up duration (s)", nodeWakeUpDuration);
    cmd.AddValue("nodeWakeUpCurrent", "Node wake-up current (A)", nodeWakeUpCurrent);
    cmd.AddValue("measurementDuration", "Sensor measurement duration (s)", measurementDuration);
    cmd.AddValue("measurementCurrent", "Sensor measurement current (A)", measurementCurrent);
    cmd.AddValue("numSample", "Number of sensor samples per measurement", numSample);
    cmd.AddValue("nodeSleepCurrent", "Node sleep current (A)", nodeSleepCurrent);
    cmd.AddValue("sensorOverhead", "Sensor overhead charge per measurement (C)", sensorOverhead);
    cmd.AddValue("radioTxOverhead", "Radio TX overhead charge (C)", radioTxOverhead);
    cmd.AddValue("radioRxOverhead", "Radio RX overhead charge (C)", radioRxOverhead);
    cmd.AddValue("standbyOverhead", "Radio Standby overhead charge (C)", standbyOverhead);
    cmd.AddValue("spreadingFactor", "LoRa spreading factor (7-12)", spreadingFactor);
    cmd.AddValue("region", "LoRaWAN region (EU, SingleChannel, ALOHA)", regionStr);
    cmd.AddValue("outputPath", "Output directory", outputPath);
    cmd.AddValue("subfolder", "Output subfolder name", subfolder);
    cmd.AddValue("verify", "verify energy consumption", verify);
    cmd.Parse(argc, argv);

    if (!subfolder.empty())
    {
        outputPath += "/" + subfolder;
    }

    // verify
    if(verify)
    {
        // Node current
         nodeWakeUpDuration = 0;      // s
         nodeWakeUpCurrent = 0;       // A
         nodeSleepCurrent = 0;   // A 

        // Sensor timing & current parameters
         measurementDuration = 0;   // s
         measurementCurrent = 0;   // A
         sensorOverhead = 0.0;          // C (sensor overhead charge per measurement)

        // Radio current (A)
         radioSleepCurrent = 0;
         radioTxCurrent = 0.1642; // 164.2 mA
         radioRxCurrent = 0;   // 

        // Radio overhead charges (Coulombs)
         radioTxOverhead = 0.0043887;  // 4388.7 uC
         radioRxOverhead = 0.0;
         standbyOverhead = 0.0;
    }

    // Setup output directory
    std::filesystem::path outDir(outputPath);
    std::error_code ec;
    std::filesystem::create_directories(outDir, ec);
    if (ec)
    {
        NS_LOG_ERROR("Could not create output directory: " << outDir);
        return 1;
    }

    RngSeedManager::SetRun(runNum);

    // Parse region
    LorawanMacHelper::Regions region = LorawanMacHelper::EU;
    if (regionStr == "SingleChannel")
        region = LorawanMacHelper::SingleChannel;
    else if (regionStr == "ALOHA")
        region = LorawanMacHelper::ALOHA;


    // -----------------------------------------------------------------------
    // Channel
    // -----------------------------------------------------------------------
    NS_LOG_INFO("Creating channel...");
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.76);
    loss->SetReference(1, 7.7);

    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------
    NS_LOG_INFO("Setting up helpers...");

    MobilityHelper mobility;
    mobility.SetPositionAllocator("ns3::UniformDiscPositionAllocator",
                                  "rho", DoubleValue(radius),
                                  "X", DoubleValue(0.0),
                                  "Y", DoubleValue(0.0),
                                  "Z", DoubleValue(0.0));
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    LoraPhyHelper phyHelper = LoraPhyHelper();
    phyHelper.SetChannel(channel);

    LorawanMacHelper macHelper = LorawanMacHelper();
    macHelper.SetRegion(region);

    LoraHelper helper = LoraHelper();
    helper.EnablePacketTracking();

    // -----------------------------------------------------------------------
    // End Devices
    // -----------------------------------------------------------------------
    NS_LOG_INFO("Creating end devices...");
    NodeContainer endNodes;
    endNodes.Create(numNodes);

    mobility.Install(endNodes);
    PrintNodePositions(endNodes);

    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);
    NetDeviceContainer endDevices = helper.Install(phyHelper, macHelper, endNodes);

    // -----------------------------------------------------------------------
    // Gateway
    // -----------------------------------------------------------------------
    NS_LOG_INFO("Creating gateway...");
    NodeContainer gateways;
    gateways.Create(1);

    Ptr<ListPositionAllocator> gwAllocator = CreateObject<ListPositionAllocator>();
    gwAllocator->Add(Vector(0, 0, 0));
    mobility.SetPositionAllocator(gwAllocator);
    mobility.Install(gateways);
    PrintNodePositions(gateways);

    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    helper.Install(phyHelper, macHelper, gateways);

    // Assign uniform spreading factor to all end devices
    {
        // distribution[0]=SF7, distribution[1]=SF8, ..., distribution[5]=SF12
        std::vector<double> distribution(6, 0.0);
        distribution[spreadingFactor - 7] = 1.0;
        LorawanMacHelper::SetSpreadingFactorsGivenDistribution(endNodes, gateways, distribution);
        NS_LOG_DEBUG("All nodes set to SF" << spreadingFactor);
    }

    // -----------------------------------------------------------------------
    // Energy Models
    // -----------------------------------------------------------------------
    NS_LOG_INFO("Installing energy models...");

    LoraEnergySourceHelper energySourceHelper;
    energySourceHelper.Set("LoraEnergySourceInitialEnergyJ", DoubleValue(initialEnergy));
    energySourceHelper.Set("LoraEnergySupplyVoltageV", DoubleValue(3.7));
    energySourceHelper.Set("LoraEnergyLowBatteryThreshold", DoubleValue(depletionTh));

    LoraRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Set("StandbyCurrentA", DoubleValue(radioRxCurrent));
    radioEnergyHelper.Set("TxCurrentA", DoubleValue(radioTxCurrent));
    radioEnergyHelper.Set("RxCurrentA", DoubleValue(radioRxCurrent));
    radioEnergyHelper.Set("SleepCurrentA", DoubleValue(radioSleepCurrent));
    
    radioEnergyHelper.Set("TxOverheadCharge", DoubleValue(radioTxOverhead));
    radioEnergyHelper.Set("RxOverheadCharge", DoubleValue(radioRxOverhead));
    radioEnergyHelper.Set("StandbyOverheadCharge", DoubleValue(standbyOverhead));
    radioEnergyHelper.SetTxCurrentModel("ns3::ConstantLoraTxCurrentModel",
                                        "TxCurrent", DoubleValue(radioTxCurrent));

    EnergySourceContainer sources = energySourceHelper.Install(endNodes);
    DeviceEnergyModelContainer radioModels =
        radioEnergyHelper.Install(endDevices, sources);

    // Install sensor energy model on each end node
    for (uint32_t i = 0; i < endNodes.GetN(); ++i)
    {
        Ptr<Node> node = endNodes.Get(i);
        Ptr<EnergySource> source = sources.Get(i);

        // Store radio energy model for dynamic SleepCurrentA control
        Ptr<LoraRadioEnergyModel> radioModel =
            DynamicCast<LoraRadioEnergyModel>(radioModels.Get(i));
        radioModel->SetSleepCurrentA(0.0);       // sensor phase: LoRa sleep is covered by node current
        g_radioEnergyModels[node->GetId()] = radioModel;

        Ptr<MySimpleEnergyModel> sensorModel = CreateObject<MySimpleEnergyModel>();
        sensorModel->SetEnergySource(source);
        sensorModel->SetOverheadCharge(sensorOverhead);
        source->AppendDeviceEnergyModel(sensorModel);
        sensorModel->SetCurrentA(0.0);

        g_sensorModels[node->GetId()] = sensorModel;

        // Connect TxCycleCompleted trace to trigger node sleep
        Ptr<LoraNetDevice> loraDevice = DynamicCast<LoraNetDevice>(node->GetDevice(0));
        NS_ASSERT(loraDevice);
        Ptr<ClassAEndDeviceLorawanMac> mac =
            DynamicCast<ClassAEndDeviceLorawanMac>(loraDevice->GetMac());
        NS_ASSERT(mac);
        mac->TraceConnectWithoutContext("TxCycleCompleted",
                                        MakeBoundCallback(&OnTxCycleCompleted,
                                                          node, packetSize, period, stopTime));
    }

    // Connect energy tracing
    for (auto i = sources.Begin(); i != sources.End(); ++i)
    {
        Ptr<LoraEnergySource> source = DynamicCast<LoraEnergySource>(*i);
        NS_ASSERT(source);
        const auto node = source->GetNode();
        NS_ASSERT(node);
        source->TraceConnectWithoutContext("RemainingEnergy",
                                           MakeBoundCallback(&RemainingEnergy, node));
    }

    // -----------------------------------------------------------------------
    // Schedule sensor cycles
    // -----------------------------------------------------------------------
    NS_LOG_INFO("Scheduling sensor cycles...");
    auto nodeTxStartTimes = AssignRandomStartTimes(endNodes, period);

    for (auto n = endNodes.Begin(); n != endNodes.End(); ++n)
    {
        Ptr<Node> node = *n;
        double startTime = nodeTxStartTimes[node->GetId()];
        Simulator::Schedule(Seconds(startTime),
                            &NodeWakeUp,
                            node, packetSize, period, stopTime);
    }

    // -----------------------------------------------------------------------
    // Run simulation
    // -----------------------------------------------------------------------
    NS_LOG_INFO("Starting simulation...");
    Simulator::Stop(Seconds(stopTime));

    auto simStart = std::chrono::high_resolution_clock::now();
    Simulator::Run();
    auto simEnd = std::chrono::high_resolution_clock::now();
    auto simDuration = std::chrono::duration_cast<std::chrono::milliseconds>(simEnd - simStart);

    NS_LOG_INFO("Simulation done in " << simDuration.count() << " ms");

    // -----------------------------------------------------------------------
    // Output
    // -----------------------------------------------------------------------
    LoraPacketTracker& tracker = helper.GetPacketTracker();
    auto gwId = gateways.Get(0)->GetId();
    std::string trackerStr =
        tracker.PrintPhyPacketsPerGw(Seconds(0), Seconds(stopTime), gwId);

    // Parse space-separated counts and save with labels
    {
        std::istringstream iss(trackerStr);
        int sent = 0, recv = 0, interf = 0, noRx = 0, underSens = 0, lostTx = 0;
        iss >> sent >> recv >> interf >> noRx >> underSens >> lostTx;

        std::ofstream pktFile((outDir / "packet-stats.csv").string());
        pktFile << "Metric,Count" << std::endl;
        pktFile << "Sent," << sent << std::endl;
        pktFile << "Received," << recv << std::endl;
        pktFile << "Interfered," << interf << std::endl;
        pktFile << "NoMoreReceivers," << noRx << std::endl;
        pktFile << "UnderSensitivity," << underSens << std::endl;
        pktFile << "LostBecauseTx," << lostTx << std::endl;

        std::cout << "Sent=" << sent
                  << " Received=" << recv
                  << " Interfered=" << interf
                  << " NoMoreRx=" << noRx
                  << " UnderSens=" << underSens
                  << " LostTx=" << lostTx
                  << std::endl;
    }

    SaveNodeConfigToFile(endNodes, gateways, channel, nodeTxStartTimes);

    Simulator::Destroy();

    // Write depletion summary
    {
        std::ofstream deplFile((outDir / "depletion.csv").string());
        deplFile << "NodeId,DepletionTime(s),RemainingEnergy(J)" << std::endl;
        for (uint32_t i = 0; i < endNodes.GetN(); ++i)
        {
            uint32_t nodeId = endNodes.Get(i)->GetId();
            auto it = g_nodeDepletionRecord.find(nodeId);
            if (it != g_nodeDepletionRecord.end())
            {
                deplFile << nodeId << ","
                         << it->second.first << ","
                         << it->second.second << std::endl;
            }
            else
            {
                deplFile << nodeId << ","
                         << stopTime << ","
                         << "not_depleted" << std::endl;
            }
        }
    }

    return 0;
}
