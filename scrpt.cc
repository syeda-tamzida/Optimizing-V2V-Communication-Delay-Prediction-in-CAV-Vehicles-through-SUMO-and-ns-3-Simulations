#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"

#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <vector>
#include <set>
#include <cmath>

using namespace ns3;

// ---------------- CSV ENTRY ----------------
struct CsvMobilityEntry {
    double time;
    std::string vehicleIdStr; // SUMO ID (e.g. "p1_1")
    double x;
    double y;
    double speed;
};

// ---------------- GLOBAL MAPS ----------------
std::map<std::string,int> vehStringToInt;               // SUMO ID → int
std::map<int, Ptr<Node>> nodeMap;                       // int → ns-3 Node
std::map<int, double> vehicleSpeedMap;                  // int → current speed
std::map<int, Vector> vehiclePosMap;                    // int → position
std::map<Ptr<Socket>, int> socketReceiverMap;           // socket → receiver int
std::map<std::pair<int,int>, Ptr<Socket>> sendSockets;  // (sender,receiver) → socket

// ---------------- LOAD CSV ----------------
std::vector<CsvMobilityEntry> LoadCsv(const std::string &filename) {
    std::vector<CsvMobilityEntry> entries;
    std::ifstream file(filename);
    if (!file.is_open()) {
        NS_FATAL_ERROR("Could not open CSV file " << filename);
    }

    std::string line;
    getline(file, line); // skip header

    while (getline(file, line)) {
        std::stringstream ss(line);
        CsvMobilityEntry entry;
        std::string timeStr, vehStr, xStr, yStr, speedStr;

        getline(ss, timeStr, ',');
        getline(ss, vehStr, ',');
        getline(ss, xStr, ',');
        getline(ss, yStr, ',');
        getline(ss, speedStr, ',');

        entry.time = std::stod(timeStr);
        entry.vehicleIdStr = vehStr;
        entry.x = std::stod(xStr);
        entry.y = std::stod(yStr);
        entry.speed = std::stod(speedStr);

        entries.push_back(entry);
    }
    return entries;
}

// ---------------- UPDATE POSITION ----------------
void SetNodePosition(int vehicleId, double x, double y, double speed) {
    Ptr<Node> node = nodeMap[vehicleId];
    Ptr<MobilityModel> mob = node->GetObject<MobilityModel>();
    mob->SetPosition(Vector(x, y, 0));
    vehicleSpeedMap[vehicleId] = speed;
    vehiclePosMap[vehicleId] = Vector(x, y, 0);
}

// ---------------- RECEIVE CALLBACK ----------------
static void ReceivePacketCallback(Ptr<Socket> socket) {
    int receiverId = socketReceiverMap[socket];
    Ptr<Packet> packet;
    Address from;

    while ((packet = socket->RecvFrom(from))) {
        Time recvTime = Simulator::Now();

        uint64_t sendTimeNs;
        int senderId;

        // Copy first 12 bytes: 8 for sendTime, 4 for senderId
        uint8_t buffer[12];
        packet->CopyData(buffer, 12);
        sendTimeNs = *((uint64_t*)buffer);
        senderId = *((int*)(buffer + 8));

        double delay = (recvTime - Time::FromInteger(sendTimeNs, Time::NS)).GetMilliSeconds();

        double distance = std::sqrt(
            std::pow(vehiclePosMap[senderId].x - vehiclePosMap[receiverId].x, 2) +
            std::pow(vehiclePosMap[senderId].y - vehiclePosMap[receiverId].y, 2)
        );

        uint32_t packetSize = packet->GetSize();

        // ---------------- RSSI ----------------
        // Friis free-space path loss model (simplified)
        double txPowerDbm = 20.0;  // transmit power in dBm
        double freqHz = 5.9e9;     // 5.9 GHz for 802.11p
        double rssiDbm = txPowerDbm + 20*std::log10(3e8/(4*M_PI*freqHz*distance));
        // Optional: clamp RSSI to minimum realistic value
        if (rssiDbm < -100) rssiDbm = -100;

        // ---------------- PACKET LOSS ----------------
        int packetLoss = (rssiDbm < -85) ? 1 : 0; // simple threshold-based loss

        std::ofstream out("6_platoon_nsm_output.csv", std::ios::app);
        out << recvTime.GetSeconds() << ","
            << senderId << ","
            << receiverId << ","
            << distance << ","
            << vehicleSpeedMap[senderId] << ","
            << vehicleSpeedMap[receiverId] << ","
            << packetSize << ","
            << rssiDbm << ","
            << delay << ","
            << packetLoss
            << "\n";
        out.close();
    }
}

// ---------------- MAIN ----------------
int main(int argc, char *argv[]) {
    std::string inputCsv = "/home/lebu/ns-allinone-3.45/ns-3.45/scratch/fcd_data_one.csv";
    double simTime = 100.0;

    CommandLine cmd;
    cmd.AddValue("inputCsv", "Mobility trace CSV", inputCsv);
    cmd.Parse(argc, argv);

    auto entries = LoadCsv(inputCsv);

    std::set<std::string> uniqueVehs;
    for (auto &e : entries) uniqueVehs.insert(e.vehicleIdStr);

    uint32_t numVehicles = uniqueVehs.size();

    // Create nodes
    NodeContainer nodes;
    nodes.Create(numVehicles);

    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    // Map SUMO string IDs → int
    int idx = 0;
    for (auto &vidStr : uniqueVehs) {
        vehStringToInt[vidStr] = idx;
        nodeMap[idx] = nodes.Get(idx);
        vehicleSpeedMap[idx] = 0.0;
        vehiclePosMap[idx] = Vector(0,0,0);
        idx++;
    }

    // Schedule mobility updates
    for (auto &e : entries) {
        int vehId = vehStringToInt[e.vehicleIdStr];
        Simulator::Schedule(Seconds(e.time),
                            &SetNodePosition,
                            vehId, e.x, e.y, e.speed);
    }

    // Wi-Fi setup
    WifiHelper wifi;
    wifi.SetStandard(WifiStandard::WIFI_STANDARD_80211p);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");

    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    InternetStackHelper internet;
    internet.Install(nodes);

    Ipv4AddressHelper ipv4;
    ipv4.SetBase("10.1.0.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

    // CSV header
    std::ofstream out("6_platoon_nsm_output.csv");
    out << "Time,SenderID,ReceiverID,Distance,SpeedSender,SpeedReceiver,PacketSize,RSSI,Delay,PacketLoss\n";
    out.close();

    // ---------------- SOCKETS ----------------
    TypeId tid = TypeId::LookupByName("ns3::UdpSocketFactory");

    for (auto &sender : vehStringToInt) {
        for (auto &receiver : vehStringToInt) {
            if (sender.second == receiver.second) continue;

            uint16_t port = 8000 + receiver.second;

            // Receiver
            Ptr<Socket> recvSock = Socket::CreateSocket(nodeMap[receiver.second], tid);
            InetSocketAddress local = InetSocketAddress(Ipv4Address::GetAny(), port);
            recvSock->Bind(local);
            socketReceiverMap[recvSock] = receiver.second;
            recvSock->SetRecvCallback(MakeCallback(&ReceivePacketCallback));

            // Sender
            Ptr<Socket> sendSock = Socket::CreateSocket(nodeMap[sender.second], tid);
            InetSocketAddress remote = InetSocketAddress(interfaces.GetAddress(receiver.second), port);
            sendSock->Connect(remote);
            sendSockets[{sender.second, receiver.second}] = sendSock;
        }
    }

    // ---------------- SEND PACKETS ----------------
    for (double t = 0.1; t < simTime; t += 0.1) {
        for (auto &[pair, sock] : sendSockets) {
            int senderId = pair.first;
            Simulator::Schedule(Seconds(t), [sock, senderId]() {
                uint64_t sendTime = Simulator::Now().GetNanoSeconds();
                uint8_t buffer[12];
                *((uint64_t*)buffer) = sendTime;
                *((int*)(buffer+8)) = senderId;
                Ptr<Packet> p = Create<Packet>(buffer, 12);
                sock->Send(p);
            });
        }
    }

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    return 0;
}
