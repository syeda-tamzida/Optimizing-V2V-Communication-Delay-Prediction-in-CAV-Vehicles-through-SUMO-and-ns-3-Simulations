# Optimizing-V2V-Communication-Delay-Prediction-in-CAV-Vehicles-through-SUMO-and-ns-3-Simulations
Integration of SUMO mobility and ns3 wireless simulation is used to predict V2V delay in CAV platoons. Using 802.11p data and ML models like RF and XGBoost, delays are predicted from speed, distance, and RSSI. Results show accurate delay prediction for safer and more efficient CAV networks.

# Algorithm: SIMULATESUMO
This algorithm describes the procedure for simulating a 3x3 grid network with Connected and Autonomous Vehicle (CAV) platoons in SUMO.

# Algorithm 1: SIMULATESUMO(N, G, P, T, Δt)
Require: N = total number of vehicles,
        G = grid size (3×3, 400 m),
        P = number of platoons (6 platoons × 5 vehicles each),
        T = 100 s (total simulation time),
        Δt = 0.1 s (simulation step length)

Ensure: Vehicle trajectories, platoon dynamics, emission data (fcd-output)

1: Generate SUMO network net.xml with grid size G
2: Define vehicle type cav with attributes (length, max speed, accel, decel, σ)
3: Create 6 predefined routes (route1–route6) across grid edges
4: Assign 5 vehicles to each platoon with staggered departures (0–28 s)
5: Save vehicle and route definitions in road.rou.xml
6: Create configuration file road.sumocfg linking net.xml and rou.xml
7: Set simulation parameters: begin=0, end=100, step=0.1
8: Specify output file fcd-output.xml for full vehicle trajectories
9: Run SUMO simulation with sumo -c road.sumocfg
10: Collect outputs: positions and speeds for all platoons
11: Post-process to evaluate traffic performance and platoon stability
12: Return trajectory data (fcd-output.xml)
