#!/bin/bash
set -e

# Navigate to NS-3 development directory (in case script is called from elsewhere)
cd ~/development/ns3-adropt-development/ns-3-dev

# Set simulation parameters for 3 devices + 8 gateways scenario
# Optimized for realistic multi-device ADR testing
VERBOSE=false
ADR_ENABLED=true
N_DEVICES=3            # 3 end devices
APP_PERIOD=300         # Base period, but devices will have different intervals
PERIODS=24             # 2 hours simulation (24 * 5min intervals)
SIDE_LENGTH=1500       # 3x3km area (1.5km radius)
GATEWAY_DISTANCE=1000  # Strategic placement for coverage
MAX_RANDOM_LOSS=10     # Realistic channel variation
INITIALIZE_SF=false    # Let devices start with default SF
MOBILE_PROB=0          # Static devices for this test
MIN_SPEED=2
MAX_SPEED=16

# Run the simulation (all arguments are recognized by your C++ main)
./ns3 run "adr_opt/adr-opt-simulation --verbose=$VERBOSE --AdrEnabled=$ADR_ENABLED  --nDevices=$N_DEVICES  --PeriodsToSimulate=$PERIODS  --sideLength=$SIDE_LENGTH  --gatewayDistance=$GATEWAY_DISTANCE  --maxRandomLoss=$MAX_RANDOM_LOSS --initializeSF=$INITIALIZE_SF  --MobileNodeProbability=$MOBILE_PROB  --MinSpeed=$MIN_SPEED  --MaxSpeed=$MAX_SPEED" > multi_device_simulation.log 2>&1

echo "Multi-device simulation completed. Check multi_device_simulation.log for detailed results."