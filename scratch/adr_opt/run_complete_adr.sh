#!/bin/bash
set -e

# Navigate to NS-3 development directory (in case script is called from elsewhere)
cd ~/development/ns3-adropt-development/ns-3-dev

# Set simulation parameters (change as needed for your experiments)
# Optimized for 100 packets, quick simulation, good ADR testing
VERBOSE=false
ADR_ENABLED=true
N_DEVICES=1
APP_PERIOD=60          # 1 packet every 60 seconds
PERIODS=100            # 100 packets total
SIDE_LENGTH=3000       # Reduced for faster simulation & better gateway coverage
GATEWAY_DISTANCE=1500  # Closer gateways for better signal diversity
MAX_RANDOM_LOSS=5      # Some channel variation for ADR to adapt to
INITIALIZE_SF=false
MOBILE_PROB=0
MIN_SPEED=2
MAX_SPEED=16

# Run the simulation (all arguments are recognized by your C++ main)
./ns3 run "adr_opt/adr-opt-simulation --verbose=$VERBOSE --AdrEnabled=$ADR_ENABLED  --nDevices=$N_DEVICES  --PeriodsToSimulate=$PERIODS  --sideLength=$SIDE_LENGTH  --gatewayDistance=$GATEWAY_DISTANCE  --maxRandomLoss=$MAX_RANDOM_LOSS --initializeSF=$INITIALIZE_SF  --MobileNodeProbability=$MOBILE_PROB  --MinSpeed=$MIN_SPEED  --MaxSpeed=$MAX_SPEED" > complete_simulation.log 2>&1

echo "Simulation completed. Check complete_simulation.log for results."