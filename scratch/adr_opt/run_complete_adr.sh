#!/bin/bash
set -e

# Navigate to NS-3 directory
cd ~/development/ns3-adropt-development/ns-3-dev

# Paper replication parameters (Heusse et al. 2020)
VERBOSE=false
ADR_ENABLED=true
N_DEVICES=1
PERIODS_TO_SIMULATE=504        # 7 days = 504 periods of 20 minutes
MOBILE_PROBABILITY=0.0         # Static indoor device
SIDE_LENGTH=4000              # Urban area coverage
MAX_RANDOM_LOSS=36.0          # Urban fading
GATEWAY_DISTANCE=8000         # Gateway deployment spacing
INITIALIZE_SF=false           # Let ADRopt optimize
MIN_SPEED=0.0                 # Static device
MAX_SPEED=0.0                 # Static device
OUTPUT_FILE="paper_replication_adr.csv"

# Run simulation
./ns3 run "adr_opt/adr-opt-simulation \
  --verbose=$VERBOSE \
  --AdrEnabled=$ADR_ENABLED \
  --nDevices=$N_DEVICES \
  --PeriodsToSimulate=$PERIODS_TO_SIMULATE \
  --MobileNodeProbability=$MOBILE_PROBABILITY \
  --sideLength=$SIDE_LENGTH \
  --maxRandomLoss=$MAX_RANDOM_LOSS \
  --gatewayDistance=$GATEWAY_DISTANCE \
  --initializeSF=$INITIALIZE_SF \
  --MinSpeed=$MIN_SPEED \
  --MaxSpeed=$MAX_SPEED \
  --outputFile=$OUTPUT_FILE"