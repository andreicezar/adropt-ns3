#!/bin/bash
set -e

# --- SIMULATION PARAMETERS ---
# Navigate to the NS-3 development directory
cd ~/development/ns3-adropt-development/ns-3-dev

# Set parameters for the simulation run
VERBOSE=false
ADR_ENABLED=true
N_DEVICES=1
PERIODS_TO_SIMULATE=1200 # ~1 week simulation time
MOBILE_PROBABILITY=0.0
SIDE_LENGTH=4000
MAX_RANDOM_LOSS=12.0 # Urban fading matching paper's ~8dB std dev
GATEWAY_DISTANCE=8000
INITIALIZE_SF=false
MIN_SPEED=0.0
MAX_SPEED=0.0
OUTPUT_FILE="paper_replication_adr.csv"
OUTPUT_LOG="paper_replication_output.txt"

# --- EXECUTION ---
echo "Starting build..."
./ns3 configure --build-profile=optimized
./ns3 build -j12

echo "Starting simulation..."
# Run the simulation, redirecting stdout and stderr to the log file
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
  --outputFile=$OUTPUT_FILE" > $OUTPUT_LOG 2>&1

echo "Simulation completed."

# --- RESULTS ANALYSIS ---
echo "Results:"
# Extract key metrics if the output files are available
if [ -f "$OUTPUT_FILE" ]; then
    TOTAL_MEASUREMENTS=$(wc -l < $OUTPUT_FILE)
    echo "  • Total measurements in $OUTPUT_FILE: $TOTAL_MEASUREMENTS data points"
fi

if [ -f "$OUTPUT_LOG" ]; then
    FINAL_PDR=$(grep -o "Packet Delivery Rate (PDR): [0-9]*\.[0-9]*%" $OUTPUT_LOG | tail -1)
    if [ ! -z "$FINAL_PDR" ]; then
        echo "  • $FINAL_PDR"
    fi
    
    TOTAL_SENT=$(grep -o "Total packets transmitted: [0-9]*" $OUTPUT_LOG | tail -1)
    if [ ! -z "$TOTAL_SENT" ]; then
        echo "  • $TOTAL_SENT"
    fi
fi