#!/bin/bash
set -e

cd ~/development/ns3-adropt-development/ns-3-dev

# FEC Configuration
VERBOSE=false
ADR_ENABLED=true
N_DEVICES=1
PERIODS_TO_SIMULATE=600
MOBILE_PROBABILITY=0.0
SIDE_LENGTH=4000
MAX_RANDOM_LOSS=36.0
GATEWAY_DISTANCE=8000
INITIALIZE_SF=false
MIN_SPEED=0.0
MAX_SPEED=0.0
OUTPUT_FILE="paper_replication_adr_fec.csv"

# *** OPTIMIZED FEC PARAMETERS FOR IMMEDIATE TESTING ***
FEC_ENABLED=true
FEC_GENERATION_SIZE=128
FEC_REDUNDANCY_RATIO=0.33   # DaRe-like redundancy

# Create timestamped log file
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_FILE="fec_simulation_${TIMESTAMP}.txt"

echo "=================================================================================="
echo "🚀 FEC-ENHANCED LORAWAN SIMULATION"
echo "=================================================================================="
echo "📅 Start time: $(date)"
echo "📁 Log file: $LOG_FILE"
echo "🔧 FEC Config: Generation size = $FEC_GENERATION_SIZE, Redundancy = ${FEC_REDUNDANCY_RATIO}00%"
echo "⏱️  Expected first FEC generation: ~19 minutes (8 packets × 144s)"
echo "=================================================================================="

# Run simulation with logging
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
  --outputFile=$OUTPUT_FILE \
  --FecEnabled=$FEC_ENABLED \
  --FecGenerationSize=$FEC_GENERATION_SIZE \
  --FecRedundancyRatio=$FEC_REDUNDANCY_RATIO" 2>&1 | tee "$LOG_FILE"

echo ""
echo "=================================================================================="
echo "✅ SIMULATION COMPLETED"
echo "=================================================================================="
echo "📅 End time: $(date)"
echo "📁 Console log: $LOG_FILE"

# Quick FEC check
echo ""
echo "🔍 QUICK FEC CHECK:"
if grep -q "🔍 FEC SendPacket" "$LOG_FILE"; then
    echo "   ✅ FEC packet processing detected!"
    echo "   📤 FEC events found:"
    grep "🔍 FEC SendPacket\|📤 SYSTEMATIC PACKET\|🎉 GENERATION.*COMPLETE" "$LOG_FILE" | head -3
else
    echo "   ❌ NO FEC packet processing found!"
    echo "   💡 Check trace connection in log"
fi

echo ""
echo "📋 Next steps:"
echo "   1. Run: python3 enhanced_fec_analyzer.py"
echo "   2. Check: $LOG_FILE for detailed output"
echo "=================================================================================="