#!/bin/bash
set -e

# --- PAPER REPLICATION PARAMETERS (Heusse et al. 2020) - CORRECTED VERSION ---
# Navigate to the NS-3 development directory
cd ~/development/ns3-adropt-development/ns-3-dev

# Set parameters for the exact paper replication
VERBOSE=false
ADR_ENABLED=true
N_DEVICES=1                    # Paper: Single indoor test device

# *** CRITICAL FIX: Calculate correct periods for EXACTLY 7 days ***
# Paper target: 7 days = 7 × 24 × 60 = 10,080 minutes
# Periods of 20 minutes: 10,080 ÷ 20 = 504 periods
PERIODS_TO_SIMULATE=504        # FIXED: 7 days = 504 periods of 20 minutes
                               # This gives: 504 × 20 × 60 = 604,800 seconds = 7 days
                               # With 144s intervals: 604,800 ÷ 144 = 4,200 packets

MOBILE_PROBABILITY=0.0         # Paper: Static indoor device (3rd floor)
SIDE_LENGTH=4000              # Paper: Urban area coverage
MAX_RANDOM_LOSS=36.0          # Paper: Urban fading ~8dB std dev (equivalent to 36dB range)
GATEWAY_DISTANCE=8000         # Paper: Gateway deployment spacing
INITIALIZE_SF=false           # Paper: Let ADRopt optimize from default
MIN_SPEED=0.0                 # Paper: Static device
MAX_SPEED=0.0                 # Paper: Static device
OUTPUT_FILE="paper_replication_adr.csv"
OUTPUT_LOG="paper_replication_output.txt"

echo "🚀 Starting CORRECTED paper replication simulation..."
echo "🔧 Duration: EXACTLY 7 days (504 periods × 20 minutes)"
echo "📊 Expected packets: ~4200 (144s intervals over 7 days)"

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

echo "✅ Simulation completed."

# --- CORRECTED VALIDATION ---
echo ""
echo "📊 CORRECTED PAPER REPLICATION RESULTS:"
echo "============================================================"

# Extract key metrics if the output files are available
if [ -f "$OUTPUT_FILE" ]; then
    TOTAL_MEASUREMENTS=$(wc -l < $OUTPUT_FILE)
    echo "  📈 ADR measurements: $TOTAL_MEASUREMENTS data points"
fi

if [ -f "$OUTPUT_LOG" ]; then
    # Extract final PDR (should be ~85-99% for realistic simulation)
    FINAL_PDR=$(grep -o "Packet Delivery Rate (PDR): [0-9]*\.[0-9]*%" $OUTPUT_LOG | tail -1)
    if [ ! -z "$FINAL_PDR" ]; then
        echo "  📊 Final $FINAL_PDR"
        
        # Extract PDR value for validation
        PDR_VALUE=$(echo $FINAL_PDR | grep -o "[0-9]*\.[0-9]*" | head -1)
        if [ ! -z "$PDR_VALUE" ]; then
            # Convert to integer for comparison (e.g., 92.5 -> 92)
            PDR_INT=$(echo $PDR_VALUE | cut -d. -f1)
            
            if [ $PDR_INT -ge 99 ]; then
                echo "  🟢 EXCELLENT: Meeting paper's DER < 0.01 target"
            elif [ $PDR_INT -ge 95 ]; then
                echo "  🟡 GOOD: Close to paper's reliability target"
            elif [ $PDR_INT -ge 85 ]; then
                echo "  🟠 ACCEPTABLE: Typical LoRaWAN performance"
            else
                echo "  🔴 POOR: Below paper's ADRopt expectations"
            fi
        fi
    fi
    
    # Extract packet counts
    TOTAL_SENT=$(grep -o "Total packets transmitted: [0-9]*" $OUTPUT_LOG | tail -1)
    TOTAL_RECEIVED=$(grep -o "Total packets received: [0-9]*" $OUTPUT_LOG | tail -1)
    
    if [ ! -z "$TOTAL_SENT" ]; then
        echo "  📤 $TOTAL_SENT"
    fi
    if [ ! -z "$TOTAL_RECEIVED" ]; then
        echo "  📥 $TOTAL_RECEIVED"
    fi
    
    # Check for paper's expected packet count (~4200)
    if [ ! -z "$TOTAL_SENT" ]; then
        SENT_VALUE=$(echo $TOTAL_SENT | grep -o "[0-9]*")
        if [ $SENT_VALUE -ge 4000 ] && [ $SENT_VALUE -le 5000 ]; then
            echo "  ✅ Packet count matches paper's 1-week experiment"
        else
            echo "  ⚠️  Packet count: $SENT_VALUE (expected ~4200 for 7 days)"
        fi
    fi
    
    # Duration validation
    DURATION_DAYS=$(grep -o "Duration: [0-9]*\.[0-9]* seconds ([0-9]*\.[0-9]* days)" $OUTPUT_LOG | tail -1)
    if [ ! -z "$DURATION_DAYS" ]; then
        echo "  📅 $DURATION_DAYS"
        
        # Extract days value
        DAYS_VALUE=$(echo $DURATION_DAYS | grep -o "([0-9]*\.[0-9]* days)" | grep -o "[0-9]*\.[0-9]*")
        if [ ! -z "$DAYS_VALUE" ]; then
            # Check if close to 7 days (allow 6.5-7.5 range)
            if (( $(echo "$DAYS_VALUE >= 6.5 && $DAYS_VALUE <= 7.5" | bc -l) )); then
                echo "  ✅ Duration matches paper's 1-week target"
            else
                echo "  ⚠️  Duration: $DAYS_VALUE days (expected ~7 days)"
            fi
        fi
    fi
fi

echo ""
echo "🔧 CONFIGURATION CORRECTION APPLIED:"
echo "  • Changed from 4200 periods (58.3 days) to 504 periods (7 days)"
echo "  • Expected packet count: ~4200 (144s intervals over 7 days)"
echo "  • This should now match the paper's experimental setup exactly"
echo "============================================================"