# NS-3 LoRaWAN ADRopt Project

This repository contains the LoRaWAN module and ADRopt algorithm implementation for NS-3.

## 📁 Structure

- `lorawan/` - LoRaWAN module for NS-3
- `scratch/` - Custom simulation projects including ADRopt

## 🚀 Setup

1. Clone NS-3.45:
   ```bash
   git clone https://gitlab.com/nsnam/ns-3-dev.git
   cd ns-3-dev
   git checkout ns-3.45 -b ns-3.45
   ```

2. Copy this project:
   ```bash
   cp -r path/to/this/repo/lorawan src/
   cp -r path/to/this/repo/scratch ./
   ```

3. Build:
   ```bash
   ./ns3 configure --enable-examples --enable-tests
   ./ns3 build
   ```

## 🧪 Usage

```bash
# Test standard LoRaWAN
./ns3 run adr-example -- --nDevices=1 --PeriodsToSimulate=2

# Run ADRopt (if implemented)
./ns3 run adropt/your-simulation
```
