# NS-3 LoRaWAN ADRopt Project

Acest proiect conține modulul LoRaWAN și implementarea algoritmului ADRopt pentru NS-3.

---

## 📦 Versiuni recomandate

- **NS-3:** 3.45
- **LoRaWAN module:** v0.3.4

> **IMPORTANT:** Pentru rezultate reproductibile, folosește exact aceste versiuni!

---

## 🚀 Instalare și configurare

### 1. Instalare pachete necesare (Ubuntu/Debian):

```bash
sudo apt update
sudo apt install -y g++ python3 python3-dev cmake ninja-build git ccache pkg-config sqlite3 libsqlite3-dev libxml2 libxml2-dev libgtk-3-dev vtun lxc uml-utilities libeigen3-dev gsl-bin libgsl-dev python3-pip
```

### 2. Clonare NS-3 și modul LoRaWAN:

```bash
# Creează un director nou pentru dezvoltare (opțional)
mkdir ns3-adropt-development
cd ns3-adropt-development

# Clonare NS-3
git clone https://gitlab.com/nsnam/ns-3-dev.git
cd ns-3-dev
git checkout ns-3.45 -b ns-3.45

# Clonare modul LoRaWAN direct în src/lorawan
git clone https://github.com/signetlabdei/lorawan src/lorawan
cd src/lorawan
git checkout v0.3.4 -b v0.3.4
cd ../..
```

### 3. Build și configurare

```bash
./ns3 configure --enable-examples --enable-tests
./ns3 build -j$(nproc)
```

### 4. Testare rapidă (opțional dar recomandat)

```bash
# Rulează testele pentru modulul LoRaWAN:
./ns3 test --suite=lorawan

# Rulează un exemplu simplu:
./ns3 run simple-network-example
```

🧪 Utilizare

```bash
# Listează exemplele LoRaWAN disponibile:
./ns3 run --list | grep lorawan

# Rulează un exemplu standard:
./ns3 run adr-example

# Rulează simularea ta ADRopt (după ce ai pus fișierele în scratch/adropt):
./ns3 run adropt/your-simulation
```

📚 Recomandări și troubleshooting

- Dacă întâmpini probleme la build:
  - Verifică dacă ai toate dependențele instalate
  - Asigură-te că versiunile de NS-3 și LoRaWAN corespund
  - Pentru rebuild curat:
    ```./ns3 clean && ./ns3 build ```

Ghid oficial: <https://www.nsnam.org/wiki/Installation>

📁 Structură proiect

- src/lorawan/ — Modulul LoRaWAN pentru NS-3
- scratch/adropt/ — Scripturile și simulările ADRopt (le poți pune aici)
- examples/ — Exemple standard din NS-3
