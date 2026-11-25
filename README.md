# Autonomous Atmospheric Dropsonde Package (5 km Altitude Profiling)

An autonomous, lightweight atmospheric dropsonde system designed to profile meteorological parameters (**Temperature**, **Barometric Pressure**, **Relative Humidity**, **GPS Position/Altitude**, and **Horizontal Wind Speed/Direction**) from **5 km altitude down to ground level**, transmitting live telemetry over **433 MHz LoRa** to a portable ground station.

---

## 1. System Architecture & Components

### A. Airborne Dropsonde Unit
* **Microcontroller:** **ESP32-C3 SuperMini** (RISC-V single-core, native ESP-IDF firmware, internal SPIFFS flash backup logging to `/spiffs/flight_log.csv`).
* **GNSS / Wind Receiver:** **GY-NEO6MV2 (u-blox NEO-6M)** with **25 × 25 mm active ceramic patch antenna** (measures Latitude, Longitude, Altitude, Course Over Ground, and Ground Speed for passive parachute wind profiling).
* **Environmental Sensor:** **BME280** (3.3V I2C breakout; measures Ambient Temperature, Barometric Pressure, and Relative Humidity).
* **RF Transceiver:** **Ra-02 (Semtech SX1278)** 433 MHz LoRa module + **433 MHz antenna** (SF9, 125 kHz BW, Sync Word `0x12`, 28-byte packed binary telemetry at 1 Hz).
* **Power & Decoupling:**
  * **Recommended Battery:** **3.7V cylindrical Li-ion cell** (e.g., 16340, 14500, or 18650 — 4.2V full charge) wired to the `5V` rail, **OR** a **3.6V Li-SOCl₂ cell** (e.g., Saft LS14500 / Tadiran TL-5903) wired to the `3V3` rail. *(If using a 3.0V CR123A, use a 5V step-up boost converter or 2× CR123A in series).*
  * **Bulk Decoupling Capacitor:** **220 µF / 25 V** electrolytic capacitor soldered across `3V3` and `GND` near the Ra-02 to absorb +17/+18 dBm LoRa transmit current spikes.

### B. Ground Station Unit
* **Receiver Hardware:** **ESP32-C3 SuperMini** + **Ra-02 (SX1278)** 433 MHz LoRa module + **433 MHz antenna** (configured as a serial telemetry bridge at 115200 baud).
* **Host Interface:** Smartphone (via USB-OTG serial terminal) or PC running the Python real-time visualization and CSV logging script (`ground_station.py` with Matplotlib). (Install Matplotlib and pyserial to Python interpreter using `python -m pip install pyserial matplotlib`)

---

## 2. Airborne Hardware Wiring Pinout

| Subsystem | Module Pin | ESP32-C3 SuperMini Pin | Notes |
| :--- | :--- | :--- | :--- |
| **LoRa (SPI)** | `SCK` | `GPIO 4` | SPI Clock |
| | `MISO` | `GPIO 5` | Master In Slave Out |
| | `MOSI` | `GPIO 6` | Master Out Slave In |
| | `NSS` (`CS`) | `GPIO 7` | Chip Select |
| | `RST` | `GPIO 10` | Hardware Reset |
| | `DIO0` | `GPIO 3` | IRQ / TxDone |
| | `3.3V` / `GND` | `3V3` / `GND` | Place **220 µF** cap across rails |
| **BME280 (I2C)** | `SDA` | `GPIO 0` | I2C Data (`0x76` or `0x77`) |
| | `SCL` | `GPIO 1` | I2C Clock |
| | `VCC` / `GND` | `3V3` / `GND` | Leave `CSB` and `SDO` unconnected |
| **GPS (UART1)** | `TX` | `GPIO 20` | GPS TX → ESP32 RX |
| | `RX` | `GPIO 21` | GPS RX ← ESP32 TX |
| | `VCC` | `5V` (or `3V3`*) | *Use `5V` for 3.7V–5V supply; `3V3` only for 3.6V Li-SOCl₂ |
| | `GND` | `GND` | Common ground |

> **⚠️ Critical RF & Power Warnings:**
> 1. **Never power on the dropsonde or ground station without the 433 MHz antenna attached** to the Ra-02; transmitting into an open port can permanently damage the SX1278 power amplifier.
> 2. **Never connect the flight battery and USB-C cable simultaneously**; the ESP32-C3 SuperMini lacks reverse-current protection diodes on its `5V` and `3V3` pins.

---

## 3. Physical Assembly & Sensor Placement (Top to Bottom)
1. **Top Cap (Sky-Facing):** Mount the **GY-NEO6MV2 25 × 25 mm ceramic patch antenna** at the very top facing straight up for an unobstructed view of GPS satellites.
2. **Upper-Middle Core (Insulated):** Secure the **GY-NEO6MV2 PCB**, **ESP32-C3 SuperMini**, and **battery cell** centrally inside foam insulation to maintain center-of-gravity stability and protect the battery from sub-zero temperatures aloft.
3. **Lower-Middle Side Duct (Vented & Shaded):** Mount the **BME280** inside an open-ended, shaded airflow channel oriented parallel to vertical airflow (shielded from direct sunlight, falling water droplets, and downward ram air).
4. **Bottom Tail / Nose (Earth-Facing):** Mount the **Ra-02 (SX1278)** and **220 µF capacitor** at the bottom bay with the **433 MHz antenna** extending vertically downward to maximize RF isolation from the GPS patch above.

---

## 4. Operating Procedures

### Procedure A: Starting From Scratch (Pre-Flight & Bench Startup)

#### Step 1: Build & Flash Firmware (If Not Already Flashed)
1. Ensure the ESP-IDF project sits in a short, space-free directory path (e.g., `C:\esp\dropsonde_tx`) to avoid Windows `MAX_PATH` build errors.
2. Attach the **433 MHz antenna** to the dropsonde Ra-02.
3. Connect the dropsonde **ESP32-C3 SuperMini** to your PC via USB-C (with the flight battery **disconnected**).
4. Open the **ESP-IDF Terminal** in VS Code and run:
   ```bash
   idf.py flash monitor
   ```
5. Verify the startup logs confirm:
   * `BME280 found at 0x76 (Chip ID 0x60)`
   * `SX1278 LoRa initialized on 433.0 MHz`
   * Live 1 Hz telemetry lines (`TX #1 | Alt: ... | T: ... | P: ... | RH: ... | Sats: ...`).
6. Exit the serial monitor using **`Ctrl + ]`** and unplug the USB-C cable.

#### Step 2: Ground Station Setup
1. Verify the **433 MHz antenna** is attached to the ground station Ra-02.
2. Connect the ground station receiver to your PC or smartphone via USB.
3. Launch the ground station logger/plotter (e.g., `python ground_station.py`) and record the local sea-level barometric pressure (**QNH**) for altitude calibration.

#### Step 3: Outdoor GPS Acquisition & Launch
1. Take the dropsonde outdoors to an open area with clear sky visibility.
2. Connect the charged **flight battery** to the dropsonde (`+` to `5V` for 3.7V Li-ion, or `+` to `3V3` for 3.6V Li-SOCl₂; `-` to `GND`).
3. Keep the ceramic patch antenna facing straight upward for **1 to 3 minutes** until the GY-NEO6MV2 acquires a 3D satellite lock:
   * The onboard LED on the GY-NEO6MV2 begins **blinking at 1 Hz**.
   * The ground station receives packets showing `Sats >= 4` and a valid local altitude.
4. Close the enclosure, verify live telemetry packets are arriving every 1 second on the ground station, and deploy the dropsonde.

---

### Procedure B: Stopping & Resuming the Program

#### Scenario 1: Paused in Bootloader / Download Mode (`BOOT` + `RST`)
If the ESP32-C3 was paused while plugged into USB by holding `BOOT` and tapping `RST`:
1. Ensure you are **not** pressing the `BOOT` button.
2. Press and release the **`RST` (Reset)** button once on the ESP32-C3 SuperMini.
3. The microcontroller immediately exits ROM download mode, boots from flash, re-initializes the BME280/SX1278/GPS, and resumes logging to `/spiffs/flight_log.csv` and transmitting at 1 Hz.
   *(Alternatively, opening the ESP-IDF Serial Monitor via `idf.py monitor` automatically resets the board via USB DTR/RTS and resumes execution.)*

#### Scenario 2: Serial Monitor Closed on PC (`Ctrl + ]`)
If you only stopped the terminal display on your PC using `Ctrl + ]` while the ESP32-C3 remained powered:
* The dropsonde firmware **never stopped running**—it continued logging to SPIFFS flash and transmitting over LoRa in the background.
* To view the live stream again without resetting the packet counter, open any serial terminal at **115200 baud** with DTR/RTS reset disabled, or run `idf.py monitor` to reconnect and restart from `TX #1`.

#### Scenario 3: Powered Down (Battery / USB Disconnected)
1. Re-verify the **433 MHz antenna** is securely connected.
2. Reconnect either the **flight battery** or the **USB-C cable** (never both at the same time).
3. The dropsonde automatically boots within ~1.5 seconds, appends new telemetry rows to `/spiffs/flight_log.csv`, and resumes 433 MHz transmissions.