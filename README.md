# Smart Home Control System (WiFi + Firebase)

A complete IoT smart home system built with ESP32 that monitors room occupancy, controls lighting/appliances, and manages an AC unit via IR remote. All data syncs with Firebase Realtime Database and is accessible via a responsive web dashboard.

## Evolution: From MQTT to Firebase

**Initial Approach (MQTT):**
- Used Mosquitto MQTT broker running on a local laptop/mobile server
- Required the mobile device to always be running the broker
- Limited to local network access only
- Dashboard had to be self-hosted
- AC control feature was **not completed**

**Current Approach (Firebase):**
- Uses Firebase Realtime Database for cloud-based data sync
- No local server needed — accessible from anywhere
- Built-in web hosting for dashboard (Firebase Hosting)
- Easier authentication and REST API
- **AC control fully implemented as code only** with IR code capture and transmission
- Scalable and production-ready

---

## System Overview

The system combines multiple sensors and actuators:
- **IR Break Beams + PIR**: Directional occupancy detection
- **Web Dashboard**: Real-time monitoring and control from any device
- **Firebase Sync**: Central data repository for all device state
- **Device Control**: Lights (relay), atomizer, music player, and AC
- **IR Transmission**: AC temperature control via RG56V2/BGEF remote

### Occupancy Detection

```
ENTRANCE: outer beam breaks → inner beam breaks → PIR detects motion → count++
EXIT:      inner beam breaks → outer beam breaks → count--
```

- Both beams must break within **3 seconds** of each other
- A **2-second cooldown** applies after every detection
- Room is OCCUPIED when `count > 0`, EMPTY when `count == 0`
- On entrance: lights ON, atomizer ON, music plays
- On exit: lights OFF, atomizer OFF, music stops

---

## Features

✅ **Occupancy Detection** – Directional IR + PIR sensor fusion
✅ **Real-time Dashboard** – Control and monitor from web browser
✅ **Firebase Integration** – All state synced to Realtime Database
✅ **Device Control**:
   - Lamp (active-LOW relay, GPIO23)
   - Ultrasonic Atomizer (GPIO21)
   - Music Player (DFPlayer, UART2)
   - AC Unit (IR transmission, GPIO19)
✅ **Automatic Sync** – Dashboard updates in real-time without page refresh
 **AC Control** – Temperature slider 16–30°C via RG56V2/BGEF remote


---

## Hardware Components

| Component | Purpose | Pin |
|---|---|---|
| ESP32 | Main microcontroller | — |
| IR Break Beam (outer) | Entry/exit detection | GPIO 13 |
| IR Break Beam (inner) | Entry/exit detection | GPIO 14 |
| PIR Sensor (HC-SR501) | Motion direction confirmation | GPIO 25 |
| DHT22 / AM2302 | Temperature & humidity | GPIO 26 |
| Relay Module | Light control | GPIO 23 (active LOW) |
| Ultrasonic Atomizer | Mist control | GPIO 21 |
| DFPlayer Mini | Music playback | UART2 (GPIO 17 TX) |
| IR LED (transmitter) | AC remote control | GPIO 19 |

---

## Pin Connections

### IR Break Beams

**Outer (GPIO 13) — outside door:**
```
3.3V ──[10kΩ]──┬── GPIO 13
               │ (phototransistor collector)
              GND
```

**Inner (GPIO 14) — inside door:**
```
3.3V ──[10kΩ]──┬── GPIO 14
               │ (phototransistor collector)
              GND
```

### PIR Sensor (GPIO 25)
```
5V    ── VCC
GND   ── GND
GPIO 25 ── OUT
```

### DHT22 / AM2302 (GPIO 26)
```
3.3V  ── VCC
GPIO 26 ── DATA (with 10kΩ pull-up to 3.3V if needed)
GND   ── GND
```

### Relay (GPIO 23) — Active LOW
```
GPIO 23 ── IN1 (when LOW, relay energizes)
5V ── JD-VCC
GND ── GND
```

### Atomizer (GPIO 21)
```
GPIO 21 ── Control pin
GND ── GND
```

### DFPlayer Mini (UART2)
```
GPIO 17 (TX) ── RX (via 1kΩ resistor or level shifter)
GND ── GND
5V  ── VCC (from separate power supply)
```

### IR Transmitter (GPIO 19)
```
GPIO 19 ── LED Anode (via ~470Ω current-limiting resistor)
GND ── LED Cathode
```

---

## Firebase Setup

Before flashing, set up Firebase:

1. **Create Firebase Project**
   - Go to [console.firebase.google.com](https://console.firebase.google.com)
   - Create new project (e.g., `embeddedpro-573f6`)

2. **Enable Realtime Database**
   - Create database in test mode (for development)
   - Note the database URL: `https://<project>.firebaseio.com`

3. **Get API Key**
   - Go to Project Settings → Web API Key
   - Copy the API key

4. **Create `secrets.h`**
   ```c
   #define WIFI_SSID   "your-network-name"
   #define WIFI_PASS   "your-wifi-password"
   #define FIREBASE_HOST    "embeddedpro-573f6.firebaseio.com"
   #define FIREBASE_API_KEY "YOUR_WEB_API_KEY_HERE"
   ```

### Firebase Database Structure
```
smarthome/room001/
├── room       → "OCCUPIED" or "EMPTY"
├── count      → number of people
├── light      → "ON" or "OFF"
├── atomizer   → "ON" or "OFF"
├── music      → "PLAYING" or "STOPPED"
├── temperature → °C (DHT22)
├── humidity    → % (DHT22)
└── command    → "LIGHTS_ON" / "LIGHTS_OFF" / "ATOMIZER_ON" / "ATOMIZER_OFF" / "MUSIC_ON" / "MUSIC_OFF" / "AC_SET_TEMP:XX" / "STATUS"
```

---

## Dashboard

The web dashboard (`dashboard.html`) is hosted on **Firebase Hosting** at:
```
https://embeddedpro-573f6.web.app
```

### Features
- **Real-time Display**: Room status, people count, sensor readings
- **Control Buttons**: Lights, Atomizer, Music (On/Off)
- **AC Temperature Slider**: 16–30°C with +/– buttons
- **Responsive Design**: Works on desktop and mobile
- **Auto-refresh**: Updates every 3 seconds without user interaction

### Deploying Dashboard
```bash
npm install -g firebase-tools
firebase login
firebase deploy --only hosting
```

---

## AC Remote Control (In Progress)

**Remote Model**: RG56V2/BGEF
**Status**: Infrastructure ready, IR codes **pending capture** (not available in MQTT version)

> **Note**: AC control was planned but never started in the MQTT version. Infrastructure is now implemented in Firebase, but actual IR codes from the RG56V2/BGEF remote still need to be captured.

### How It Works
1. Dashboard sends "AC_SET_TEMP:<temp>" to Firebase `/command` node
2. ESP32 polls `/command` and parses the temperature
3. ESP32 transmits corresponding IR code to AC via GPIO19
4. AC adjusts temperature

### Capturing IR Codes

To get the actual IR codes from your remote:

1. Set `IR_CAPTURE_ENABLED` to `1` in `main.c`:
   ```c
   #define IR_CAPTURE_ENABLED 1
   ```

2. Compile and upload:
   ```bash
   idf.py build
   idf.py -p COM5 flash monitor
   ```

3. Open Serial Monitor (115200 baud)
   You'll see: `=== IR CODE CAPTURE MODE ===`

4. Point remote at ESP32's IR receiver (GPIO 13) and press buttons:
   - Press 16°C button → note: `IR CODE: addr=0xXX cmd=0xYY`
   - Press 17°C button → note: `IR CODE: addr=0xXX cmd=0xYY`
   - Continue through 30°C

5. Once collected, update `handle_ac_command()` in `main.c`:
   ```c
   static void handle_ac_command(int temp)
   {
       uint8_t cmd_codes[15] = {
           0xXX, // 16°C
           0xYY, // 17°C
           // ... continue through 30°C
       };
       uint32_t cmd = cmd_codes[temp - 16];
       send_ir_nec(0x01, cmd);
   }
   ```

6. Disable capture mode:
   ```c
   #define IR_CAPTURE_ENABLED 0
   ```

7. Recompile and upload

---

## Configuration

Update `secrets.h` with your credentials:

```c
#define WIFI_SSID        "your-ssid"
#define WIFI_PASS        "your-password"
#define FIREBASE_HOST    "your-project.firebaseio.com"
#define FIREBASE_API_KEY "your-web-api-key"
```

---

## Building and Flashing

Find your ESP32 COM port:
```powershell
Get-PnpDevice -Class Ports | Where-Object Status -eq 'OK'
```

Build, flash, and monitor:
```bash
idf.py -p COM5 flash monitor
```

Or step-by-step:
```bash
idf.py build
idf.py -p COM5 flash
idf.py -p COM5 monitor
```

Press **Ctrl+]** to exit monitor.

---

## Serial Output

### Startup
```
=== Smart Home — WiFi/Firebase build ===
WiFi connected. IP: 192.168.1.100
Firebase: anonymous sign-in OK
Firebase ready — publishing enabled.
System ready.
```

### Sensor Debug (every 2 seconds)
```
SENSORS: outer=1 inner=1 pir=0 | state=0
```

### Commands Received
```
Command received: LIGHTS_ON
firebase_put(/smarthome/room001/light) = "ON" ✓
```

### IR Capture Mode
```
=== IR CODE CAPTURE MODE ===
Press buttons on your remote — codes will be logged to serial
IR CODE: addr=0x01 cmd=0x10
```

---

## Troubleshooting

| Issue | Solution |
|---|---|
| WiFi won't connect | Check SSID/password in `secrets.h`, verify WiFi 2.4GHz |
| Dashboard shows "Unknown" | Check Firebase API Key, verify REST URL format |
| No sensor data in Firebase | Verify Firebase anonymous sign-in succeeded in logs |
| IR codes not capturing | Check GPIO13 connection, ensure remote is pointed correctly |
| AC not responding | Capture IR codes and verify they match remote output |

---

## Implementation Status

### Completed ✅
- WiFi + Firebase Realtime Database integration
- Occupancy detection (IR break beams + PIR sensor fusion)
- Device control (lights, atomizer, music player)
- Web dashboard with real-time sync
- IR transmitter hardware setup (GPIO19)
- IR code capture mode for remote learning
- Command polling and parsing infrastructure

### In Progress 🔄
- **AC Remote Control**: Capture actual IR codes from RG56V2/BGEF remote (16–30°C)
- Update `handle_ac_command()` with captured codes
- Test complete AC control flow end-to-end

### Future Enhancements
- [ ] Fine-tune sensor debouncing parameters
- [ ] Add temperature/humidity chart display to dashboard
- [ ] Add scheduling/automation features
