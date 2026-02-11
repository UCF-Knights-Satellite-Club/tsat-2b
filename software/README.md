## TSAT-2B Software
The stack is split into airborne firmware, ground side firmware, and a ground station application. Communication flows from the satellite to the ground station, with optional testing support.

## Air Firmware Requirements
To edit, compile, and flash the airborne firmware (`tsat.ino`), the following are required:

**Development Environment**
- Arduino IDE

**Required Libraries**
- [RFM69](https://github.com/LowPowerLab/RFM69) by LowPowerLab  
- [Adafruit BMP3XX](https://github.com/adafruit/Adafruit_BMP3XX) by Adafruit  
- [Adafruit MMA8451](https://github.com/adafruit/Adafruit_MMA8451_Library) by Adafruit  

## Air Segment (`air/tsat`) 
The firmware runs on the microcontroller mounted on the satellite.

### Responsibilities

* Sensor sampling (pressure, temperature, acceleration, altitude)
* Fixed telemetry encoding for bandwidth efficiency
* Framing and transmitting telemetry packets over serial
* Periodic ping packet support for link testing

### Packet Design

* All telemetry values are transmitted as fixed point integers (scaled ×1000)
* They begin with a small header:
  * Satellite ID
  * Packet type (`PING` or `TELEMETRY`)
* Telemetry packets include:
  * Frame counter
  * Timestamp
  * Altitude
  * Pressure
  * Temperature
  * Acceleration magnitude
  * Vertical velocity

This keeps packets compact and easy to deserialize on the ground.

## Ground Pipe (`ground_pipe`)
This firmware acts as a bridge between the satellite and the ground station.

### Responsibilities
* Receive raw telemetry bytes from the satellite
* Forwards packets over USB serial to the ground station
* Maintain clean packet boundaries
* Provide a stable baud-rate interface for Python ingestion

This layer is to isolate the serial handling from the visualization and processing.

## Ground Station Application (`ground`)

### Overview
The ground station is a live telemetry monitor with real time visualization using Dash.

**Dependencies:** `requirements.txt` 

### Key Features
* Live telemetry decoding from serial
* Packet deserialization mirroring embedded C structs
* Fixed point to floating point conversion
* Plotting dashboard
* Autoscaling window mode (last 60 seconds)
* Offline playback from recorded telemetry files
* Test graphing mode with to test live data visualization

### Packet Handling (Ground Side)
The ground station defines a strict mapping with the packet format:

* `PacketHeader`
* `PingPacket`
* `TelemetryPacket`
* `DataPoint` (fully converted, floating-point representation)

Deserialization is handled using Python’s struct module with little-endian layouts to match the ESP32/MCU architecture.

### Visualization Dashboard

The UI is built with Dash and Plotly, arranged in grids:

Tracked plots:
* Temperature (°C)
* Pressure (Pa)
* Altitude (m)
* Acceleration Magnitude (g)
* Ascent Velocity (m/s)

### Running the Ground Station

#### Navigate to Ground Folder & Create Python VENV

```bash
cd software/ground
python -m venv venv
source venv/bin/activate
```

#### Install dependencies

```bash
pip install -r requirements.txt
```

#### Live telemetry mode

```bash
python main.py --live <serial_port>
```

#### Playback from file

```bash
python main.py --from-file telemetry.csv
```

#### Test mode

```bash
python main.py --test-graphs
```