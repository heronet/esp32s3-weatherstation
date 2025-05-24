# ESP32-S3 Weather Station with MQTT

A comprehensive weather monitoring system built with ESP32-S3 that measures environmental conditions and publishes data via MQTT protocol. Features a local OLED display and wireless connectivity for remote monitoring.

## 🌟 Features

- **Multi-sensor Environmental Monitoring**

  - Temperature, humidity, and atmospheric pressure (BME280)
  - Light intensity measurement (BH1750)
  - Magnetic field and compass heading (QMC5883L)
  - Calculated altitude based on sea level pressure

- **Real-time Data Visualization**

  - 128x64 OLED display with organized sensor readings
  - Visual light level indicator bar
  - Compass heading display

- **Wireless Connectivity**

  - WiFi connection with automatic reconnection
  - MQTT publishing for remote monitoring
  - Retained messages for reliable data delivery

- **Robust Operation**
  - Error handling and sensor validation
  - Automatic WiFi and MQTT reconnection
  - Configurable update intervals

## 🛠️ Hardware Requirements

| Component            | Model                 | Purpose                         |
| -------------------- | --------------------- | ------------------------------- |
| Microcontroller      | ESP32-S3              | Main processing unit            |
| Environmental Sensor | BME280                | Temperature, humidity, pressure |
| Light Sensor         | BH1750                | Ambient light measurement       |
| Magnetometer         | QMC5883L              | Magnetic field and compass      |
| Display              | SSD1306 OLED (128x64) | Local data visualization        |

### Pin Configuration

```
I2C SDA: Pin 17
I2C SCL: Pin 16
```

All sensors communicate via I2C protocol on the shared bus.

## 📋 Dependencies

Add these libraries to your Arduino IDE or PlatformIO project:

```cpp
// Core libraries
#include <WiFi.h>
#include <Wire.h>

// Sensor libraries
#include <Adafruit_BME280.h>
#include <BH1750.h>
#include <QMC5883LCompass.h>

// Display library
#include <Adafruit_SSD1306.h>

// MQTT library
#include <AsyncMqttClient.h>
```

### Library Installation

**Arduino IDE:**

1. Go to Tools → Manage Libraries
2. Search and install each required library
3. Ensure you have the ESP32 board package installed

**PlatformIO:**

```ini
lib_deps =
    adafruit/Adafruit BME280 Library
    adafruit/Adafruit SSD1306
    claws/BH1750
    mprograms/QMC5883LCompass
    marvinroger/AsyncMqttClient
```

## ⚙️ Configuration

### WiFi Settings

```cpp
constexpr const char *WIFI_SSID = "Your_WiFi_Network";
constexpr const char *WIFI_PASSWORD = "Your_Password";
```

### MQTT Broker Settings

```cpp
constexpr const char *MQTT_HOST = "your-mqtt-broker.local";
constexpr uint16_t MQTT_PORT = 1883;
constexpr const char *MQTT_USERNAME = "your_username";
constexpr const char *MQTT_PASSWORD = "your_password";
```

### Sensor Calibration

The QMC5883L magnetometer includes calibration values. Update these based on your specific sensor:

```cpp
compass.setCalibration(-1361, 1307, -1430, 1383, -1117, 1301);
```

To calibrate your magnetometer:

1. Run the sensor in various orientations
2. Record min/max values for each axis
3. Update the calibration parameters

## 📊 MQTT Topics

The system publishes data to the following topics:

| Topic                    | Data Type | Description                  |
| ------------------------ | --------- | ---------------------------- |
| `esp/bme280/temperature` | Float     | Temperature in Celsius       |
| `esp/bme280/humidity`    | Float     | Relative humidity (%)        |
| `esp/bme280/pressure`    | Float     | Atmospheric pressure (hPa)   |
| `esp/bme280/altitude`    | Float     | Calculated altitude (meters) |
| `esp/bh1750/light`       | Float     | Light intensity (lux)        |
| `esp/qmc5883l/x`         | Integer   | Magnetic field X-axis        |
| `esp/qmc5883l/y`         | Integer   | Magnetic field Y-axis        |
| `esp/qmc5883l/z`         | Integer   | Magnetic field Z-axis        |
| `esp/qmc5883l/heading`   | Float     | Compass heading (degrees)    |

All messages are published with QoS 1 and retained flag for reliability.

## 🔧 Installation & Setup

1. **Hardware Assembly**

   - Connect all sensors to the ESP32-S3 via I2C
   - Ensure proper power supply (3.3V for sensors)
   - Double-check wiring connections

2. **Software Setup**

   - Install required libraries
   - Update WiFi and MQTT credentials
   - Adjust sea level pressure for your location
   - Upload the code to your ESP32-S3

3. **Calibration**
   - Power on the device
   - Allow sensors to stabilize
   - Calibrate magnetometer if needed

## 📱 Display Layout

The OLED display shows real-time data in an organized format:

```
┌─────────────────────────────┐
│     WEATHER STATION         │
├─────────────────────────────┤
│ T: 25.1°C      H: 65.2%     │
├─────────────────────────────┤
│ P: 1013hPa     A: 25m       │
├─────────────────────────────┤
│ L: 245.6lux    C: 180°      │
│ 0 ████████████░░░░░░░░░░ 1K │
└─────────────────────────────┘
```

The bottom bar visualizes light intensity from 0 to 1000 lux.

## 🔍 Monitoring & Debugging

The system provides comprehensive serial output for debugging:

- Sensor readings every 500ms
- WiFi connection status
- MQTT connection and publish confirmations
- Error messages for troubleshooting

Connect to the serial monitor at 115200 baud to view real-time logs.

## 🚀 Usage Examples

### Home Automation Integration

Integrate with Home Assistant, OpenHAB, or similar platforms using MQTT:

```yaml
# Home Assistant sensor configuration
sensor:
  - platform: mqtt
    name: "Weather Station Temperature"
    state_topic: "esp/bme280/temperature"
    unit_of_measurement: "°C"

  - platform: mqtt
    name: "Weather Station Humidity"
    state_topic: "esp/bme280/humidity"
    unit_of_measurement: "%"
```

### Data Logging

Set up MQTT subscribers to log historical data:

```python
import paho.mqtt.client as mqtt
import json
from datetime import datetime

def on_message(client, userdata, message):
    data = {
        'timestamp': datetime.now().isoformat(),
        'topic': message.topic,
        'value': float(message.payload.decode())
    }
    # Save to database or file
    print(json.dumps(data))

client = mqtt.Client()
client.on_message = on_message
client.connect("your-mqtt-broker", 1883, 60)
client.subscribe("esp/+/+")
client.loop_forever()
```

## 🛠️ Customization

### Update Intervals

Modify the sensor reading frequency:

```cpp
constexpr unsigned long SENSOR_UPDATE_INTERVAL = 1000; // 1 second
```

### Additional Sensors

The I2C bus can support additional sensors. Add them to the `readSensors()` function and create corresponding MQTT topics.

### Display Customization

Modify the `updateDisplay()` function to change the layout, add graphics, or show different information.

## 🐛 Troubleshooting

### Common Issues

**Sensors not detected:**

- Check I2C wiring (SDA/SCL connections)
- Verify sensor addresses using I2C scanner
- Ensure adequate power supply

**WiFi connection problems:**

- Verify SSID and password
- Check signal strength
- Ensure ESP32-S3 is within range

**MQTT publishing fails:**

- Confirm broker address and credentials
- Check network connectivity
- Verify topic permissions

**Display issues:**

- Confirm OLED I2C address (usually 0x3C)
- Check display wiring
- Verify library compatibility

### Debug Commands

Enable verbose logging by modifying the serial output or adding debug flags.

## 📈 Performance

- **Update Rate:** 500ms (configurable)
- **Power Consumption:** ~80mA average (depends on WiFi usage)
- **Memory Usage:** Minimal with efficient sensor libraries
- **Connectivity:** Automatic reconnection for robust operation

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is open source. Please check the LICENSE file for details.

## 🙏 Acknowledgments

- Adafruit for excellent sensor libraries
- ESP32 community for development support
- Arduino ecosystem for making embedded development accessible

---

**Built with ❤️ for the maker community**
