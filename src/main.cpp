/**
 * Weather Station for ESP32-S3 with MQTT
 *
 * Hardware components:
 * - ESP32-S3 board
 * - BME280 temperature/humidity/pressure sensor
 * - BH1750 light sensor
 * - QMC5883L magnetometer
 * - SSD1306 OLED display (128x64)
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_BME280.h>
#include <Adafruit_SSD1306.h>
#include <BH1750.h>
#include <QMC5883LCompass.h>
#include <AsyncMqttClient.h>

// Function declarations
void readSensors();
void updateDisplay();
void printSensorValues();
void connectToWifi();
void connectToMqtt();
void WiFiEvent(WiFiEvent_t event);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason);
void onMqttPublish(uint16_t packetId);

// WiFi credentials
constexpr const char *WIFI_SSID = "Galaxy Note10+ 5G";
constexpr const char *WIFI_PASSWORD = "00000000";

// MQTT Broker
constexpr const char *MQTT_HOST = "raspberrypi.local";
constexpr uint16_t MQTT_PORT = 1883;
constexpr const char *MQTT_USERNAME = "";
constexpr const char *MQTT_PASSWORD = "";

// MQTT Topics
constexpr const char *MQTT_PUB_TEMP = "esp/bme280/temperature";
constexpr const char *MQTT_PUB_HUM = "esp/bme280/humidity";
constexpr const char *MQTT_PUB_PRES = "esp/bme280/pressure";
constexpr const char *MQTT_PUB_ALT = "esp/bme280/altitude";
constexpr const char *MQTT_PUB_LIG = "esp/bh1750/light";
constexpr const char *MQTT_PUB_MAG_X = "esp/qmc5883l/x";
constexpr const char *MQTT_PUB_MAG_Y = "esp/qmc5883l/y";
constexpr const char *MQTT_PUB_MAG_Z = "esp/qmc5883l/z";
constexpr const char *MQTT_PUB_HEADING = "esp/qmc5883l/heading";

// Pin definitions
constexpr uint8_t I2C_SDA = 17;
constexpr uint8_t I2C_SCL = 16;

// Constants
constexpr uint8_t SCREEN_WIDTH = 128;
constexpr uint8_t SCREEN_HEIGHT = 64;
constexpr unsigned long SENSOR_UPDATE_INTERVAL = 500; // Match MQTT publish rate
constexpr float SEA_LEVEL_PRESSURE_HPA = 1010.0f;     // Sylhet sea level pressure in hPa

// Global objects
Adafruit_BME280 bme;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
BH1750 lightSensor;
QMC5883LCompass compass; // Changed to QMC5883L compass object
AsyncMqttClient mqttClient;

// Timer handlers for reconnection
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

// Timing variables
unsigned long lastSensorUpdate = 0;

// Sensor readings
float temperature, humidity, pressure, altitude, lightLevel;
// Magnetometer readings
int magX, magY, magZ; // QMC5883L library uses int values
float heading;

void setup()
{
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("\nWeather Station Starting...");

  // Create timers for reconnection
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(SENSOR_UPDATE_INTERVAL), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(SENSOR_UPDATE_INTERVAL), pdFALSE, (void *)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  // Set WiFi event handler
  WiFi.onEvent(WiFiEvent);

  // Configure MQTT client
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USERNAME, MQTT_PASSWORD);

  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialize BME280 sensor
  if (!bme.begin(0x76))
  {
    Serial.println("Could not find BME280 sensor!");
    while (1)
      delay(10);
  }

  // Initialize light sensor
  if (!lightSensor.begin())
  {
    Serial.println("Could not find BH1750 light sensor!");
    while (1)
      delay(10);
  }

  // Initialize QMC5883L magnetometer
  compass.init();
  // Optional settings for QMC5883L
  compass.setCalibration(-1361, 1307, -1430, 1383, -1117, 1301);
  compass.setSmoothing(10, true); // Set smoothing to 10 readings with weighted smooth

  // Initialize OLED display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println("SSD1306 allocation failed");
    while (1)
      delay(10);
  }
  display.clearDisplay();
  display.display();

  // Set BME280 sea level pressure for altitude calculation
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X16, // temperature
                  Adafruit_BME280::SAMPLING_X16, // pressure
                  Adafruit_BME280::SAMPLING_X16, // humidity
                  Adafruit_BME280::FILTER_X16,
                  Adafruit_BME280::STANDBY_MS_500);

  // Initial sensor readings
  readSensors();
  updateDisplay();

  // Connect to WiFi
  connectToWifi();

  Serial.println("Setup complete!");
}

void loop()
{
  unsigned long currentTime = millis();

  // Update sensor readings, display, and publish MQTT data
  if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL)
  {
    readSensors();
    printSensorValues();
    updateDisplay();

    // Publish sensor data to MQTT if connected
    if (mqttClient.connected())
    {
      // Publish temperature
      uint16_t packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(temperature).c_str());
      Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_TEMP, packetIdPub1);
      Serial.printf("Message: %.2f \n", temperature);

      // Publish humidity
      uint16_t packetIdPub2 = mqttClient.publish(MQTT_PUB_HUM, 1, true, String(humidity).c_str());
      Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HUM, packetIdPub2);
      Serial.printf("Message: %.2f \n", humidity);

      // Publish pressure
      uint16_t packetIdPub3 = mqttClient.publish(MQTT_PUB_PRES, 1, true, String(pressure).c_str());
      Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_PRES, packetIdPub3);
      Serial.printf("Message: %.2f \n", pressure);

      // Publish light level
      uint16_t packetIdPub4 = mqttClient.publish(MQTT_PUB_LIG, 1, true, String(lightLevel).c_str());
      Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_LIG, packetIdPub4);
      Serial.printf("Message: %.2f \n", lightLevel);

      // Publish altitude
      uint16_t packetIdPub5 = mqttClient.publish(MQTT_PUB_ALT, 1, true, String(altitude).c_str());
      Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_ALT, packetIdPub5);
      Serial.printf("Message: %.2f \n", altitude);

      // Publish magnetometer data
      uint16_t packetIdPub6 = mqttClient.publish(MQTT_PUB_MAG_X, 1, true, String(magX).c_str());
      Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_MAG_X, packetIdPub6);
      Serial.printf("Message: %d \n", magX);

      uint16_t packetIdPub7 = mqttClient.publish(MQTT_PUB_MAG_Y, 1, true, String(magY).c_str());
      Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_MAG_Y, packetIdPub7);
      Serial.printf("Message: %d \n", magY);

      uint16_t packetIdPub8 = mqttClient.publish(MQTT_PUB_MAG_Z, 1, true, String(magZ).c_str());
      Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_MAG_Z, packetIdPub8);
      Serial.printf("Message: %d \n", magZ);

      uint16_t packetIdPub9 = mqttClient.publish(MQTT_PUB_HEADING, 1, true, String(heading).c_str());
      Serial.printf("Publishing on topic %s at QoS 1, packetId %i: ", MQTT_PUB_HEADING, packetIdPub9);
      Serial.printf("Message: %.2f \n", heading);
    }
    else
    {
      Serial.println("MQTT not connected. Skipping publishing.");
    }

    // Update the last sensor update time
    lastSensorUpdate = currentTime;
  }

  // Small delay to prevent tight loop
  delay(10);
}

// WiFi event handler
void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
  case ARDUINO_EVENT_WIFI_STA_GOT_IP:
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
    connectToMqtt();
    break;
  case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    Serial.println("WiFi lost connection");
    xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
    xTimerStart(wifiReconnectTimer, 0);
    break;
  }
}

void connectToWifi()
{
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt()
{
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void onMqttConnect(bool sessionPresent)
{
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
  Serial.println("Disconnected from MQTT.");
  if (WiFi.isConnected())
  {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttPublish(uint16_t packetId)
{
  Serial.print("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void readSensors()
{
  try
  {
    // Read values from BME280
    temperature = bme.readTemperature();
    humidity = bme.readHumidity();
    pressure = bme.readPressure() / 100.0F;              // Convert Pa to hPa
    altitude = bme.readAltitude(SEA_LEVEL_PRESSURE_HPA); // Use a known sea level pressure for altitude calculation

    // Read light level from BH1750
    lightLevel = lightSensor.readLightLevel();

    // Read magnetometer values from QMC5883L
    compass.read(); // Read the compass values

    // Get raw magnetometer values
    magX = compass.getX();
    magY = compass.getY();
    magZ = compass.getZ();

    // Get heading in degrees (0-360)
    heading = compass.getAzimuth();

    // Optional: Get cardinal direction as text
    // const char* direction = compass.getDirection();
  }
  catch (...)
  {
    Serial.println("Error reading sensors");
  }
}

void printSensorValues()
{
  Serial.println("\nCurrent sensor readings:");
  Serial.print("Temperature: ");
  Serial.print(temperature, 1);
  Serial.println(" C");
  Serial.print("Humidity: ");
  Serial.print(humidity, 1);
  Serial.println(" %");
  Serial.print("Pressure: ");
  Serial.print(pressure, 1);
  Serial.println(" hPa");
  Serial.print("Altitude: ");
  Serial.print(altitude, 2);
  Serial.println(" meters");
  Serial.print("Light Level: ");
  Serial.print(lightLevel, 1);
  Serial.println(" lux");

  // Print magnetometer values
  Serial.print("Magnetometer X: ");
  Serial.println(magX);
  Serial.print("Magnetometer Y: ");
  Serial.println(magY);
  Serial.print("Magnetometer Z: ");
  Serial.println(magZ);
  Serial.print("Heading: ");
  Serial.print(heading);
  Serial.println(" degrees");
}

void updateDisplay()
{
  try
  {
    // Clear the display
    display.clearDisplay();

    // Draw header with border
    display.fillRect(0, 0, 128, 12, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK, SSD1306_WHITE);
    display.setCursor(6, 2);
    display.print("WEATHER STATION");

    // Switch to white text on black background
    display.setTextColor(SSD1306_WHITE);

    // Draw separator lines
    display.drawFastHLine(0, 14, 128, SSD1306_WHITE);
    display.drawFastHLine(0, 27, 128, SSD1306_WHITE); // Separator between first and second row
    display.drawFastHLine(0, 40, 128, SSD1306_WHITE); // Separator between second and third row
    display.drawRect(0, 0, 128, 64, SSD1306_WHITE);   // Border around the display

    // Row 1: Temperature and Humidity
    display.setCursor(5, 18);
    display.print("T:");
    display.setCursor(70, 18);
    display.print("H:");

    // Row 2: Pressure and Altitude
    display.setCursor(5, 31);
    display.print("P:");
    display.setCursor(70, 31);
    display.print("A:");

    // Row 3: Light level and compass direction
    display.setCursor(5, 44);
    display.print("L:");

    display.setCursor(70, 44);
    display.print("C:");

    // Format the text for display
    // Temperature and humidity
    display.setCursor(20, 18);
    display.print(temperature, 1);
    display.print("C");

    display.setCursor(85, 18);
    display.print(humidity, 1);
    display.print("%");

    // Pressure and altitude
    display.setCursor(20, 31);
    display.print(int(pressure));
    display.print("hPa");

    display.setCursor(85, 31);
    display.print(int(altitude));
    display.print("m");

    // Light level
    display.setCursor(20, 44);
    display.print(lightLevel, 1);
    display.print("lux");

    // Compass heading
    display.setCursor(85, 44);
    display.print(int(heading));
    display.print("°");

    // Draw scale markers
    display.setCursor(5, 54);
    display.print("0");
    display.setCursor(113, 54);
    display.print("1K");

    // Draw empty bar outline
    display.drawRect(15, 54, 95, 6, SSD1306_WHITE);

    // Fill bar based on light level (max 1000 lux for full bar)
    int barWidth = min(int((lightLevel / 1000) * 93), 93); // Ensure it fits inside outline
    if (barWidth > 0)
    { // Only draw if there's something to show
      display.fillRect(16, 55, barWidth, 4, SSD1306_WHITE);
    }

    // Show the display
    display.display();
  }
  catch (...)
  {
    Serial.println("Error updating display");
  }
}