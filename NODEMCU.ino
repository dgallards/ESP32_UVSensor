#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <DFRobot_AirQualitySensor.h>
#include <LTR390.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

#define I2C_SDA 14  // SDA Connected to GPIO 14
#define I2C_SCL 15  // SCL Connected to GPIO 15

Adafruit_BME280 BME;

LTR390 LTR(0x53);

DFRobot_AirQualitySensor PM25(&Wire, 0x19);

String tempString, UVString, PM25String, humidityString, pressureString;

//config variables----------------------------------------------------------

//these are the config variables for the wifi connection
const char* ssid = "Tenda_DIEGOX";
const char* password = "dieguitusN1";

//these are the config variables for the mqtt connection
const char* mqtt_server = "192.168.1.142";
const char* client_id = "Sensor_1";
const char* password_mqtt = "rSBsyGswREbf";

//these are the lat and long for the location of the sensor
const char* lat = "39.469846";
const char* lon = "-6.380116";

//ending of config variables -------------------------------------------------


WiFiClient espWifi;

PubSubClient client(espWifi);

long lastMsg = 0;
char msg[100];
int value = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  BMEInit();
  LTRInit();
  PM25Init();
  setup_Wifi();
  client.setServer(mqtt_server, 1883);
}

void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsg > 60000) {
    lastMsg = now;

    BMEprintValues();
    LTRprintValues();
    PM25printValues();

    //clear char msg array
    
   
      msg[0]='\0';
      strcat(msg, client_id);
      strcat(msg, ",");
      strcat(msg, password_mqtt);
      strcat(msg, ",");
      strcat(msg, lat);
      strcat(msg, ",");
      strcat(msg, lon);
      strcat(msg, ",");
      strcat(msg, UVString.c_str());
      strcat(msg, ",");
      strcat(msg, tempString.c_str());
      strcat(msg, ",");
      strcat(msg, humidityString.c_str());
      strcat(msg, ",");
      strcat(msg, pressureString.c_str());
      strcat(msg, ",");
      strcat(msg, PM25String.c_str());
      Serial.println(msg);
      client.publish("esp32/sensors", msg);
    
  }
}

void BMEInit() {
  Serial.println(F("BME280 test"));
  if (!BME.begin()) {
    Serial.println(F("BME280 ERROR"));
    delay(10);
  }
  Serial.println("Found BME sensor!");
}

void BMEprintValues() {
  auto temp = BME.readTemperature();
  auto pres = BME.readPressure() / 100.0F;
  auto alt = BME.readAltitude(1013.25);
  auto hum = BME.readHumidity();

  Serial.print("Temperature = ");
  Serial.print(temp);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(pres);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(alt);
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(hum);
  Serial.println(" %");

  Serial.println();

  tempString = String(temp);
  humidityString = String(hum);
  pressureString = String(pres);

}

void LTRInit() {
  Serial.println("LTR390 test");

  if (!LTR.init()) {
    Serial.println("LTR390 ERROR");
    delay(10);
  }
  Serial.println("Found LTR sensor!");
  LTR.setMode(LTR390_MODE_UVS);
}

void LTRprintValues() {
  if (LTR.newDataAvailable()) {
    Serial.print("UV Index: ");
    LTR.setGain(LTR390_GAIN_18);                  //Recommended for UVI - x18
    LTR.setResolution(LTR390_RESOLUTION_20BIT);
    LTR.setMode(LTR390_MODE_UVS);
    auto LTRVal = LTR.getUVI();
    Serial.println(LTRVal);


    Serial.print("UV index: ");
    Serial.println(LTRVal);

    UVString = String(LTRVal);
  }
}
void PM25Init() {
  Serial.println("PM25 test");
  if (!PM25.begin()) {
    Serial.println("PM25 ERROR");
    delay(10);
  }
  Serial.println("Found PM25 sensor!");
}

void PM25printValues() {
  uint16_t PM2_5 = PM25.gainParticleConcentration_ugm3(PARTICLE_PM2_5_STANDARD);
  uint16_t PM1_0 = PM25.gainParticleConcentration_ugm3(PARTICLE_PM1_0_STANDARD);
  uint16_t PM10 = PM25.gainParticleConcentration_ugm3(PARTICLE_PM10_STANDARD);
  Serial.print("PM2.5 concentration:");
  Serial.print(PM2_5);
  Serial.println(" ug/m3");
  Serial.print("PM1.0 concentration:");
  Serial.print(PM1_0);
  Serial.println(" ug/m3");
  Serial.print("PM10 concentration:");
  Serial.print(PM10);
  Serial.println(" ug/m3");
  Serial.println();

  PM25String = String(PM2_5);
}

void setup_Wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
