#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define LED_PIN A0
#define LED_COUNT 10

const char* ssid = "***REMOVED***";
const char* password = "***REMOVED***";

const char* mqtt_server = "192.168.100.42";

const String mqtt_power_command_topic = "home/light/qt_py_fairy/power/set";
const String mqtt_rgb_command_topic = "home/light/qt_py_fairy/rgb/set";
const String mqtt_brightness_command_topic = "home/light/qt_py_fairy/brightness/set";

const String mqtt_power_status_topic = "home/light/qt_py_fairy/power/status";
const String mqtt_rgb_status_topic = "home/light/qt_py_fairy/rgb/status";
const String mqtt_brightness_status_topic = "home/light/qt_py_fairy/brightness/status";

WiFiClient espClient;
PubSubClient client(espClient);

IPAddress local_IP(192, 168, 100, 74);
IPAddress gateway(192, 168, 100, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(1, 1, 1, 1);
IPAddress secondaryDNS(1, 0, 0, 1);

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels(1, PIN_NEOPIXEL);

void reconnect() {
  while (!client.connected()) {
    String clientId = "MQTT Fairy";

    if (client.connect(clientId.c_str())) {
      return;
    } else {
      delay(5000);
    }
  }
}

/**
 * Rainbow cycle the strip
 * @param wait Millisecond delay between frames
 */
void rainbow(int wait) {
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    strip.rainbow(firstPixelHue);
    strip.show();
    client.loop();
    delay(wait);
  }
}

void init_strip() {
  strip.begin();
  strip.show();
  strip.setBrightness(50);
}

int r = 100;
int g = 0;
int b = 100;
int brightness = 100;
String power = "ON";

void publishBrightness() {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.setBrightness(brightness);
  pixels.show();
  client.publish(mqtt_brightness_status_topic.c_str(), String(brightness).c_str());
}

void publishRgb() {
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
  client.publish(mqtt_rgb_status_topic.c_str(), (String(r) + "," + String(g) + "," + String(b)).c_str());
}

void publishPower() {
  if (power == "ON") {
    pixels.setPixelColor(0, pixels.Color(r, g, b));
  } else {
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  }

  pixels.show();
  client.publish(mqtt_power_status_topic.c_str(), power.c_str());
}

void on_command(char* rawTopic, byte* payload, unsigned int length) {
  String topic = String(rawTopic);
  String parsed = String((char*) payload).substring(0, length);

  if (topic == mqtt_power_command_topic) {
    Serial.print("Setting power to ");
    Serial.println(parsed);

    power = parsed;
    publishPower();
  } else if (topic == mqtt_brightness_command_topic) {
    Serial.print("Setting brightness to ");
    Serial.println(String(parsed).toInt());

    brightness = String(parsed).toInt();
    publishBrightness();
  } else if (topic == mqtt_rgb_command_topic) {
    uint8_t firstIndex = parsed.indexOf(',');
    uint8_t lastIndex = parsed.lastIndexOf(',');

    r = parsed.substring(0, firstIndex).toInt();
    g = parsed.substring(firstIndex + 1, lastIndex).toInt();
    b = parsed.substring(lastIndex + 1).toInt();

    publishRgb();
  }

  Serial.println(topic);
  Serial.println(parsed);
}

void init_mqtt() {
  client.subscribe(mqtt_power_command_topic.c_str());
  client.subscribe(mqtt_brightness_command_topic.c_str());
  client.subscribe(mqtt_rgb_command_topic.c_str());

  publishPower();
  publishBrightness();
  publishRgb();

  Serial.println("Setting up MQTT callback");
  client.setCallback(on_command);
}

void setup() {
#if defined(NEOPIXEL_POWER)
  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH);
#endif

  Serial.begin(112500);

  Serial.println("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }

  Serial.println(WiFi.localIP());

  Serial.println("Connecting to MQTT broker");
  client.setServer(mqtt_server, 1883);

  if (!client.connected()) {
    Serial.println("No connection yet, trying again");
    reconnect();
  }

  Serial.println(client.state());

  pixels.begin();

  init_strip();
  init_mqtt();
}

void loop() {
  client.loop();
  // rainbow(100);
}