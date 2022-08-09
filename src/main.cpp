#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define LED_PIN A0
#define LED_COUNT 100

const char* ssid = "***REMOVED***";
const char* password = "***REMOVED***";

const char* mqtt_server = "192.168.100.42";

const String mqtt_power_command_topic = "home/light/qt_py_fairy/power/set";
const String mqtt_rgb_command_topic = "home/light/qt_py_fairy/rgb/set";
const String mqtt_brightness_command_topic = "home/light/qt_py_fairy/brightness/set";
const String mqtt_effect_command_topic = "home/light/qt_py_fairy/effect/set";

const String mqtt_power_status_topic = "home/light/qt_py_fairy/power/status";
const String mqtt_rgb_status_topic = "home/light/qt_py_fairy/rgb/status";
const String mqtt_brightness_status_topic = "home/light/qt_py_fairy/brightness/status";
const String mqtt_effect_status_topic = "home/light/qt_py_fairy/effect/status";

WiFiClient espClient;
PubSubClient client(espClient);

IPAddress local_IP(192, 168, 100, 74);
IPAddress gateway(192, 168, 100, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(1, 1, 1, 1);
IPAddress secondaryDNS(1, 0, 0, 1);

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

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

// Generated with:
// Array(100).fill().map(() => Math.random()).map(x => x * Math.PI * 2).map(x => x.toFixed(2)).join(',')
float randomPhases[] = {5.04,5.81,4.95,6.19,2.88,3.42,5.60,2.07,5.27,5.89,0.41,3.66,1.61,4.67,6.05,2.91,3.42,0.36,4.71,5.04,4.26,6.15,1.68,1.53,2.43,6.14,4.38,1.80,0.17,5.15,5.11,1.67,5.03,0.31,5.95,1.95,4.91,1.62,3.57,5.43,4.28,2.52,2.22,2.52,5.18,1.20,0.16,0.99,4.23,5.59,1.39,3.46,1.67,5.61,1.12,5.16,1.79,5.49,1.48,0.84,4.61,0.04,2.81,3.15,3.70,2.97,5.27,3.13,1.98,5.28,2.20,0.65,5.42,5.00,1.45,0.91,2.87,4.02,5.46,4.29,5.61,5.87,0.57,0.57,4.80,1.80,6.05,3.96,1.53,0.81,1.73,6.21,2.25,0.95,0.89,3.15,4.28,4.22,4.76,4.43};

float beegHueFromSmol(float hue) {
  return hue * 65535.0 / 360.0;
}

uint32_t fireflyHue(float t, float phase = 0) {
  float E = (sin(t + phase) + cos(5.0*t + phase) - cos(10.0*t + phase) + sin(25.0*t + phase)) / 35.0;
  float x = 20.0 * sin(0.5 * t + phase);
  float fire = 2.0 / (1.0 + exp(x * x));
  float intensity = fire + (1 - fire) * E;
  float hue = 130.0 * intensity + 280.0 * (1.0 - intensity);
  float beegHue = beegHueFromSmol(hue);

  return uint32_t(beegHue);
}

uint32_t auroraHue(float t, float phase, float beginningHue, float endingHue) {
  float intensity = (sin(t + phase) + cos(5.0*t + phase) - cos(10.0*t + phase) + sin(25.0*t + phase)) / 7.05;
  float hue = endingHue * intensity + beginningHue * (1.0 - intensity);
  float beegHue = beegHueFromSmol(hue);

  return uint32_t(beegHue);
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
String power = "on";
String effect = "static";

void publishBrightness() {
  client.publish(mqtt_brightness_status_topic.c_str(), String(brightness).c_str());
}

void publishRgb() {
  client.publish(mqtt_rgb_status_topic.c_str(), (String(g) + "," + String(r) + "," + String(b)).c_str());
}

void publishPower() {
  client.publish(mqtt_power_status_topic.c_str(), power.c_str());
}

void publishEffect() {
  client.publish(mqtt_effect_status_topic.c_str(), effect.c_str());
}

void onCommand(char* rawTopic, byte* payload, unsigned int length) {
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

    g = parsed.substring(0, firstIndex).toInt();
    r = parsed.substring(firstIndex + 1, lastIndex).toInt();
    b = parsed.substring(lastIndex + 1).toInt();

    publishRgb();
  } else if (topic == mqtt_effect_command_topic) {
    effect = parsed;

    publishEffect();
  }

  Serial.println(topic);
  Serial.println(parsed);
}

void init_mqtt() {
  client.subscribe(mqtt_power_command_topic.c_str());
  client.subscribe(mqtt_brightness_command_topic.c_str());
  client.subscribe(mqtt_rgb_command_topic.c_str());
  client.subscribe(mqtt_effect_command_topic.c_str());

  publishPower();
  publishBrightness();
  publishRgb();
  publishEffect();

  Serial.println("Setting up MQTT callback");
  client.setCallback(onCommand);
}

void setup() {
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

  strip.begin();

  init_strip();
  init_mqtt();
}

void effectLoop() {
  if (power == "OFF") {
    strip.fill(strip.Color(0, 0, 0));
    strip.show();

    return;
  };

  strip.setBrightness(brightness);

  long t = millis();

  if (effect == "static") {
    strip.fill(strip.Color(r, g, b));
    strip.setBrightness(brightness);
  } else if (effect == "rainbow") {
    long iterations = t % 128000;
    strip.rainbow(round(256.0 * iterations / 100.0));
  } else if (effect == "fireflies") {
    float pTime = (float)t / 2000.0;

    for (int i = 0; i < LED_COUNT; ++i) {
      float hue = fireflyHue(pTime, randomPhases[i]);
      strip.setPixelColor(i, strip.ColorHSV(hue, 255, brightness));
    }
  } else if (effect == "aurora") {
    float pTime = (float)t / 4000.0;

    for (int i = 0; i < LED_COUNT; ++i) {
      float hue = auroraHue(pTime, randomPhases[i], 282.0, 350.0);
      strip.setPixelColor(i, strip.ColorHSV(hue, 255, brightness));
    }
  } else if (effect == "torchlight") {
    float pTime = (float)t / 1000.0;

    for (int i = 0; i < LED_COUNT; ++i) {
      float hue = auroraHue(pTime, randomPhases[i], 100.0, 130.0);
      strip.setPixelColor(i, strip.ColorHSV(hue, 255, brightness));
    }
  }

  strip.show();
}

void loop() {
  client.loop();

  effectLoop();
}
