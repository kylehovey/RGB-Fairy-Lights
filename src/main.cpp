#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define LED_PIN A0
#define LED_COUNT 100

const char* ssid = "CHANGEME";
const char* password = "CHANGEME";

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

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGB + NEO_KHZ800);

// Adapted from https://stackoverflow.com/a/26233318
int getHue(int red, int green, int blue) {
    float minVal = float(min(min(red, green), blue));
    float maxVal = float(max(max(red, green), blue));
    float hueVal = 0.0;

    if (minVal == maxVal) {
        return 0;
    }

    if (maxVal == red) {
        hueVal = (float(green) - float(blue)) / (maxVal - minVal);
    } else if (maxVal == green) {
        hueVal = 2.0 + (float(blue) - float(red)) / (maxVal - minVal);
    } else {
        hueVal = 4.0 + (float(red) - float(green)) / (maxVal - minVal);
    }

    hueVal = hueVal * 60.0;
    if (hueVal < 0) hueVal = hueVal + 360.0;

    return round(hueVal);
}

int r = 100;
int g = 0;
int b = 100;
int hue = getHue(r, g, b);
int brightness = 255;
String power = "on";
String effect = "fireflies";

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

float fireflyActivation(float t, float phase = 0) {
  float E = (sin(t + phase) + cos(5.0*t + phase) - cos(10.0*t + phase) + sin(25.0*t + phase)) / 35.0;
  float x = 20.0 * sin(0.5 * t + phase);
  float fire = 2.0 / (1.0 + exp(x * x));
  float intensity = fire + (1 - fire) * E;

  return intensity;
}

uint32_t fireflyHue(float intensity) {
  float hue = 63.0 * intensity + 262.0 * (1.0 - intensity);
  float beegHue = beegHueFromSmol(hue);

  return uint32_t(beegHue);
}

float fireflyBrightness(float intensity) {
  return float(brightness) / (2.0 * (1.0 + exp(-8.0 * (intensity - 0.5)))) + 0.5;
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

void publishBrightness() {
  int percentage = round(100.0 * float(brightness) / 255.0);

  client.publish(mqtt_brightness_status_topic.c_str(), String(percentage).c_str());
}

void publishRgb() {
  client.publish(mqtt_rgb_status_topic.c_str(), (String(r) + "," + String(g) + "," + String(b)).c_str());
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

    brightness = round(255.0 * (String(parsed).toFloat() / 100.0));
    publishBrightness();
  } else if (topic == mqtt_rgb_command_topic) {
    uint8_t firstIndex = parsed.indexOf(',');
    uint8_t lastIndex = parsed.lastIndexOf(',');

    r = parsed.substring(0, firstIndex).toInt();
    g = parsed.substring(firstIndex + 1, lastIndex).toInt();
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
  publishRgb();
  publishEffect();
  publishBrightness();

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


  long t = millis();
  strip.setBrightness(brightness);

  if (effect == "static") {
    strip.fill(strip.Color(r, g, b));
  } else if (effect == "rainbow") {
    long iterations = t % 128000;
    strip.rainbow(round(256.0 * iterations / 100.0));
  } else if (effect == "fireflies") {
    float pTime = (float)t / 2000.0;

    for (int i = 0; i < LED_COUNT; ++i) {
      float intensity = fireflyActivation(pTime, randomPhases[i]);
      float hue = fireflyHue(intensity);
      float fBrightness = fireflyBrightness(intensity);
      strip.setPixelColor(i, strip.ColorHSV(hue, 255, round(fBrightness)));
    }
  } else if (effect == "aurora") {
    float pTime = (float)t / 4000.0;

    for (int i = 0; i < LED_COUNT; ++i) {
      float hue = auroraHue(pTime, randomPhases[i], 150.0, 220.0);
      strip.setPixelColor(i, strip.ColorHSV(hue, 255, brightness));
    }
  } else if (effect == "auroraSettable") {
    float pTime = (float)t / 4000.0;

    for (int i = 0; i < LED_COUNT; ++i) {
      int deltaHue = 70;
      int startingHue = hue;
      int endingHue = hue + deltaHue % 360;
      float hue = auroraHue(pTime, randomPhases[i], startingHue, endingHue);
      strip.setPixelColor(i, strip.ColorHSV(hue, 255, brightness));
    }
  } else if (effect == "torchlight") {
    float pTime = (float)t / 3000.0;

    for (int i = 0; i < LED_COUNT; ++i) {
      float hue = auroraHue(pTime, randomPhases[i], 18.0, 39.0);
      strip.setPixelColor(i, strip.ColorHSV(hue, 255, brightness));
    }
  }

  strip.show();
}

void loop() {
  client.loop();

  effectLoop();
}
