#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define LED_PIN A0
#define LED_COUNT 10

const char* ssid = "***REMOVED***";
const char* password = "***REMOVED***";

const char* mqtt_server = "192.168.100.42";
// const char* mqtt_discovery_topic = "homeassistant/light/rgb_fairy_lights/bedroom_fairy_lights/config";
// const String mqtt_device_map = "{\"configuration_url\":\"http://192.168.100.74/\", \"name\":\"Rainbow Fairy Lights\"}";
//

const String mqtt_power_command_topic = "home/light/qt_py_fairy/power/set";
const String mqtt_rgb_command_topic = "home/light/qt_py_fairy/rgb/set";
const String mqtt_brightness_command_topic = "home/light/qt_py_fairy/brightness/set";

const String mqtt_power_status_topic = "home/light/qt_py_fairy/power/status";
const String mqtt_rgb_status_topic = "home/light/qt_py_fairy/rgb/status";
const String mqtt_brightness_status_topic = "home/light/qt_py_fairy/brightness/status";

// const String discovery_payload = "{\"device_map\":" + mqtt_device_map + ", \"rgb_state_topic\":\"" + mqtt_state_topic + "\", \"rgb_command_topic\":\"" + mqtt_command_topic + "\", \"supported_color_modes\":[\"rgb\"], \"color_mode\": true }";

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

void on_command(char* topic, byte* payload, unsigned int length) {
  String parsed = String((char*) payload).substring(0, length);
  Serial.println(topic);
  Serial.println(parsed);
}

void init_mqtt() {
  // Serial.println("Publishing discovery payload with topic/payload:");
  // Serial.println(mqtt_discovery_topic);
  // Serial.println(discovery_payload);
  //
  // success = client.publish(mqtt_discovery_topic, discovery_payload.c_str(), true);
  //
  // Serial.print("Success: ");
  // Serial.println(success);

  client.subscribe(mqtt_power_command_topic.c_str());
  client.subscribe(mqtt_brightness_command_topic.c_str());
  client.subscribe(mqtt_rgb_command_topic.c_str());

  client.publish(mqtt_power_status_topic.c_str(), "ON");
  client.publish(mqtt_brightness_status_topic.c_str(), "100");
  client.publish(mqtt_rgb_status_topic.c_str(), "100,0,100");

  Serial.println("Setting up MQTT callback");
  client.setCallback(on_command);
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

  init_strip();
  init_mqtt();
}

void loop() {
  client.loop();
  // rainbow(100);
}
