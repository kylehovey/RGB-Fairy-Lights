#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define LED_PIN A0
#define LED_COUNT 100

const char* ssid = "***REMOVED***";
const char* password = "***REMOVED***";

const char* mqtt_server = "192.168.100.42";
const char* mqtt_discovery_topic = "homeassistant/sensor/esp32temperaturespy/config"; const char* mqtt_state_topic = "home/sensor/esp32temperaturespy/value";

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
    Serial.print("Attempting MQTT connection...");
    String clientId = "MQTT Fairy";

    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      return;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");

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
    delay(wait);
  }
}

void init_strip() {
  strip.begin();
  strip.show();
  strip.setBrightness(50);
}

void setup() {
  Serial.begin(112500);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }

  Serial.print("Connecting to WiFi ..");

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }

  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1883);

  if (!client.connected()) {
    reconnect();
  }

  init_strip();
}

void loop() {
  rainbow(100);
}
