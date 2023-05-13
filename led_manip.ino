#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoBLE.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include "Adafruit_NeoPixel.h"
#include "Ultrasonic.h"

#ifdef __AVR__
  #include <avr/power.h>
#endif

// Globals
#define PIXEL_LED 33

#define NUMPIXELS 10

#define SIG 32

#define BUTTON_PIN 15


#define STATE_UPDATE_INTERVAL 1 * 1000

// Enum
enum LEDSTATE {
  off,
  reds,
  greens,
  blues
};

int ledState = 0;
bool isDistanceControlled = false;

// Wi-Fi and MQTT settings
char ssid[64];
char password[64];
char mqttServer[128];
const int mqttPort = 8883;
char mqttUsername[64];
char mqttPassword[64];

const char* rootCACert = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";


// states
int red = 100, green = 123, blue = 96;
int intensity = 25;
bool isInitialParameters = true;
int currentButtonState;


// LED pixels
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIXEL_LED, NEO_GRB + NEO_KHZ800);

// BLE settings
BLEService ledService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic ledStateCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite | BLENotify, 20);
BLEStringCharacteristic updatePreferencesCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite | BLENotify, 20);

// Attributes
WiFiClientSecure espClient;
PubSubClient client(espClient);
Ultrasonic ultrasonic(SIG);
Preferences preferences;

unsigned long lastMillis = 0;

void setup() {
  Serial.begin(115200);
  savePreferences();
  readPreferences();
  setupLED();
  setupBLE();
  setupWiFi();
  setupMQTT();
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();
  readDistanceAndChangeState();
  handleBLEConnections();
  handleButton();
  // interval update led state
  unsigned long currentMillis = millis();
  if(currentMillis - lastMillis >= STATE_UPDATE_INTERVAL) {
    lastMillis = currentMillis;
    sendLedStateJson();
  }
}

void setupButton() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void setupLED() {
  pixels.begin();
  setLedColorBrihtness();
}

void setupWiFi() {
  delay(10);
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Wi-Fi connected");
}

void setupMQTT() {
  espClient.setCACert(rootCACert);
  client.setServer(mqttServer, mqttPort);
  client.setCallback(mqttCallback);
}

void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("aybrlpk", mqttUsername, mqttPassword)) {
      Serial.println("connected");
      client.subscribe("change_led_state");
      client.subscribe("change_led_brightness");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}


void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (String(topic) == "change_led_state") {
    ledState = (char)payload[0] - '0';
    ledStateCharacteristic.writeValue(ledState);

    DynamicJsonDocument receivedDoc(1024);
    deserializeJson(receivedDoc, payload, length);

    intensity = receivedDoc["brightness"];
    red = receivedDoc["color"]["red"];
    green = receivedDoc["color"]["green"];
    blue = receivedDoc["color"]["blue"];
    setLedColorBrihtness();

    Serial.println("Parsed JSON data:");
    Serial.print("Brightness: ");
    Serial.println(intensity);
    Serial.print("Red: ");
    Serial.println(red);
    Serial.print("Green: ");
    Serial.println(green);
    Serial.print("Blue: ");
    Serial.println(blue);

  }

  if (String(topic) == "change_led_brightness") {
    ledState = (char)payload[0] - '0';
    ledStateCharacteristic.writeValue(ledState);

    DynamicJsonDocument receivedDoc(1024);
    deserializeJson(receivedDoc, payload, length);
    intensity = receivedDoc["brightness"];
    setLedColorBrihtness();

    Serial.println("Parsed JSON data:");
    Serial.print("Brightness: ");
    Serial.println(intensity);

  }
}

void setupBLE() {
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("ESP32_RGB_LED");
  BLE.setAdvertisedService(ledService);

  ledService.addCharacteristic(ledStateCharacteristic);
  ledService.addCharacteristic(updatePreferencesCharacteristic);
  
  BLE.addService(ledService);


  String ledStateString = String(red) + "," + String(green) + "," + String(blue);
  ledStateCharacteristic.writeValue(ledStateString.c_str());

  String preferencesCharacteristic = String(ssid) + "," + String(password) + "," + String(mqttServer) + String(mqttUsername) + "," + String(mqttPassword);
  updatePreferencesCharacteristic.writeValue(preferencesCharacteristic.c_str());

  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void readPreferences() {
  preferences.begin("creds", true); 

  strcpy(ssid, preferences.getString("wifiSsid", "").c_str());
  strcpy(password, preferences.getString("wifiPassword", "").c_str());
  strcpy(mqttServer, preferences.getString("mqttServer", "").c_str());
  strcpy(mqttUsername, preferences.getString("mqttUsername", "").c_str());
  strcpy(mqttPassword, preferences.getString("mqttPassword", "").c_str());

  preferences.end();
}

void savePreferences() {
  preferences.begin("creds", false); 
  preferences.putString("wifiSsid", "");
  preferences.putString("wifiPassword", "");
  preferences.putString("mqttServer", "");
  preferences.putString("mqttUsername", "");
  preferences.putString("mqttPassword", "");
  preferences.end();

  Serial.println("Preferences saved.");
}

void handleButton() {
  int buttonState = digitalRead(BUTTON_PIN);
  if(buttonState != currentButtonState) {
    isInitialParameters = !isInitialParameters;
    if(isInitialParameters) Serial.println("Mode parametres initial activé");
    else Serial.println("Mode parametres initial désactivé");
    currentButtonState = buttonState;
  }
  delay(200);
}

void handleBLEConnections() {
  BLEDevice central = BLE.central();

  if (central) {
    if (central.connected()) {
      if (ledStateCharacteristic.written()) {
        const uint8_t *ledStateData = ledStateCharacteristic.value();
        size_t dataLength = ledStateCharacteristic.valueLength();
        String ledStateString = "";

        for (size_t i = 0; i < dataLength; i++) {
          ledStateString += static_cast<char>(ledStateData[i]);
        }

        int separatorIndex1 = ledStateString.indexOf(',');
        int separatorIndex2 = ledStateString.lastIndexOf(',');

        if (separatorIndex1 > 0 && separatorIndex2 > 0 && separatorIndex1 != separatorIndex2) {
          red = ledStateString.substring(0, separatorIndex1).toInt();
          green = ledStateString.substring(separatorIndex1 + 1, separatorIndex2).toInt();
          blue = ledStateString.substring(separatorIndex2 + 1).toInt();
          intensity = 50; 
          setLedColorBrihtness();
        }
      }

      if (updatePreferencesCharacteristic.written() && isInitialParameters) {
        String newPreferences = updatePreferencesCharacteristic.value();
        int separatorIndex1 = newPreferences.indexOf(',');
        int separatorIndex2 = newPreferences.indexOf(',', separatorIndex1 + 1);
        int separatorIndex3 = newPreferences.indexOf(',', separatorIndex2 + 1);
        int separatorIndex4 = newPreferences.indexOf(',', separatorIndex3 + 1);

        if (separatorIndex1 > 0) {

          String newWifiSsid = newPreferences.substring(0, separatorIndex1);
          String newWifiPassword = newPreferences.substring(separatorIndex1 + 1, separatorIndex2);
          String newMqttUsername = newPreferences.substring(separatorIndex2 + 1, separatorIndex3);
          String newMqttPassword = newPreferences.substring(separatorIndex3 + 1, separatorIndex4);
          String newMqttServer = newPreferences.substring(separatorIndex4 + 1);

          preferences.begin("creds", false); // Begin preferences in read-write mode
          preferences.putString("wifiSsid", newWifiSsid);
          preferences.putString("wifiPassword", newWifiPassword);
          preferences.putString("mqttServer", mqttServer);
          preferences.putString("mqttUsername", newMqttUsername);
          preferences.putString("mqttPassword", newMqttPassword);
          preferences.end(); // End preferences

          Serial.println("Preferences updated from BLE.");
        }
      }
    }

    else {
      Serial.print("Disconnected from central: ");
      Serial.println(central.address());
    }
  }
}


void changeLEDState() {
  
  red = 0;
  green = 0;
  blue = 0;

  switch (ledState) {
    case 0 : 
      red = 255;
      break;
    case 1 : 
      green = 255;
      break;
    case 2 :
      blue = 255;
      break;
    case 3 : 
      intensity = 0;
      break;
  }
  setLedColorBrihtness();
  
}

void readDistanceAndChangeState() {
  int centimeters = ultrasonic.read();

  isDistanceControlled = centimeters <= 40;
  
  if(isDistanceControlled) {
    if (centimeters >= 0 && centimeters < 10) ledState = 0;
    if (centimeters >= 10 && centimeters < 20) ledState = 1;
    if (centimeters >= 20 && centimeters < 30) ledState = 2;
    if (centimeters >= 30) ledState = 3;
    
    changeLEDState();
  }
}

void setLedColorBrihtness() {
  pixels.clear();
  pixels.setBrightness(intensity);
  pixels.setPixelColor(0, pixels.Color(red,green,blue));
  pixels.setPixelColor(1, pixels.Color(red,green,blue));
  pixels.setPixelColor(2, pixels.Color(red,green,blue));
  pixels.setPixelColor(3, pixels.Color(red,green,blue));
  pixels.setPixelColor(4, pixels.Color(red,green,blue));
  pixels.setPixelColor(5, pixels.Color(red,green,blue));
  pixels.setPixelColor(6, pixels.Color(red,green,blue));
  pixels.setPixelColor(7, pixels.Color(red,green,blue));
  pixels.setPixelColor(8, pixels.Color(red,green,blue));
  pixels.setPixelColor(9, pixels.Color(red,green,blue));
  pixels.show();
  notifyBLE();
}

void notifyBLE() {
  String ledStateString = String(red) + "," + String(green) + "," + String(blue);
  ledStateCharacteristic.writeValue(ledStateString.c_str());
}

void sendLedStateJson() {
  DynamicJsonDocument sendDoc(1024);

  sendDoc["brightness"] = intensity;
  sendDoc["red"] = red;
  sendDoc["green"] = green;
  sendDoc["blue"] = blue;

  char buffer[512];
  serializeJson(sendDoc, buffer);

  if (client.publish("/led_state", buffer)) {
  } else {
    Serial.println("Failed to send JSON data to MQTT server");
  }
}