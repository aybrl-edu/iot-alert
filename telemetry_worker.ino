#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid     = "";
const char* password = "";

// MQTT client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient); 
char *mqttServer = "broker.hivemq.com";
int mqttPort = 1883;
String LED_STATE = "LOW";
unsigned long lastMsg = 0;

void setup()
{
    Serial.begin(115200);
    delay(10);
    // Config LED
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Connect to a WiFi network
    connectWifi();

    // Setup MQTT Server
    setupMQTT();
}

void connectWifi() {
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

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Callback - ");
  Serial.print("Message:");
  String messageTemp;
  for (int i = 0; i < length; i++) {
    Serial.print((char) payload[i]);
    messageTemp += (char) payload[i];
  }
  // EXO2 Get command LED from MQTT 
  if (String(topic) == "/aybrl/led_telemetrie/switch") {
    Serial.print("Changing LED to ");
    if(messageTemp == "on"){
      Serial.println("on");
      turnLedOn();
    }
    else if(messageTemp == "off"){
      Serial.println("off");
      turnLedOff();
    }
  }
}

void setupMQTT() {
  mqttClient.setServer(mqttServer, mqttPort);
  // set the callback function
  mqttClient.setCallback(callback);
}

void publishLEDState() {
    char data[1];
    sprintf(data, "%d", 0);
    if(getLEDState() == "HIGH") sprintf(data, "%d", 1);
    mqttClient.publish("/aybrl/led_telemetrie", data);
}

// Reconnect TO MQTT Server
void reconnect() {
  Serial.println("Connecting to MQTT Broker...");
  while (!mqttClient.connected()) {
      Serial.println("Reconnecting to MQTT Broker..");
      String clientId = "clientId-";
      clientId += String(random(0xffff), HEX);
      
      if (mqttClient.connect(clientId.c_str())) {
        Serial.println("Connected.");
        // subscribe to topic to publish led state
        mqttClient.subscribe("/aybrl/led_telemetrie");
        // subscribe to topic to get commands
        mqttClient.subscribe("/aybrl/led_telemetrie/switch");
      }
      
  }
}

String getLEDState() {
    return LED_STATE;
}

void turnLedOn() {
    digitalWrite(LED_BUILTIN, HIGH);
    LED_STATE = "HIGH";
}

void turnLedOff() {
    digitalWrite(LED_BUILTIN, LOW);
    LED_STATE = "LOW";
}

void loop()
{
    
    if (!mqttClient.connected())
       reconnect();
    mqttClient.loop();

    // EXO1 Publish LED Télémetrie -> MQTT 
    //if(getLEDState() == "HIGH") turnLedOff();
    //else turnLedOn();

    unsigned long now = millis();
    if (now - lastMsg > 2000) {
      lastMsg = now;
      publishLEDState();
    }
    
}

