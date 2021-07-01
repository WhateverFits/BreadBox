#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266httpUpdate.h>
#include <PubSubClient.h>
#include <Time.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_AM2320.h"

#define FREQUENCY            10 

Adafruit_AM2320 am2320 = Adafruit_AM2320();

#define DNSNAME "breadbox"
#define MQTT_SERVER "pi4"
#define MQTT_PORT 1883
#define MQTT_CHANNEL_TEMP "home/" DNSNAME "/temp"
#define MQTT_CHANNEL_HUMID "home/" DNSNAME "/humidity"
#define MQTT_USER "user"
#define MQTT_PASSWORD "pass"
#define UPDATE_URL "http://pi4/cgi-bin/test.rb"

const char* ssids[] = {"WiFi", "Info"};
const char* passs[] = {"Goes", "Here"};
const int wifiCount = 2;
const char* server    = "api.thingspeak.com";
int peltierDutyLength = 1000;
float tempVariance = 1.0;
float powerMultiplier = 0.2;

long lastTime = 0;
long lastTimeClock = 0;
long lastReconnectAttempt = 0; 
String mqttClientId;
float temp;
float humid;
bool peltierOn = false;
bool connectedOnce = false;


ESP8266WiFiMulti wifiMulti;
WiFiClient mqttWiFiClient;

void mqttCallback(char* topic, byte* payload, unsigned int length);

void ConnectAP(void);

PubSubClient mqttClient(MQTT_SERVER, MQTT_PORT, mqttCallback, mqttWiFiClient);

String generateMqttClientId() {
  char buffer[4];
  uint8_t macAddr[6];
  WiFi.macAddress(macAddr);
  sprintf(buffer, "%02x%02x", macAddr[4], macAddr[5]);
  return DNSNAME + String(buffer);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.printf("Inside mqtt callback: %s\n", topic);
  Serial.println(length);

  String topicString = (char*)topic;
  topicString = topicString.substring(topicString.lastIndexOf('/')+1);
  Serial.print("Topic: ");
  Serial.print(topicString);

  String action = (char*)payload;
  action = action.substring(0, length);
  Serial.println(action);

  if (action == "Update") {
	  WiFiClient updateWiFiClient;
	  t_httpUpdate_return ret = ESPhttpUpdate.update(updateWiFiClient, UPDATE_URL);
	  switch (ret) {
		  case HTTP_UPDATE_FAILED:
			  Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
			  break;

		  case HTTP_UPDATE_NO_UPDATES:
			  Serial.println("HTTP_UPDATE_NO_UPDATES");
			  break;

		  case HTTP_UPDATE_OK:
			  Serial.println("HTTP_UPDATE_OK");
			  break;
	  }
  }
}

void mqttPublishTemp(float temperature) {
  char buf[256];
	if (mqttClient.connected()) {
    String(temperature).toCharArray(buf, 256);
		mqttClient.publish(MQTT_CHANNEL_TEMP, buf, true);
	}
}


void mqttPublishHumidity(float humidity) {
  char buf[256];
	if (mqttClient.connected()) {
    String(humidity).toCharArray(buf, 256);
		mqttClient.publish(MQTT_CHANNEL_HUMID, buf, true);
	}
}

boolean mqttReconnect() {
  char buf[100];
  mqttClientId.toCharArray(buf, 100);
  if (mqttClient.connect(buf, MQTT_USER, MQTT_PASSWORD)) {
  }

  return mqttClient.connected();
}

bool validateWiFi(long milliseconds) {
  //return connectedOnce;
  // Update WiFi status. Take care of rollover
  if (milliseconds >= lastTimeClock + 1000 || milliseconds < lastTimeClock) {
    if (wifiMulti.run() != WL_CONNECTED) {
      Serial.println("Disconnected");
      connectedOnce = false;
    } else {
      if (!connectedOnce) {
        Serial.print("Connected late to ");
        Serial.println(WiFi.SSID());
        onConnect();
      }

      connectedOnce = true;
    }
  }

  return connectedOnce;
}

void onConnect() {
  Serial.print("Connected: ");
  Serial.println(WiFi.localIP());
  connectedOnce = true;
}

void validateMqtt(long milliseconds) {
  if (!mqttClient.connected()) {
    if (milliseconds - lastReconnectAttempt > 5000 || lastReconnectAttempt == 0 || milliseconds < lastReconnectAttempt) {
      Serial.println("MQTT not connected");
      lastReconnectAttempt = milliseconds;
      Serial.println("MQTT reconnecting");
      // Attempt to reconnect
      if (mqttReconnect()) {
        Serial.println("MQTT reconnected");
      }
    }

    if (milliseconds - lastReconnectAttempt > 60000) {
      Serial.println("MQTT disconnecting WiFi");
      WiFi.disconnect();
      delay(500);
    }
  } else {
    mqttClient.loop();
  }
}


void ConnectAP(void) {
  WiFi.forceSleepWake();
  delay( 1 );
  WiFi.persistent( false );
  WiFi.mode(WIFI_STA);    /* Set WiFi to station mode */
#ifdef DNSNAME
  WiFi.hostname(DNSNAME);
#else
  char buffer[4];
  uint8_t macAddr[6];
  WiFi.macAddress(macAddr);
  sprintf(buffer, "%02x%02x", macAddr[4], macAddr[5]);
  WiFi.hostname("BreadBox" + String(buffer));
#endif

  delay(100);
  Serial.print("Connecting Wifi: ");
  for (int i=0; i < wifiCount; i++) {
    wifiMulti.addAP(ssids[i], passs[i]);
  }
  Serial.println("Connecting");
  wifiMulti.run();
  Serial.println("");
  Serial.println("WiFi connecting");
  Serial.println("IP address: ");
  IPAddress ip = WiFi.localIP();
  Serial.println(ip);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // hang out until serial port opens
  }

  long milliseconds = millis();

  mqttClientId = generateMqttClientId();

  Serial.println("Temperature and humidity monitoring");
  am2320.begin();
  ConnectAP();
}

// Get the temperature and humidity
bool getTempHumid(long milliseconds) {
	if (milliseconds >= lastTime + 60000 || milliseconds < lastTime || lastTime == 0 ) {
		temp = am2320.readTemperature();
		humid = am2320.readHumidity();
		Serial.print("Temp: "); Serial.println(temp);
		Serial.print("Hum: "); Serial.println(humid);

		if (isnan(humid) || isnan(temp)) {
			Serial.println("Failed to read from DHT sensor!");
			return false;
		}

		return true;
	}
}

// Publish the temperature and humidity for monitoring
void logTempHumid() {
  if (mqttReconnect()) {
    mqttClient.loop();
    mqttPublishTemp(temp);
  }

  if (mqttReconnect()) {
    mqttClient.loop();
    mqttPublishHumidity(humid);
  }
}

bool cyclePeltier(long milliseconds, float targetTemp, float currentTemp) {
	static long lastTimePeltier;

	// Power value is zero if within tempVariance of measured temp otherwise 
	// power is a multiple of the temperature distance from the target.
	long power;
	if (abs(targetTemp - currentTemp) > tempVariance) {
		power = abs(targetTemp - currentTemp) * powerMultiplier;
	} else {
		power = 0;
	}

  // If we have a power value then we want to attempt to turn it on
	if (power > 0) {
		bool heating = targetTemp > currentTemp;

		// TODO: Set relay for heating/cooling current direction

		int powerDelay = peltierDutyLength * power;
		if (!peltierOn) {
			if (milliseconds - (peltierDutyLength - powerDelay) > lastTimePeltier || milliseconds < lastTimePeltier) {
				peltierOn = true;
				lastTimePeltier = milliseconds;
				// TODO: Turn on peltier mosfet
				Serial.print("Peltier turned on to ");
				if (heating) Serial.println("heat"); else Serial.println("cool");
			}
		} else {
			if (milliseconds - powerDelay > lastTimePeltier || milliseconds < lastTimePeltier) {
				peltierOn = false;
				lastTimePeltier = milliseconds;
				// TODO: Turn off peltier mosfet
				Serial.println("Peltier turned off to ");
				if (heating) Serial.println("heat"); else Serial.println("cool");
			}
		}
	}
}

void loop() {
  long milliseconds = millis();

	cyclePeltier(milliseconds, 22.2, temp);

  // Check if connected then handle the connected magic
  if (!peltierOn && validateWiFi(milliseconds)) {
    validateMqtt(milliseconds);

		// Check the time. Set alarms. Take care of rollover
		if (getTempHumid(milliseconds))
			logTempHumid();
	}
}
