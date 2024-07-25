/* 
 * Project myProject
 * Author: Your Name
 * Date: 
 * For comprehensive documentation and examples, please visit:
 * https://docs.particle.io/firmware/best-practices/firmware-template/
 */

// Include Particle Device OS APIs
#include "Particle.h"
#include "Adafruit_BME280.h"
#include "Air_Quality_Sensor.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_MQTT.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "credentials.h"

// Let Device OS manage the connection to the Particle Cloud
SYSTEM_MODE(SEMI_AUTOMATIC);

/************ Global State (you don't need to change this!) ***   ***************/ 
TCPClient TheClient; 

// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

/****************************** Feeds ***************************************/ 
// Setup Feeds to publish or subscribe 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Subscribe buttonfeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/buttonfeed"); 
Adafruit_MQTT_Publish moisturefeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisturefeed");
Adafruit_MQTT_Publish tempFfeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/tempFfeed");
Adafruit_MQTT_Publish humidityfeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidityfeed");
Adafruit_MQTT_Publish concentrationfeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/concentrationfeed");
/************Declare Variables*************/
unsigned int last, lastTime;
float numValue;
bool subValue;

/************Declare Functions*************/
void MQTT_connect();
bool MQTT_ping();

AirQualitySensor sensor(A1);

Adafruit_BME280 bme ;
const int OLED_RESET=-1;
Adafruit_SSD1306 display(OLED_RESET);
int Last500MillisSec;
int currentTime;
int moisture;
const int air = 5248;
const int water = 1760;
const int moistSoil = 3370;
const int drySoil = 3520;
int last60kmillisSec;
bool togglePump;
int last150millisSec;
float tempC;
float pressPA;
float humidity;
float tempF;
float inHg;
const int hexAddress = 0x76;
bool status;
unsigned int lastConcentration;
unsigned int lastTempF;
unsigned int lastMoisture;
unsigned int lastHumidity;

const int pin = D8;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancy = 0;
float ratio = 0;
float concentration = 0;

// Run the application and system concurrently in separate threads
//SYSTEM_THREAD(ENABLED);

// setup() runs once, when the device is first turned on
void setup() {
Serial.begin();
waitFor(Serial.isConnected,10000);
Serial.printf("Ready To Go");
pinMode(A5,INPUT);
pinMode(S4,OUTPUT);
pinMode(pin,INPUT);

while (!Serial);
Serial.printf("Waiting sensor to init...");
delay(5000);

if (sensor.init()) {
        Serial.printf("Sensor ready.");
    } else {
        Serial.printf("Sensor ERROR!");
    }

WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
  Serial.printf("\n\n");

status = bme . begin (hexAddress);
  if(status == false) {
    Serial.printf("BME280 at address 0x%02X failed to start", hexAddress);
  }
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.display();
  delay(1000);
  display.clearDisplay();
  starttime = millis();
  mqtt.subscribe(&buttonfeed);
}

// loop() runs over and over again, as quickly as it can execute.
void loop() {
  MQTT_connect();
  MQTT_ping();

  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &buttonfeed) {
      subValue = atoi((char *)buttonfeed.lastread);
    }
    Serial.printf("button %i\n",subValue);
  }

  if(subValue){
      digitalWrite(S4,HIGH);
      Serial.printf("Manually Pump Water\n");
      delay(500);//remove delay later
      digitalWrite(S4,LOW);
  }

  if((millis()-lastMoisture > 61000)) {
    if(mqtt.Update()) {
      moisturefeed.publish(moisture);
      Serial.printf("Publishing moisture %i\n",moisture);
    }
    lastMoisture = millis();
  }

      if((millis()-lastTempF > 6000)) {
    if(mqtt.Update()) {
      tempFfeed.publish(tempF);
      Serial.printf("Publishing tempF %0.2f\n",tempF);
    }
    lastTempF = millis();
    
    }

      if((millis()-lastHumidity > 6000)) {
    if(mqtt.Update()) {
      humidityfeed.publish(humidity);
      Serial.printf("Publishing humidity %0.2f\n",humidity);
    }
    lastHumidity = millis();
    
    }
  
    if((millis()-lastConcentration > 31000)) {
    if(mqtt.Update()) {
      concentrationfeed.publish(concentration);
      Serial.printf("Publishing concentration %0.2f\n",concentration);
    }
    lastConcentration = millis();
    
    }

        if((moisture>drySoil)) { 
      digitalWrite(S4,HIGH);}
      delay(500);//remove delay later
      digitalWrite(S4,LOW);

  currentTime = millis ();
  if((currentTime-last60kmillisSec)>60000) {
    last60kmillisSec = millis();
  moisture=analogRead(A5);
   Serial.printf("moisture sensor %i\n",moisture);
  }
        currentTime = millis ();
  if((currentTime-last150millisSec)>650) {
    last150millisSec = millis();
  tempC = bme.readTemperature();
    tempF = 1.8 * tempC + 32;
  pressPA = bme.readPressure();
    inHg= 0.000295 * pressPA + 4.77;
  humidity = bme.readHumidity();
    Serial.printf("\ntempF %0.2f\ninHg %0.2f\nhumidity %0.2f\n",tempF ,inHg, humidity);
  }


duration = pulseIn(pin, LOW);
lowpulseoccupancy = lowpulseoccupancy+duration;

if ((millis()-starttime) > sampletime_ms) {
  ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
  concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
  if(lowpulseoccupancy != 0){
    Serial.print(lowpulseoccupancy);
    Serial.print(",");
    Serial.print(ratio);
    Serial.print(",");
    Serial.print(concentration);
    lowpulseoccupancy = 0;
    starttime = millis();
    Serial.printf("concentration%0.1f",concentration);
  }

}

int quality = sensor.slope();

    Serial.printf("Sensor value.%i\n",sensor.getValue());
   

    if (quality == AirQualitySensor::FORCE_SIGNAL) {
        Serial.printf("High pollution! Force signal active.\n");
        display.setCursor(0,32);
        display.display();
        display.printf("High pollution! Force signal active. %i",sensor.getValue());
        display.setCursor(0,32);
        display.display();
    } else if (quality == AirQualitySensor::HIGH_POLLUTION) {
        Serial.printf("High pollution!\n");
        display.setCursor(0,32);
        display.printf("High pollution! %i",sensor.getValue());
        display.display();
    } else if (quality == AirQualitySensor::LOW_POLLUTION) {
        Serial.printf("Low pollution!\n");
        display.setCursor(0,32);
        display.printf("Low pollution! %i",sensor.getValue());
        display.display();
    } else if (quality == AirQualitySensor::FRESH_AIR) {
        Serial.printf("Fresh air.\n");
        display.setCursor(0,32);
        display.printf("Fresh air. %i",sensor.getValue());
        display.display();
    }

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.printf("tempF.%0.2f\nhumidity.%0.2f\nmoisture.%i\nconcentration.%0.1f\n",tempF,humidity,moisture,concentration);
  display.display();
  display.clearDisplay();




  
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}
