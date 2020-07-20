
#include "DHT.h"

#define DHTPIN_A 6    // Digital pin connected to the DHT sensor
#define DHTPIN_B 9
#define DHTPIN_C 12     // Digital pin connected to the DHT sensor
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321


#define deviceName "wnf_remote_101"
#define devEUI "101"

#include <SHT1x.h>
// Specify data and clock connections and instantiate SHT1x object
#define dataPin  10
#define clockPin 11
SHT1x sht1x(dataPin, clockPin);

DHT dht_a(DHTPIN_A, DHTTYPE);
DHT dht_b(DHTPIN_B, DHTTYPE);
DHT dht_c(DHTPIN_C, DHTTYPE);

#include <SPI.h>
#include <RH_RF95.h>

#include <Wire.h>

#include "SparkFunBME280.h"
#include "RTCZero.h" // https://github.com/arduino-libraries/RTCZero
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson

#define VBATPIN A7
#define LED 13
#define INTERVALSWITCH A2

const unsigned SHORT_INTERVAL = 30; // 10 second interval
const unsigned LONG_INTERVAL = 1800; // 30 minute interval

int TX_INTERVAL = SHORT_INTERVAL; // default short interval

//const unsigned TX_INTERVAL = 300; //default is 30 minutes

// for feather m0  
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

#define RTC_SLEEP 1

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

BME280 mySensor;

RTCZero rtc;

StaticJsonDocument<200> doc;

double round2(double value) {
   return (int)(value * 100 + 0.5) / 100.0;
}

void setup() 
{

     dht_a.begin();
     dht_b.begin();
     dht_c.begin();
     
  pinMode(INTERVALSWITCH, INPUT_PULLUP);

   pinMode(LED, OUTPUT);

  //Serial.println("here!");

if(RTC_SLEEP) {
      // Initialize RTC
    rtc.begin();
    // Use RTC as a second timer instead of calendar
    rtc.setEpoch(0);
}



  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(9600);

  /*
  while(!Serial) {

    ;
  }
  */

  delay(100);

  //Serial.println("Feather LoRa TX Test!");

  Wire.begin();

Wire.begin();
  Wire.setClock(400000); //Increase to fast I2C speed!
  
  if (mySensor.beginI2C() == false) //Begin communication over I2C
  {
    Serial.println("The sensor did not respond. Please check wiring.");
    while(1); //Freeze
  }

  mySensor.setMode(MODE_SLEEP); //Sleep for now

  
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  //Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission



void loop()
{


pinMode(LED, OUTPUT);

int sleepmode = digitalRead(INTERVALSWITCH);
//Serial.print("sleep button:");
//Serial.println(sleepmode);

if(sleepmode) {

TX_INTERVAL = LONG_INTERVAL;

}
else {

TX_INTERVAL = SHORT_INTERVAL;

}

TX_INTERVAL = SHORT_INTERVAL;

  //Serial.print("sleep interval = ");
  //Serial.print(TX_INTERVAL);
  //Serial.println(" seconds");
  
  // battery measurement
   float measuredvbat = analogRead(VBATPIN);
measuredvbat *= 2;    // we divided by 2, so multiply back
measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
measuredvbat /= 1024; // convert to voltage


// from BME280

mySensor.setMode(MODE_FORCED); //Wake up sensor and take reading
 
 //long startTime = millis();
  while(mySensor.isMeasuring() == false) ; //Wait for sensor to start measurment
  while(mySensor.isMeasuring() == true) ; //Hang out while sensor completes the reading    
 // long endTime = millis();


  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h_d_a = dht_a.readHumidity();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float d_t_a = dht_a.readTemperature(true);

  float h_d_b = dht_b.readHumidity();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float d_t_b = dht_b.readTemperature(true);

  float h_d_c = dht_c.readHumidity();
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float d_t_c = dht_c.readTemperature(true);
  

  float b_temp = mySensor.readTempF();
  float h_b = mySensor.readFloatHumidity();
  float b_pressure = mySensor.readFloatPressure();


float s_temp_c;
  float s_temp_f;
  float h_s;

  // Read values from the sensor
  s_temp_f = sht1x.readTemperatureF();
  h_s = sht1x.readHumidity();

  
//doc["devEUI"] = devEUI;
//doc["deviceName"]=deviceName;

doc["t_b"] = round2(b_temp);
doc["t_s"] = round2(s_temp_f);
doc["t_d_a"] = round2(d_t_a);
doc["t_d_b"] = round2(d_t_b);
doc["t_d_c"] = round2(d_t_c);

doc["h_b"] = round2(h_b);
doc["h_s"] = round2(h_s);
doc["h_d_a"] = round2(h_d_a);
doc["h_d_b"] = round2(h_d_b);
doc["h_d_c"] = round2(h_d_c);

doc["p_b"] = round2(b_pressure);

Serial.println("bme280, sh10, DHT22_A, DHT22_B, DHT22_C");
Serial.print(h_b);
Serial.print(", ");
Serial.print(h_s);
Serial.print(", ");
Serial.print(h_d_a);
Serial.print(", ");
Serial.print(h_d_b);
Serial.print(", ");
Serial.print(h_d_c);
Serial.println();
Serial.println();


//doc["BatV"] = round2(measuredvbat);

  //Serial.println(temp);
  //Serial.println(humidity);
  //Serial.println(pressure); 
  
  delay(1000); // Wait 1 second between transmits, could also 'sleep' here!
 // Serial.println("Transmitting..."); // Send a message to rf95_server
  
  char radiopacket[140] = "Hello World #      ";

  serializeJson(doc,radiopacket,140);
  
  //itoa(packetnum++, radiopacket+13, 10);
  //Serial.print("Sending "); Serial.println(radiopacket);
  //radiopacket[19] = 0;
  
  //Serial.println("Sending...");
  delay(10);
  rf95.send((uint8_t *)radiopacket, 140);

  //Serial.println("Waiting for packet to complete..."); 
  delay(10);
  rf95.waitPacketSent();
  // Now wait for a reply

  rf95.sleep();

digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(100);               // wait for a second
  
digitalWrite(LED, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);               // wait for a second
  digitalWrite(LED, LOW);    // turn the LED off by making the voltage LOW
  delay(100);               // wait for a second


//delay(2000); // to make sure we wait before sleeping

  if(RTC_SLEEP) {

    //pinMode(13,INPUT_PULLUP);
    
            // Sleep for a period of TX_INTERVAL using single shot alarm
            rtc.setAlarmEpoch(rtc.getEpoch() + TX_INTERVAL);
            rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
            rtc.attachInterrupt(alarmMatch);
            
            // USB port consumes extra current
            USBDevice.detach();
           
            // Enter sleep mode
            rtc.standbyMode();
            
            
            // Reinitialize USB for debugging
            USBDevice.init();
            USBDevice.attach();
            }
            else {

              delay(TX_INTERVAL*1000); // delay regular way if not sleeping
            }
  
}

void alarmMatch()
{

}
