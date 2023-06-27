/* 
  Name: Wilson Dhalwani 
  Date: 5/9/23 
  Projekt: HOSA ILC DMAC
  Credits: 
    -> SparkFun MAX3010x Sensor Library for help with GYMAX30102 Sensor
    -> https://www.instructables.com/DS18B20-temperature-probe-with-LCD/ for help with temp sensor 
    -> https://randomnerdtutorials.com/esp32-wifimulti/#:~:text=It%20allows%20you%20to%20register,next%20network%20on%20the%20list for help with multiconnectWifi
    -> Giacomo Pugliese for help with website interfacing
    -> Myself for the rest (yes I actually had to program a lot; it was fun tho)
  Notes: 
    Hello either Future Wilson or HOSA ILC Judges!!!! :))
      -> If Future Wilson: 
          > Make sure to change the wifi for the one that is currently accesible
          > Also Battery Capacitance
            - Prob will either be 500 mAh or 2000 mAh
          > Light Table 
            - Red: Bad; Some Error (Reset board and adjust; Check serial monitor)
            - Green: GOOD GOOD GOOD YOU ARE FINALLY DOING SOMETHING GOOD 
            - Blue: We be loadin yk 
          //////////////////////////////////////////////////////////////////////////////////////////////> deviceID = 123005; CAN BE FOUND DOWN BELOW
          > deviceID = 5261; CAN BE FOUND DOWN BELOW
      -> If HOSA ILC Judges: 
          > Have a good day, and enjoy our projekt !!!! 
*/


//Libraries
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include <HTTPClient.h>
#include <SoftwareSerial.h>
#include <OneWire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

//WIFI and Website Stuff
  //HOME
  //  const char* WIFI_SSID = "wifiuser";
  //  const char* WIFI_PASS = "wifipassword";
  //SCHOOL
  //  const char* WIFI_SSID = "Bergen-Staff";
  //  const char* WIFI_PASS = "I@m@nEduc@tor";
  //GIACOMO iPhone
  //  const char* WIFI_SSID = "Giacomo-iPhone";
  //  const char* WIFI_PASS = "Octane3245";  
  //Wilson iPhone
    const char* WIFI_SSID = "Midlandswim iPhone";
    const char* WIFI_PASS = "wd123456";
  //HOSA ILC
  //  const char* WIFI_SSID = "HOSAILC23";
  //  const char* WIFI_PASS = "2023!Hosa";


//Wifi & Website
  const char* FASTAPI_BACKEND_URL = "https://smart-chart-backend-8726395027f2.herokuapp.com/data";
  HTTPClient http;

//Temp Sensor Stuff
  int DS18S20_Pin = 32;        //DS18S20 Signal pin on digital 32
  char tmpstring[10];
  OneWire ds(DS18S20_Pin);   //Temperature chip i/o

//battery Stuff
  const float VOLTAGE_DIVIDER_RATIO = 2.0;   // Voltage divider ratio
  const float VOLTAGE_DIVIDER_OFFSET = 0.0;  // Voltage divider offset
  const float BATTERY_CAPACITY = 500.0;      // Battery capacity in mAh   
  //const float BATTERY_CAPACITY = 2000.0;      // Battery capacity in mAh  FOR BIG BOI BATTERY
  const float BATTERY_VOLTAGE_MIN = 3.0;     // Minimum safe battery voltage
  //const float BATTERY_VOLTAGE_MID = 3.7;     // Minimum safe battery voltage NEVER USED, ONLY FOR REFERENCE
  const float BATTERY_VOLTAGE_MAX = 4.2;     // Maximum safe battery voltage
  float battery_voltage = 0.0;     // Battery voltage in volts
  float battery_percentage = 0.0;  // Battery life percentage

//PulseOxi Sensor Stuff
  MAX30105 particleSensor;
  #define MAX_BRIGHTNESS 255
  #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  //Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
  //To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
  uint16_t irBuffer[100];   //infrared LED sensor data
  uint16_t redBuffer[100];  //red LED sensor data
  #else
  uint32_t irBuffer[100];   //infrared LED sensor data
  uint32_t redBuffer[100];  //red LED sensor data
  #endif
  int32_t bufferLength;   //data length
  int32_t spo2;           //SPO2 value
  int8_t validSPO2;       //indicator to show if the SPO2 calculation is valid
  int32_t heartRate;      //heart rate value
  int8_t validHeartRate;  //indicator to show if the heart rate calculation is valid
  byte pulseLED = 14;  //Must be on PWM pin
  byte readLED = 13;   //Blinks with each data read

//Where the fun begins! Initilization
void setup() {  
  //board initator
    Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to Wi-Fi...");
    neopixelWrite(0, 0, 0, 255);
  }

  Serial.println("Connected to Wi-Fi!");
  neopixelWrite(0, 0, 255, 0);

  // Configure ADC for 12-bit resolution and voltage range of 0-3.3V
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);
    pinMode(pulseLED, OUTPUT);
    pinMode(readLED, OUTPUT);

  // Initialize sensor
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST))  //Use default I2C port, 400kHz speed
    {
      Serial.println(F("MAX30105 was not found. Please check wiring/power."));
      neopixelWrite(0, 255, 0, 0);
      while (1)
        ;
    }

  //load and implement custom settings
    neopixelWrite(0, 0, 255, 0);
    byte ledBrightness = 60;  //Options: 0=Off to 255=50mA
    byte sampleAverage = 1;   //Options: 1, 2, 4, 8, 16, 32  (1 gives way more accurate bpm)
    byte ledMode = 2;         //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    byte sampleRate = 100;    //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411;     //Options: 69, 118, 215, 411
    int adcRange = 4096;      //Options: 2048, 4096, 8192, 16384

    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);  //Configure sensor with these settings
}

//gettemp from sensor function
float getTemp() {
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if (!ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    neopixelWrite(0, 255, 0, 0);
    return -1000;
  }

  if (addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    neopixelWrite(0, 255, 0, 0);
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44, 1);  // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);
  ds.write(0xBE);  // Read Scratchpad

  for (int i = 0; i < 9; i++) {  // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB);  //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}

//Where the fun perpetuates into either bliss or chaos....
void loop() {

  //temp sensor
    float temperature = getTemp();
    float tmp = (int)temperature;   //in celsious
    float tempF = ((tmp * 1.8) + 32);   //1.8 = 9/5 so gives farenheit
    Serial.println(tempF);

  //batteryLife
    // Read battery voltage using voltage divider circuit
    int battery_adc_value = analogRead(35);                                                    // ADC channel 35 (GPIO35)
    float battery_adc_voltage = (battery_adc_value / 4095.0) * 3.3;                            // Convert ADC value to voltage
    battery_voltage = (battery_adc_voltage * VOLTAGE_DIVIDER_RATIO) + VOLTAGE_DIVIDER_OFFSET;  // Calculate battery voltage
    // Calculate battery life percentage
    battery_percentage = ((battery_voltage - BATTERY_VOLTAGE_MIN) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN)) * 100.0;
    if (battery_percentage > 100.0) {
      battery_percentage = 100.0;
    } else if (battery_percentage < 0.0) {
      battery_percentage = 0.0;
    }
    // Print battery information to serial monitor
    Serial.print("Battery Voltage: ");
    Serial.print(battery_voltage, 2);
    Serial.print(" V, Battery Life: ");
    Serial.print(battery_percentage, 0);
    Serial.println("%");

  //MAX30102
    bufferLength = 100;  //buffer length of 100 stores 4 seconds of samples running at 25sps
    //read the first 100 samples, and determine the signal range
    for (byte i = 0; i < bufferLength; i++) {
      while (particleSensor.available() == false)  //do we have new data?
        particleSensor.check();                    //Check the sensor for new data
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();  //We're finished with this sample so move to next sample
      neopixelWrite(0, 0, 0, 255); 
      Serial.println("Calibrating..."); 
      // Serial.print(F("red="));
      // Serial.print(redBuffer[i], DEC);
      // Serial.print(F(", ir="));
      // Serial.println(irBuffer[i], DEC);
    }

    //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);


  //Continuously taking samples from MAX30102, temp sensor, and then sends to website.  
  while (1) {
    
    //data assignments
      float temp = tempF;
      float battLife = battery_percentage;
      int deviceID = 5261;
      float HeartRate = heartRate;  //HR should be between 60 and 95 
      float Spo2 = spo2;       //SPO2 should be between 95 and 100
      float alive = 1; 
 

    //send data to Website
      if (WiFi.status() == WL_CONNECTED) {
        String json = "{\"temp\": " + String(temp) + ", \"battLife\": " + String(battLife) + ", \"deviceID\": " + String(deviceID) + ", \"HeartRate\": " + String(HeartRate) + ", \"Spo2\": " + String(Spo2) + ", \"alive\": " + String(alive) + "}";
        http.begin(FASTAPI_BACKEND_URL);
        http.addHeader("Content-Type", "application/json");
        //Serial.println("Hello");
        int httpCode = http.POST(json);
        String payload = http.getString();

        if (httpCode > 0) {
          Serial.println("Values sent to the FastAPI backend!");
          neopixelWrite(0, 0, 255, 0);
        } else {
          neopixelWrite(0, 255, 0, 0);
          Serial.print("Error sending values. HTTP code: ");
          Serial.println(httpCode);
        }

        http.end();
      } else {
        Serial.println("Error connecting to the FastAPI backend!");
        neopixelWrite(0, 255, 0, 0);
      }

    //help the memory
      //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
      for (byte i = 25; i < 100; i++) {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
      }

    //keep taking data, end if no finger
      //take 25 sets of samples before calculating the heart rate.
      for (byte i = 75; i < 100; i++) {
        while (particleSensor.available() == false)  //do we have new data?
          particleSensor.check();                    //Check the sensor for new data

        digitalWrite(readLED, !digitalRead(readLED));  //Blink onboard LED with every data read

        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample();  //We're finished with this sample so move to next sample

        if (irBuffer[i] < 6000) {
          Serial.println(F("No Finger in Clip! Reset Board."));
          neopixelWrite(0, 255, 0, 0);
          while (1);
        }

      //display that info to Serial monitor
        //send samples and calculation result to terminal program through UART
        Serial.print(F("red="));
        Serial.print(redBuffer[i], DEC);
        Serial.print(F(", ir="));
        Serial.print(irBuffer[i], DEC);

        Serial.print(F(", HR="));
        Serial.print(heartRate, DEC);

        Serial.print(F(", HRvalid="));
        Serial.print(validHeartRate, DEC);

        Serial.print(F(", Temp="));
        Serial.print(tempF, DEC);

        Serial.print(F(", BattLife="));
        Serial.print(battery_percentage, DEC);

        Serial.print(F(", SPO2="));
        Serial.print(spo2, DEC);

        Serial.print(F(", SPO2Valid="));
        Serial.println(validSPO2, DEC);
    }

    //MAX30102
      //After gathering 25 new samples recalculate HR and SP02
      maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    //tempSensor
      float temperature = getTemp();
      float tmp = (int)temperature;
      float tempF = ((tmp * 1.8) + 32);

    //batteryLife
      // Read battery voltage using voltage divider circuit
      int battery_adc_value = analogRead(35);                                                    // ADC channel 35 (GPIO35)
      float battery_adc_voltage = (battery_adc_value / 4095.0) * 3.3;                            // Convert ADC value to voltage
      battery_voltage = (battery_adc_voltage * VOLTAGE_DIVIDER_RATIO) + VOLTAGE_DIVIDER_OFFSET;  // Calculate battery voltage
      // Calculate battery life percentage
      battery_percentage = ((battery_voltage - BATTERY_VOLTAGE_MIN) / (BATTERY_VOLTAGE_MAX - BATTERY_VOLTAGE_MIN)) * 100.0;
      if (battery_percentage > 100.0) {
      battery_percentage = 100.0;
      } else if (battery_percentage < 0.0) {
        battery_percentage = 0.0;
      }

    delay(1000);
  }
}