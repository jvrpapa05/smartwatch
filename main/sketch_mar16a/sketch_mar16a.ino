#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

/** Libraries for the OLED */
#include "SSD1306Ascii.h"
#include "SSD1306AsciiAvrI2c.h"
/** Defined values for the OLED */
#define I2C_ADDRESS 0x3C
SSD1306AsciiAvrI2c oled;


MAX30105 particleSensor;

const byte RATE_SIZE = 255; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;


//////////////////////////////////////////////////////////////////////////////////////////////////
#include <SoftwareSerial.h>

//Create software serial object to communicate with SIM800L
SoftwareSerial mySerial(3, 2); //SIM800L Tx & Rx is connected to Arduino #3 & #2
//////////////////////////////////////////////////////////////////////////////////////////////////


#define BPM_NOT_VALID 100
#define BPM_SEND_SMS  140


/** Variable declaration */
#define stringPR "Pulse Rate: "
String sPulseRate = stringPR;

String sOxygenSaturation = "Oxygen Saturation: ";


/** Pre-processor directives */
//#define USE_SMS

void setup()
{
  Serial.begin(9600);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  /** OLED initialization. */
  oled.begin(&Adafruit128x32, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  oled.clear();
  /** first row */
  oled.println("Monitoring:");
  /** second row */
  oled.set1X();
  oled.println();
  oled.println(sPulseRate);
  /** third row */
  oled.set1X();
  oled.print(sOxygenSaturation);

  ///////////////////////////////////////////////////////////////////////////////////
  //Begin serial communication with Arduino and SIM800L
  mySerial.begin(9600);

  Serial.println("Initializing, please wait 1 second. ");
  delay(1000);

  #ifdef USE_SMS
  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();
  mySerial.println("AT+CSQ"); //Signal quality test, value range is 0-31 , 31 is the best
  updateSerial();
  mySerial.println("AT+CCID"); //Read SIM information to confirm whether the SIM is plugged
  updateSerial();
  mySerial.println("AT+CREG?"); //Check whether it has registered in the network
  updateSerial();
  #endif
}

void loop() {
  if (finger_present() == true){

    if (get_BPM() > BPM_NOT_VALID){
      /** Print to OLED only when:
      1. The finger is present.
      2. When the BMP is valid. */
      sPulseRate = stringPR + String(get_BPM());
  
      //Serial.println(sPulseRate);

      oled.clear();
      oled.println("Monitoring:");
      oled.println(sPulseRate);

      /** Clearing the variable content */
      sPulseRate = "";
    }else{
      oled.clear();
      oled.println("Monitoring:");
      oled.print("Data might not valid!");
      oled.println("\nBPM: " + String(get_BPM()));
    }
    
  }else{
    oled.clear();
    oled.println("Monitoring:");
    oled.print("Place finger.");
  }

  #ifdef USE_SMS
  if (finger_present() == true){

    if (get_BPM() > BPM_NOT_VALID){
      Serial.println(get_BPM());
      String sendThisMessage = ("BPM val: " + String(get_BPM()) + '\n');
      send_SMS(sendThisMessage);
    }

  }else{
    Serial.println("Place your finger.");
    delay(1000);
  }
  #endif
}

void send_SMS(String message){
  mySerial.println("AT"); //Once the handshake test is successful, it will back to OK
  updateSerial();

  mySerial.println("AT+CMGF=1"); // Configuring TEXT mode
  updateSerial();
  mySerial.println("AT+CMGS=\"+639755065672\"");//change ZZ with country code and xxxxxxxxxxx with phone number to sms
  updateSerial();
  mySerial.print(message); //text content
  updateSerial();
  mySerial.write(26);

  updateSerial();
}


void spo2_sensor(){
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < 50000)
    Serial.print(" No finger?");

  Serial.println();
}


float get_BPM(){
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.println(beatsPerMinute);

  return beatsPerMinute;
}

boolean finger_present(){
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
  }

  if (irValue < 50000){
    return false;
  } else{
    return true;
  }
}


void updateSerial()
{
  delay(500);
  while (Serial.available()) 
  {
    mySerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while(mySerial.available()) 
  {
    Serial.write(mySerial.read());//Forward what Software Serial received to Serial Port
  }
}