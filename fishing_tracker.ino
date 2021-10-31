#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Time Control vars
#define GPSWAIT 10000
#define LEDDELAY 300
#define GPSAcquireTime 1000

//Other vars
#define LED A0
#define LEDGPS A2
#define BUTTON A1
uint8_t BUTTON_STATE = LOW;
uint8_t PREV_BUTTON_STATE = LOW;
#define percent 100

// Fat limitation: filename (8 chars name) + "." (1 char) + file extension (3 chars)
#define filename "fishdata.csv"

// The TinyGPS++ object
#define rxPin 2
#define txPin 3
float lati = 0.00, longi = 0.00, last_lat = 0.00, last_long = 0.00;
TinyGPSPlus gps;

// BMP
Adafruit_BMP280 bmp; // I2C
float air_temp, air_pressure;

// SD CARD
#define SD_CS 10
File myFile;

// Water Temp
// Data wire is plugged into digital pin 2 on the Arduino
#define WATER_PROBE_PIN 4
// Setup a oneWire instance to communicate with any OneWire device
OneWire oneWire(WATER_PROBE_PIN);

// Pass oneWire reference to DallasTemperature library
DallasTemperature sensors(&oneWire);
float water_temp;

int bla = 0;

// Setup Code
void setup(){
  // Setup pins
  pinMode(LED, OUTPUT);
  pinMode(LEDGPS, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  BUTTON_STATE = digitalRead(BUTTON);
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  if (!SD.begin(10)) {
    while (1);
  }
  
  // Start BMP
  bmp.begin();
  /* Default settings from datasheet for BMP280. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */ 
  // Start SD Card
  SD.begin(SD_CS);
  
  // Start Water sensor
  sensors.begin();
  
}


//
// Main IO function
//
// Fetch data and store
void fetchAndStore(){
  digitalWrite(LED,HIGH); // Turn on led to inform processing
  
  // Get GPS data
  unsigned long start = millis();
  do{
    while (Serial.available())
      gps.encode(Serial.read());
  } while (millis() - start < GPSWAIT); // This custom version of delay() ensures that the gps object is being "fed".

  lati = gps.location.lat();
  longi = gps.location.lng();
  
  // Send the command to get water temperatures
  sensors.requestTemperatures();
  water_temp = sensors.getTempCByIndex(0);
  water_temp = water_temp + water_temp * 0.01; // calibrated in armpit vs thermometer

  // Get Air Pressure and Air Temperature

  air_pressure = bmp.readPressure()-bmp.readPressure()*.0028; // absolute pressure minus error (error calculated at a known height, temperature and sea level pressure)
  air_temp = bmp.readTemperature(); // reading sounds legit
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open(filename, FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    myFile.print(lati); myFile.print(','); myFile.print(longi); myFile.print(',');
    myFile.print(air_temp); myFile.print(',');myFile.print(air_pressure); myFile.print(',');
    myFile.print(water_temp); myFile.print(',');
    
    // close the file:
    myFile.close();
    Serial.println(F("done."));
  }
  
  digitalWrite(LED,LOW); 
}

void loop(){

  if(gps.satellites.value() > 3 && gps.satellites.isValid()){
    digitalWrite(LEDGPS,HIGH); // Turn on led to inform gps available
  } else {digitalWrite(LEDGPS,LOW);}
  
  BUTTON_STATE = digitalRead(BUTTON);
  Serial.println(BUTTON_STATE);
  if ( BUTTON_STATE != PREV_BUTTON_STATE){
    // has the button switch been closed?
    if ( BUTTON_STATE == LOW ){
      fetchAndStore();
    }
  }
  PREV_BUTTON_STATE = BUTTON_STATE;
} 
