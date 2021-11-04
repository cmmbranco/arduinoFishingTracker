#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/sleep.h>

// Time Control vars
#define GPSWAIT 10000
#define LEDDELAY 300
#define GPSAcquireTime 1000

//Other vars
#define LED A0
#define LEDGPS A2
#define REGISTER_BUTTON A1
#define WAKE_BUTTON 2
uint8_t WAKE_BUTTON_STATE = HIGH;
uint8_t PREV_WAKE_BUTTON_STATE = HIGH;
uint8_t REGISTER_BUTTON_STATE = LOW;
uint8_t PREV_REGISTER_BUTTON_STATE = LOW;
#define percent 100

// Fat limitation: filename (8 chars name) + "." (1 char) + file extension (3 chars)
#define filename "fishdata.csv"

// The TinyGPS++ object
#define rxPin 5
#define txPin 6
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
  pinMode(REGISTER_BUTTON, INPUT_PULLUP);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  digitalWrite (WAKE_BUTTON, HIGH);  // enable pull-up
  REGISTER_BUTTON_STATE = digitalRead(REGISTER_BUTTON);
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
    myFile.println(water_temp);
    
    // close the file:
    myFile.close();
  }
  
  digitalWrite(LED,LOW); 
  digitalWrite(LEDGPS, LOW);
}

void wake ()
{
  // cancel sleep as a precaution
  sleep_disable();
  // precautionary while we do other stuff
  detachInterrupt (0);
}  // end of wake

void loop(){
  digitalWrite(LED, HIGH);
  delay(50);
  digitalWrite(LED, LOW);
  delay(50);

  

  if(gps.satellites.value() > 3 && gps.satellites.isValid()){
    digitalWrite(LEDGPS,HIGH); // Turn on led to inform gps available
  } else {digitalWrite(LEDGPS,HIGH); delay(50); digitalWrite(LEDGPS,LOW);}
  
  REGISTER_BUTTON_STATE = digitalRead(REGISTER_BUTTON);
  //Serial.println(REGISTER_BUTTON_STATE);
  if ( REGISTER_BUTTON_STATE != PREV_REGISTER_BUTTON_STATE){
    // has the button switch been closed?
    if ( REGISTER_BUTTON_STATE == LOW ){
      fetchAndStore();
    }
  }
  
  PREV_REGISTER_BUTTON_STATE = REGISTER_BUTTON_STATE;

  
  PREV_WAKE_BUTTON_STATE = HIGH;
  WAKE_BUTTON_STATE = HIGH;
  
  WAKE_BUTTON_STATE = digitalRead(WAKE_BUTTON);
  //Serial.println(WAKE_BUTTON_STATE);
  if ( WAKE_BUTTON_STATE != PREV_WAKE_BUTTON_STATE){
    // has the button switch been closed?
    if ( WAKE_BUTTON_STATE == LOW ){
      //Serial.println("HERE");
      // disable ADC
      ADCSRA = 0;
      
      set_sleep_mode (SLEEP_MODE_PWR_DOWN);
      sleep_enable();
    
      // Do not interrupt before we go to sleep, or the
      // ISR will detach interrupts and we won't wake.
      noInterrupts ();
      
      // will be called when pin D2 goes low  
      attachInterrupt (0, wake, FALLING);
      EIFR = bit (INTF0);  // clear flag for interrupt 0
     
      // turn off brown-out enable in software
      // BODS must be set to one and BODSE must be set to zero within four clock cycles
      MCUCR = bit (BODS) | bit (BODSE);
      // The BODS bit is automatically cleared after three clock cycles
      MCUCR = bit (BODS); 
      
      // We are guaranteed that the sleep_cpu call will be done
      // as the processor executes the next instruction after
      // interrupts are turned on.
      interrupts ();  // one cycle
      sleep_cpu ();   // one cycle
      delay(150);
    }
  }
    
} 
