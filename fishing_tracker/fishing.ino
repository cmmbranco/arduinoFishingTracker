#include <TinyGPS.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/sleep.h>
#include <SoftwareSerial.h>

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

// The TinyGPS object
#define rxPin 6
#define txPin 5
float lati = 0.00, longi = 0.00, last_lat = 0.00, last_long = 0.00;
TinyGPS gps;
SoftwareSerial ss(rxPin,txPin);

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

// Wake arduino fuction (via interrupt)
void wake ()
{
  // cancel sleep as a precaution
  sleep_disable();
  // precautionary while we do other stuff
  detachInterrupt (0);
}  // end of wake


////
//// From example in TinyGPS library
////

////
//// Delay that polls the Serial for GPS data
////
static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

////
//// Print GPS lat and long helper function
////
static void print_float(float val, float invalid, int len, int prec)
{
  if (val == invalid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; i++)
      Serial.print(',');
  }
  smartdelay(0);
}

////
//// Print GPS date helper function
////
static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("********** ******** ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d,%02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    Serial.print(sz);
  }
  smartdelay(0);
}

////
//// Main IO function
////
//// Fetch data and store
void fetchAndStore(){
  digitalWrite(LED,HIGH); // Turn on led to inform processing
  
  
  // Get GPS data
  byte month, day, hour, minute, second, hundredths;
  int year;
  float flat, flon;
  unsigned long age = 0;
  char sz[32];
  // Fetch data for the last "X" ms.
  smartdelay(100);
  gps.f_get_position(&flat, &flon, &age);
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);

  //compose time/date string with comma between
  sprintf(sz, "%02d/%02d/%02d,%02d:%02d:%02d ",month, day, year, hour, minute, second);

  ////
  //// Send the command to get water temperatures
  ////
  sensors.requestTemperatures();
  water_temp = sensors.getTempCByIndex(0);
  water_temp = water_temp + water_temp * 0.01; // calibrated in armpit vs thermometer

  Serial.println(water_temp);

  ////
  //// Get Air Pressure and Air Temperature
  ////

  air_pressure = bmp.readPressure()-bmp.readPressure()*.0028; // absolute pressure minus error (error calculated at a known height, temperature and sea level pressure)
  air_temp = bmp.readTemperature(); // reading sounds legit
  
  //// open the file. note that only one file can be open at a time,
  //// so you have to close this one before opening another.
  myFile = SD.open(filename, FILE_WRITE);

  //// if the file opened okay, write to it:
  if (myFile) {
    myFile.print(flat, 6); myFile.print(','); myFile.print(flon, 6); myFile.print(',');myFile.print(sz);myFile.print(',');
    myFile.print(air_temp); myFile.print(',');myFile.print(air_pressure); myFile.print(',');
    myFile.println(water_temp);
    
    //// close the file:
    myFile.close();
  }
  
  digitalWrite(LED,LOW); 
  digitalWrite(LEDGPS, LOW);
}

////
//// Setup Code
////
void setup(){
  // Setup pins
  pinMode(LED, OUTPUT);
  pinMode(LEDGPS, OUTPUT);
  pinMode(REGISTER_BUTTON, INPUT_PULLUP);
  pinMode(rxPin, INPUT);
  pinMode(txPin, OUTPUT);
  ss.begin(9600);
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


void loop(){
  
   
  float flat, flon;
  unsigned long age = 0;
  smartdelay(100);
  gps.f_get_position(&flat, &flon, &age);


  //flash slow if lock acquired
  if(flat !=  TinyGPS::GPS_INVALID_F_ANGLE){
    digitalWrite(LED, HIGH);
    delay(300);
    digitalWrite(LED, LOW);
    delay(300);
    
    
    //// Uncomment the following block for live Serial debug
    //// print_float functions taken from the example and used for the above if condition troubleshooting aka gps lock

    ////
    //// Sample block, if uncommented keep the following one commented
    ////
    
    //print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
    //print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
    //print_date(gps);

    ////
    //// Block implement in register function
    ////
    
    //Serial.println(gps.satellites());
    //print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
    //print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 10, 6);
    //print_date(gps);
    //Serial.println("");
    // Get GPS data
    //byte month, day, hour, minute, second, hundredths;
    //int year;
    //float flat, flon;
    //unsigned long age = 0;
    //char sz[32];
    // Fetch data for the last "X" ms.
    //smartdelay(100);
    //gps.f_get_position(&flat, &flon, &age);
    //gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
    
    //Serial.print(flat, 6);
    //Serial.print(",");
    //Serial.print(flon, 6);
    //Serial.print(",");
    //sprintf(sz, "%02d/%02d/%02d,%02d:%02d:%02d ",month, day, year, hour, minute, second);
    //Serial.print(sz);
    
  }
  // If satellite lock fails, flash fast
  else{
    digitalWrite(LED, HIGH);
    delay(25);
    digitalWrite(LED, LOW);
    delay(25);
  }
  ////
  //// Routine implementation to sleep arduino on dedicated button click
  ////
  REGISTER_BUTTON_STATE = digitalRead(REGISTER_BUTTON);
  //Serial.println(REGISTER_BUTTON_STATE);
  if ( REGISTER_BUTTON_STATE != PREV_REGISTER_BUTTON_STATE){
    // has the button switch been closed?
    if ( REGISTER_BUTTON_STATE == LOW ){
      fetchAndStore();
    }
  }
  
  PREV_REGISTER_BUTTON_STATE = REGISTER_BUTTON_STATE;

  // reset button state to high, if pressed wake
  PREV_WAKE_BUTTON_STATE = HIGH;
  WAKE_BUTTON_STATE = HIGH;
  ////
  //// wake routine
  ////
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
