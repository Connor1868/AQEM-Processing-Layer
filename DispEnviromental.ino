/*
 *  AQEM Sensor Display System
 *  By: Connor Widder
 *  Date: 11/04/2019
 *  
 *  This sketch:
 *  Communicates with BME280s with different I2C addresses
 *  Displays BME280 sensor values on the LCD (I2C comm)
 *  
 *  Devices:
 *  SparkFun BME280 & CCS811 Sensor w/ Breakout board
 *  SparkFun RGB OpenLCD Serial display
 *  SparkFun Logic Level Converter
 *  
 *  Hardware connections: 
 *  BME280  /   Arduino 
 *  GND     /   GND 
 *  3.3     /   3.3 
 *  SDA     /   SDA 
 *  SCL     /   SCL
 *  
 *  LCD     /   Arduino 
 *  GND     /   GND 
 *  RAW     /   Logic Level Converer 
 *  SDA     /   SDA 
 *  SCL     /   SCL
 *  
 */

#include <Wire.h>
#include "SparkFunBME280.h"
BME280 mySensorA; //Uses default I2C address 0x77
#include <SerLCD.h> //Click here to get the library: http://librarymanager/All#SparkFun_SerLCD
SerLCD lcd; // Initialize the library with default I2C address 0x72

float temperatureF = 0;
//Backup sensor
float tempSensorValue = 0;
float temp_VOut = 0;
float temp2 = 0;
//CO
int co = 0;
//co_VOut = 0;

int humidity = 0;
float pressureFlt = 0;
int pressure = 0;

//LCD preset posistions
  const byte pos_Title[2] = {8,0};
  //const byte pos_Time[2] = {15,0};
  //const byte pos_Date[2] = {6,0};
  const byte pos_A1[2] = {2,1};
  const byte pos_A2[2] = {12,1};
  const byte pos_B1[2] = {2,3};
  const byte pos_B2[2] = {12,3};

//Bottons
const int buttonUpPin = 7;
int buttonUpState = 0;
const int buttonDownPin = 6;
int buttonDownState = 0;

int scrollIndex = 0;


//Char symbols
int degF_Sym = 0;
byte degF_Char[8] = {
  0x1C,
  0x14,
  0x1C,
  0x07,
  0x04,
  0x07,
  0x04,
  0x04
};
//Temp Symbol
int temp_Sym = 1;
byte temp_Char[8] = {
  0x04,
  0x0A,
  0x0A,
  0x0E,
  0x0E,
  0x1F,
  0x1F,
  0x0E
};
//Humidity Symbol
int hum_Sym = 2;
byte hum_Char[8] = {
  0x04,
  0x04,
  0x0A,
  0x0A,
  0x11,
  0x11,
  0x11,
  0x0E
};
//Ambient Light Symbol
int light_Sym = 3;
byte light_Char[8] = {
  0x00,
  0x04,
  0x15,
  0x0E,
  0x1F,
  0x0E,
  0x15,
  0x04
};
//CO
int co_Sym = 4;
/*byte co_Char[8] = {
  B01110,
  B10001,
  B10001,
  B01110,
  B00000,
  B10001,
  B10001,
  B01110
};*/
byte co_Char[8] = {
  B01110,
  B10001,
  B10001,
  B00000,
  B01110,
  B10001,
  B10001,
  B01110
};

void setup() {
  //Setup serial port
  Serial.begin(9600);

  Serial.println("----AQEM sensor validation----"); //Initial serial statment

  //Set up I2C Communications
  Wire.begin();

  //The default for the SparkFun Environmental Combo board is 0x77 (jumper open).
  //If you close the jumper it is 0x76
  //The I2C address must be set before .begin() otherwise the cal values will fail to load.
  mySensorA.setI2CAddress(0x77);
  if(mySensorA.beginI2C() == false) Serial.println("Sensor A connect failed");
  
  //Define Pins
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(buttonUpPin, INPUT);
  pinMode(buttonDownPin, INPUT);

  //Setup the LCD (20x4)
  lcd.begin(Wire);
  lcd.setBacklight(255, 255, 255); //Set backlight to bright white
  //lcd.setContrast(5); //Set contrast. Lower to 0 for higher contrast.
  lcd.clear(); //Clear the display - this moves the cursor to home position as well
  delay(2000);
  lcd.setCursor(3, 1);
  lcd.print("---- AQEM ----");
  lcd.setCursor(3, 2);
  delay(5000/14);
  for (int i = 0; i < 14;i++){
    lcd.print(char(255));
    delay(3000/14);
  }
  lcd.clear();

  //Custom Characters 
  lcd.createChar(degF_Sym, degF_Char); //Degree symbol with F
  lcd.createChar(temp_Sym, temp_Char); //Temnp symbol
  lcd.createChar(hum_Sym, hum_Char);  //Humidity symbol
  lcd.createChar(light_Sym, light_Char);  //Ambient light symbol
  lcd.createChar(co_Sym, co_Char);  //CO symbol symbol
}

void loop() {
  readSensors();
  //serialLog();
  //readButtonStates();
  updateDisplay();
}

void readSensors(){
  //Temp
  temperatureF = mySensorA.readTempF();
  //Temp 2
  tempSensorValue = analogRead(0);
  temp_VOut = (tempSensorValue * 5000) / 1024;
  temp2 = temp_VOut/10;
  //Humidity
  humidity = mySensorA.readFloatHumidity();
  //Pressure
  pressureFlt = mySensorA.readFloatPressure();
  pressureFlt /= 100.0;
  pressure = round(pressureFlt);
  //CO
  co = analogRead(1);

}

void updateDisplay(){
  //Read scroll buttons
  buttonUpState = digitalRead(buttonUpPin);
  buttonDownState = digitalRead(buttonDownPin);

  if((buttonUpState == 1) && (scrollIndex < 10)){
    scrollIndex++;
    delay(50);
  }else if((buttonDownState == 1) && (scrollIndex > 0)){
    scrollIndex--;
    delay(50);
  }
  
  //Header
  lcd.setCursorPos(pos_Title);
  lcd.print("AQEM ");
  lcd.print(scrollIndex);

  /*//Date and Time
   * lcd.setCursorPos(pos_Time);
   * lcd.print("00:00");
   * lcd.setCursorPos(pos_Date);
   * lcd.print("00/00/00");
   */
  
  //Temperature
  //lcd.setCursor(0, 1);
  lcd.setCursorPos(pos_A1);
  lcd.write(temp_Sym);
  lcd.print(temperatureF,1);
  lcd.print(String((char)223) + String("F"));

  lcd.setCursor(3,2);
  lcd.print(temp2,1);

  //Humidity
  lcd.setCursorPos(pos_A2);
  lcd.write(hum_Sym);
  lcd.print(String(humidity) + String((char)37));

  //Pressure
  lcd.setCursorPos(pos_B1);
  lcd.print(String(pressure) + String("hPa"));

  //CO
  lcd.setCursor(12,2);
  lcd.write(co_Sym);
  lcd.print(String(co));

  //Ambient Light
  lcd.setCursorPos(pos_B2);
  lcd.write(light_Sym);
  lcd.print(String(co) + String((char)37));
}

void serialLog(){
  //Display enviromental snesors
  //Temp
  Serial.print("TempA: ");
  Serial.print(temperatureF, 0);
  //Humidity 
  Serial.print(" HumidityA: ");
  Serial.print(humidity, 0);
  //Pressure
  Serial.print(" PressureA: ");
  Serial.print(pressure, 0);

  Serial.println();
}

void scrollPages(){
//LCD data locations
  /*
   * A1: (0,1), A2: (10,1)
   * B1: (0,3), A2: (10,3)
   */
  

  //Read button states
    //Next page
    //Prev page

  //Define pages
    /*
     * 1: S1 - S2  / S3 - S4
     * 2: S3 - S4  / S5 - S6
     * 3: S5 - S6  / S7 - S8
     * 4: S7 - S8  / S9 - S10
     * 5: S9 - S10 / S11 -S12
     */
   //Case statement base on button states/indexs to determine page
  
}
