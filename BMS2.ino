// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    https://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//
// testar med mega 2560 mini. I2c D21 och D20
// ADC I2C device found at address 0x48
// Oled disp I2C device found at address 0x3C

#define VersionNumber "  Ver. 230507.07"


#include <Wire.h>
#include<ADS1115_lite.h>
#include <U8g2lib.h>
#include <EEPROM.h>

#define Button1Pin 19
#define Button2Pin 32
#define Button3Pin 34
#define Button4Pin 36
#define MenuMonitor 0
#define MenuCalibrate_14 1
#define MenuCalibrate_58 2

#define CutoffLowVoltageAddress 0 //EEPROM address
#define CutoffHighVoltageAddress 10 //EEPROM address
#define Factor1_address 20
#define Factor2_address 30
#define Factor3_address 40
#define Factor4_address 50
#define Factor5_address 60
#define Factor6_address 70
#define Factor7_address 80
#define Factor8_address 90

/*
  U8glib Example Overview:
    Frame Buffer Examples: clearBuffer/sendBuffer. Fast, but may not work with all Arduino boards because of RAM consumption
    Page Buffer Examples: firstPage/nextPage. Less RAM usage, should work with all Arduino boards.
    U8x8 Text Only Example: No RAM usage, direct communication with display controller. No graphics, 8x8 Text only.


    Lilla displayen (pchbutik 1667 har I2C-adress 0x3C
*/
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

ADS1115_lite ads(ADS1115_DEFAULT_ADDRESS);  // 0x48 addr pin connected to GND


long ADC0raw = 0;
double Volts=0;
double resolution=0;
float cellPotential[8] ={1,2,3,4,5,6,7,8};
float cellVotage[8] ={1,2,3,4,5,6,7,8};
float cellFactor[8] ={0.99634,2.0108,3,4,5,6,7,8};
int indicator = 0;
volatile bool state = LOW;
unsigned long lastISRmillis = 0L;
unsigned long lastADCmillis = 0L;
int ButtonPressCount = 0;
int currentMenu = 0;
int currentCell = 1;
int cusorPosition = 0;

void setup() {
  Wire.begin();
  pinMode(Button1Pin, INPUT_PULLUP);
  pinMode(Button2Pin, INPUT_PULLUP);
  pinMode(Button3Pin, INPUT_PULLUP);
  pinMode(Button4Pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(Button1Pin), ISR_button_pressed, FALLING);
  
  display.begin();

  showVersion();

  Serial.begin(115200);
  while (!Serial); // Leonardo: wait for serial monitor
  Serial.println("\nBMS!");
  ads_config();
  
  resolution=4096.00/32752.00;

  
  //float defalue = 3.0;
  //EEPROM.put(Factor4_address, defalue);
  
  EEPROM.get(Factor1_address, cellFactor[0]);
  EEPROM.get(Factor2_address, cellFactor[1]);
  EEPROM.get(Factor3_address, cellFactor[2]);
  EEPROM.get(Factor4_address, cellFactor[3]);
  EEPROM.get(Factor5_address, cellFactor[4]);
  EEPROM.get(Factor6_address, cellFactor[5]);
  EEPROM.get(Factor7_address, cellFactor[6]);
  EEPROM.get(Factor8_address, cellFactor[7]);
}
void ads_config(){
  ads.setGain(ADS1115_REG_CONFIG_PGA_4_096V); // GAIN_ONE and resolution to ± 4.096V
  ads.setSampleRate(ADS1115_REG_CONFIG_DR_8SPS); // Set to the fastest MODE 860Samples per sec
}


void loop() {
  int nDevices = 0;

  //ADC0raw=ads_read();
  //delay(200);
  //float result=(ADC0raw*resolution)/1000.00;

  unsigned long lastDispmillis = 0L;
  unsigned long lastRmillis = 0L;

  int updateDelay;
  switch (currentMenu) {
    case MenuMonitor:
      updateDelay = 1000;
      break;
    case MenuCalibrate_14:
      updateDelay = 8000;
      break;
    case MenuCalibrate_58:
      updateDelay = 8000;
      break;
  }
  if (millis() > lastADCmillis + updateDelay) {
    
    readVoltage(currentCell);
    currentCell++;
    if (currentCell==5) {
      currentCell = 1;
    }
    
   
    lastADCmillis = millis();
  }
  
  displayscreen();
  checkButtons();
  
}

void checkButtons() {
  
  if (digitalRead(Button2Pin)==LOW) {
    Serial.println("B2");
    if (currentMenu == MenuCalibrate_14 || currentMenu == MenuCalibrate_58) {
      cusorPosition++;
      if (cusorPosition> 4) {
        cusorPosition = 0;
      }
    }
    delay(200);
  }

  if (digitalRead(Button3Pin)==LOW) {
    Serial.println("B3");
    if (currentMenu == MenuCalibrate_14) {
      if (cusorPosition==4) {
        //Save Settings
        SaveFactorsToEEProm();
  
      } else {
        //Incr calibraion
        cellFactor[cusorPosition] = cellFactor[cusorPosition] - 0.0002;
        readVoltage(1+cusorPosition);
      }   
    } 

    if (currentMenu == MenuCalibrate_58) {
      if (cusorPosition==4) {
        //Save Settings
        SaveFactorsToEEProm();
  
      } else {
        //Decr calibraion
        cellFactor[4 + cusorPosition] = cellFactor[4 + cusorPosition] - 0.0002;
        readVoltage(1+4+cusorPosition);
      }   
    } 
  }
  
  if (digitalRead(Button4Pin)==LOW) {
    Serial.println("B4");
    if (currentMenu == MenuCalibrate_14) {
      if (cusorPosition==4) {
        //Save Setings       
        SaveFactorsToEEProm();
      } else {
        //Decr calibraion
        cellFactor[cusorPosition] = cellFactor[cusorPosition] + 0.0002;
        readVoltage(1 + cusorPosition);
      }
      
    }
    if (currentMenu == MenuCalibrate_58) {
      if (cusorPosition==4) {
        //Save Settings
        SaveFactorsToEEProm();
  
      } else {
        //Incr calibraion
        cellFactor[4 + cusorPosition] = cellFactor[4 + cusorPosition] + 0.0002;
        readVoltage(1+4+cusorPosition);
      }   
    }
    delay(200);
  }
  if (state)//if an interrup has occured
    {
      ButtonPressCount++; 
      Serial.print("Interrupt at time "); Serial.print(millis());
      Serial.print(" Antal: ");
      Serial.println(ButtonPressCount);
      
      state = false;//reset interrupt flag   
      currentMenu++;
      cusorPosition = 0;
      
      if (currentMenu >=3) {
        currentMenu = MenuMonitor;
      }
    }
}

void SaveFactorsToEEProm(){
  Serial.println("Settings saved");
  Serial.print("F3 = ");
  Serial.println(cellFactor[2], 4);
  
  EEPROM.put(Factor1_address, cellFactor[0]);
  EEPROM.put(Factor2_address, cellFactor[1]);
  EEPROM.put(Factor3_address, cellFactor[2]);
  EEPROM.put(Factor4_address, cellFactor[3]);
  EEPROM.put(Factor5_address, cellFactor[4]);
  EEPROM.put(Factor6_address, cellFactor[5]);
  EEPROM.put(Factor7_address, cellFactor[6]);
  EEPROM.put(Factor8_address, cellFactor[7]);

  display.clearBuffer();          // clear the internal memory
  display.setFont(u8g2_font_t0_11_te); 
  display.setCursor(10, 20);
  display.print(F("Saved"));
  display.sendBuffer();
  
  delay(2000);
  currentMenu = MenuMonitor;
}

void showVersion(){
  display.clearBuffer();          // clear the internal memory
  display.setFont(u8g2_font_t0_11_te); 
  display.setCursor(0, 20);
  display.println(F(VersionNumber)); 
  display.sendBuffer();
  delay(1000);
}

void ISR_button_pressed(void) 
{
  if (millis() > lastISRmillis + 200)// mS debounce time
  {
    state = true;//set flag
    lastISRmillis = millis();
  }
}

void readVoltage(int S){
  int16_t raw;
  int i;
  
  unsigned long starttime;
  unsigned long endtime ;
  
  starttime = micros(); //Record a start time for demonstration

  switch (S) {
    case 1:
      ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_0);  // Single mode input on AIN0 (AIN0 - GND)
      ads.triggerConversion();  // Triggered mannually 
      raw = ads.getConversion();  // int16_t value
      endtime = micros();
      //Serial.print("Conversion complete: "); Serial.print(" Time "); Serial.print(endtime - starttime);  Serial.println("us");
      
      cellPotential[0] = raw*resolution/1000.00 * cellFactor[0];
      cellVotage[0] = raw*resolution/1000.00 * cellFactor[0];
      break;
    case 2:
      ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_1);  
      ads.triggerConversion();  // Triggered mannually 
      raw = ads.getConversion();  // int16_t value
      cellPotential[1] = raw*resolution/1000.00 * cellFactor[1];
      cellVotage[1] = cellPotential[1] - cellVotage[0];
      break;
     case 3:
      ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_2);
      ads.triggerConversion();  // Triggered mannually 
      raw = ads.getConversion();  // int16_t value
      cellVotage[2] = raw*resolution/1000.00;
      break;
     case 4:
      ads.setMux(ADS1115_REG_CONFIG_MUX_SINGLE_3);  
      ads.triggerConversion();  // Triggered mannually 
      raw = ads.getConversion();  // int16_t value
      cellVotage[3] = raw*resolution/1000.00;
      break;
  }
}

void displayscreen() {

  int row = 0;
  int topPadding = 24;
  int rowHeight = 12;
  int col2 = 60;
  
  display.clearBuffer();          // clear the internal memory
  display.setFont(u8g2_font_t0_11_te); 
    
  if (currentMenu == MenuMonitor) {
    float s1=(ADC0raw*resolution)/1000.00;
    
    display.setCursor(0,10);
    if (indicator == 0) {
      display.print(F("BMS monitor"));
      indicator++;
    } else {
      display.print(F("BMS monitor       -"));
      indicator = 0;
    }
    
    for(row=0; row<4; row++){
      display.setCursor(0, topPadding + (rowHeight*row));
      display.print(F("S")); 
      display.setCursor(6, topPadding + (rowHeight*row));
      display.print(1+ row);
      
      display.setCursor(18, topPadding + (rowHeight*row));   
      display.print(cellVotage[row], 3);
  
      display.setCursor(43+5, topPadding + (rowHeight*row));
      display.print(F("V")); 
  
      display.setCursor(col2, topPadding + (rowHeight*row));
      display.print(F("S")); 
      display.setCursor(6 + col2, topPadding + (rowHeight*row));
      display.print(1+ 4 + row);
  
      display.setCursor(18 + col2, topPadding + (rowHeight*row));   
      display.print(cellVotage[4 + row], 3);
    }
  }
  
  if (currentMenu == MenuCalibrate_14) {
    display.setCursor(0,10);
    display.print(F("Calibrate cell 1-4")); 
    if (cusorPosition==4) {
      display.setCursor(0, topPadding + (rowHeight));
      display.print(F("Save settings *"));
      
  
    } else {
      for(row=0; row<4; row++){
        display.setCursor(0, topPadding + (rowHeight*row));
        display.print(F("P")); 
        display.setCursor(6, topPadding + (rowHeight*row));
        display.print(1+ row);
        
        display.setCursor(18, topPadding + (rowHeight*row));   
        display.print(cellPotential[row], 3);
    
        display.setCursor(43+5, topPadding + (rowHeight*row));
        display.print(F("V")); 
    
        display.setCursor(col2, topPadding + (rowHeight*row));
        display.print(F("F")); 
        display.setCursor(6 + col2, topPadding + (rowHeight*row));
        display.print(1+ row);
    
        display.setCursor(18 + col2, topPadding + (rowHeight*row));   
        display.print(cellFactor[row], 4);
  
        if (cusorPosition==row) {
          display.setCursor(18 + 30 + 7 + col2, topPadding + (rowHeight*row));   
          display.print("*");
        }
      }
    }
        
  }
  
  if (currentMenu == MenuCalibrate_58) {
    display.setCursor(0,10);
    display.print(F("Calibrate cell 5-8")); 
    if (cusorPosition==4) {
      display.setCursor(0, topPadding + (rowHeight));
      display.print(F("Save settings *"));
      
    } else {
      for(row=0; row<4; row++){
        display.setCursor(0, topPadding + (rowHeight*row));
        display.print(F("P")); 
        display.setCursor(6, topPadding + (rowHeight*row));
        display.print(1 + 4 + row);
        
        display.setCursor(18, topPadding + (rowHeight*row));   
        display.print(cellPotential[4 + row], 3);
    
        display.setCursor(43+5, topPadding + (rowHeight*row));
        display.print(F("V")); 
    
        display.setCursor(col2, topPadding + (rowHeight*row));
        display.print(F("F")); 
        display.setCursor(6 + col2, topPadding + (rowHeight*row));
        display.print(1+ 4 + row);
    
        display.setCursor(18 + col2, topPadding + (rowHeight*row));   
        display.print(cellFactor[4 + row], 4);
  
        if (cusorPosition==row) {
          display.setCursor(18 + 30 + 7 + col2, topPadding + (rowHeight*row));   
          display.print("*");
        }
      }
    }
        
  }
  
  display.sendBuffer();
}
