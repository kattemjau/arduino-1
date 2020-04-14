/*
 * Test Code for the ADE9153AAPI
 * 
 * Designed specifically to work with the EV_ADE9153ASHIELDZ board and
 * ADE9153AAPI library
 * ---- http://www.analog.com/ADE9153A
 *
 * Created by David Lath for Analog Devices Inc., January 8, 2018
 * Modified by Sindre Hovland for Heimdall Power AS, April 9, 2020
 * 
 * Depending on Adafruit SSD1306 library
 *
 * Copyright (c) 2018, Analog Devices, Inc.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted (subject to the limitations in the disclaimer 
 * below) provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the 
 * documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of Analog Devices, Inc. nor the names of its 
 * contributors may be used to endorse or promote products derived from this 
 * software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED 
 * BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, 
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#define ARM_MATH_CM0PLUS

#include <SPI.h>
#include <ADE9153A.h>
#include <ADE9153AAPI.h>

//inclues for OLED screen
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);


/* Basic initializations */
#define SPI_SPEED 1000000     //SPI Speed
#define CS_PIN 8              //8-->Arduino Zero. 15-->ESP8266 
#define ADE9153A_RESET_PIN 4  //On-board Reset Pin
#define USER_INPUT 5          //On-board User Input Button Pin
#define LED 6                 //On-board LED pin
ADE9153AClass ade9153A;

struct EnergyRegs energyVals;  //Energy register values are read and stored in EnergyRegs structure
struct PowerRegs powerVals;    //Metrology data can be accessed from these structures
struct RMSRegs rmsVals;  
struct PQRegs pqVals;
struct AcalRegs acalVals;
struct Temperature tempVal;

void readandwrite(void);
void resetADE9153A(void);

int ledState = LOW;
int inputState = LOW;
unsigned long lastReport = 0;
const long reportInterval = 2000;
const long blinkInterval = 500;


void setup() {
  /* Pin and serial monitor setup */
  pinMode(LED, OUTPUT);
  pinMode(USER_INPUT, INPUT);
  pinMode(ADE9153A_RESET_PIN, OUTPUT);
  digitalWrite(ADE9153A_RESET_PIN, HIGH);  
  Serial.begin(115200);


  Serial.println(F("INIT OLED"));
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.display();
  //delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();



  resetADE9153A();            //Reset ADE9153A for clean startup
  delay(1000);
  /*SPI initialization and test*/
  bool commscheck = ade9153A.SPI_Init(SPI_SPEED,CS_PIN); //Initialize SPI
  while (!commscheck) {
    Serial.println(F("ADE9153A Shield not detected. Plug in Shield and reset the Arduino"));
    
      delay(1000);
    }
    
  ade9153A.SetupADE9153A(); //Setup ADE9153A according to ADE9153AAPI.h
  /* Read and Print Specific Register using ADE9153A SPI Library */
  Serial.println(String(ade9153A.SPI_Read_32(REG_VERSION_PRODUCT), HEX)); // Version of IC
  ade9153A.SPI_Write_32(REG_AIGAIN, -268435456); //AIGAIN to -1 to account for IAP-IAN swap
  delay(500); 

  
}

void loop() {
  /* Main loop */
  /* Returns metrology to the serial monitor and waits for USER_INPUT button press to run autocal */
  unsigned long currentReport = millis();
  
  if ((currentReport - lastReport) >= reportInterval){
    lastReport = currentReport;
    readandwrite();
  }
  
  inputState = digitalRead(USER_INPUT);

  if (inputState == LOW) {
    Serial.println(F("Autocalibrating Current Channel"));
    ade9153A.StartAcal_AINormal();
    runLength(20);
    ade9153A.StopAcal();
    Serial.println(F("Autocalibrating Voltage Channel"));
    ade9153A.StartAcal_AV();
    runLength(40);
    ade9153A.StopAcal();
    delay(100);
    
    ade9153A.ReadAcalRegs(&acalVals);
    Serial.print(F("AICC: "));
    Serial.println(acalVals.AICC);
    Serial.print(F("AICERT: "));
    Serial.println(acalVals.AcalAICERTReg);
    Serial.print(F("AVCC: "));
    Serial.println(acalVals.AVCC);
    Serial.print(F("AVCERT: "));
    Serial.println(acalVals.AcalAVCERTReg);
    long Igain = (-(acalVals.AICC / 838.190) - 1) * 134217728;
    long Vgain = ((acalVals.AVCC / 13411.05) - 1) * 134217728;
    ade9153A.SPI_Write_32(REG_AIGAIN, Igain);
    ade9153A.SPI_Write_32(REG_AVGAIN, Vgain);
    
    Serial.println(F("Autocalibration Complete"));
    delay(2000);
  }
}

void readandwrite()
{ 
 /* Read and Print WATT Register using ADE9153A Read Library */
  ade9153A.ReadPowerRegs(&powerVals);    //Template to read Power registers from ADE9000 and store data in Arduino MCU
  ade9153A.ReadRMSRegs(&rmsVals);
  ade9153A.ReadPQRegs(&pqVals);
  ade9153A.ReadTemperature(&tempVal);

  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner

  
  display.print(F("RMS Current: "));
  display.print(rmsVals.CurrentRMSValue/1000);
  display.println(F(" A"));
  Serial.print(F("RMS Current:\t"));        
  Serial.print(rmsVals.CurrentRMSValue/1000); 
  Serial.println(F(" A"));

  display.print(F("RMS Voltage: "));
  display.print(rmsVals.VoltageRMSValue/1000);
  display.println(F(" V"));
  Serial.print(F("RMS Voltage:\t"));        
  Serial.print(rmsVals.VoltageRMSValue/1000);
  Serial.println(F(" V"));

  display.print(F("Power draw: "));
  display.print(powerVals.ActivePowerValue/1000);
  display.println(F(" W"));  
  Serial.print(F("Active Power:\t"));        
  Serial.print(powerVals.ActivePowerValue/1000);
  Serial.println(" W");

  display.print(F("R Power: "));
  display.print(powerVals.FundReactivePowerValue/1000);
  display.println(F(" VAR"));  
  Serial.print("Reactive Power:\t");        
  Serial.print(powerVals.FundReactivePowerValue/1000);
  Serial.println(F(" VAR"));

  display.print(F("A Power: "));
  display.print(powerVals.ApparentPowerValue/1000);
  display.println(F(" VA"));  
  Serial.print(F("Apparent Power:\t"));        
  Serial.print(powerVals.ApparentPowerValue/1000);
  Serial.println(F(" VA"));

  display.print(F("Power Factor: "));
  display.println(pqVals.PowerFactorValue);
  Serial.print(F("Power Factor:\t"));        
  Serial.println(pqVals.PowerFactorValue);

  display.print(F("Frequency: "));
  display.print(pqVals.FrequencyValue);
  display.println(F(" Hz"));  
  Serial.print(F("Frequency:\t"));        
  Serial.print(pqVals.FrequencyValue);
  Serial.println(F(" Hz"));
 
  Serial.print(F("Temperature:\t"));        
  Serial.print(tempVal.TemperatureVal);
  Serial.println(F(" degC"));

  Serial.println("");
  Serial.println("");
  display.display();
}

void resetADE9153A(void)
{
 digitalWrite(ADE9153A_RESET_PIN, LOW);
 delay(100);
 digitalWrite(ADE9153A_RESET_PIN, HIGH);
 delay(1000);
 Serial.println("Reset Done");
}

void runLength(long seconds)
{
  unsigned long startTime = millis();
  
  while (millis() - startTime < (seconds*1000)){
    digitalWrite(LED, HIGH);
    delay(blinkInterval);
    digitalWrite(LED, LOW);
    delay(blinkInterval);
  }  
}
