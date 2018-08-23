/*
 * Copyright (c) 2016 Intel Corporation.  All rights reserved.
 * See the bottom of this file for the license terms.
 */

/*
   This sketch example demonstrates how the BMI160 on the
   Intel(R) Curie(TM) module can be used to read accelerometer data
*/

#include "CurieIMU.h"
#include <Servo.h>
#include <CurieBLE.h>

BLEService batteryService("180F"); // BLE Battery Service
BLEService monitorService("4ca04ea0-380a-4aa9-af51-a0d958c28099"); //custom 120-bit UUID for custom Service
BLEService motorService("ebbb19a6-c943-44f9-aee0-e180300007f0"); //custom 120-bit UUID custom service
 
// BLE Battery Level Characteristic"
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",  // standard 16-bit characteristic UUID
    BLERead | BLENotify);     // remote clients will be able to
// get notifications if this characteristic changes

//motor characteristic
BLEIntCharacteristic motorReading("00211321-dc03-4f55-82b6-14a630bd8e2d",
BLERead|BLENotify|BLEWrite);

//motor properties
BLEIntCharacteristic motorExtendChar("809ba7a9-13ad-4446-a005-bdc12ca93c76", BLERead|BLENotify|BLEWrite);
BLEIntCharacteristic motorContractChar("2fad8a3f-e1a1-47cd-982b-24e13fbe9342", BLERead|BLENotify|BLEWrite);

int oldBatteryLevel = 0;  // last battery level reading from analog input
long previousMillis = 0;  // last time the battery level was checked, in ms

Servo myservoflex;  // create servo object to control a servo
Servo myservoextend;  // create servo object to control a servo
// twelve servo objects can be created on most boards

float timeMillis=0;
float timeMillisMotionReset=0;
float timeMillisGyroReset=0;
float ax, ay, az, atot, gx, gy, gz, gtot, gx_bias, gy_bias, gz_bias, gtot_filt;
int motionDetect=0;
int switchPosition=0; // fingers extended
int autoButton=0;
int autoButtonOld=0;
int extendButton=0;
int flexButton=1;
int extendButtonOld=0;
int extendButtonChange=0;
int delayMode=0;
int timeMillisDelayMode=0;
int extendMotor=50;//50 = fully squeeze bottle; 80 = partially squeeze bottle
int retractMotor=150;//150 = fully extend fingers; 120 = partially extend fingers

int manualExtend = 0;
int manualContract = 1;
int manualRelax = -1;
int test=0;

void setup() {
  Serial.begin(9600);    // initialize serial communication
  pinMode(13, OUTPUT);   // initialize the LED on pin 13 to indicate when a central is connected

  //initalize eventListeners
  //motorReading.setEventHandler(BLEWritten, switchCharacteristicWritten);
  // begin initialization
  BLE.begin();

  BLE.setLocalName("LegoHERO");
  
  BLE.setAdvertisedService(batteryService);  // add the service UUID
  batteryService.addCharacteristic(batteryLevelChar); // add the battery level characteristic
  BLE.addService(batteryService);   // Add the BLE Battery service
  batteryLevelChar.setValue(oldBatteryLevel);   // initial value for this characteristic
  
  BLE.setAdvertisedService(motorService);  // add the service UUID
  motorService.addCharacteristic(motorReading); 
  motorService.addCharacteristic(motorExtendChar);
  motorService.addCharacteristic(motorContractChar);
  BLE.addService(motorService);   // Add the BLE Battery service


  motorReading.setValue(manualRelax);
  motorExtendChar.setValue(extendMotor);
  motorContractChar.setValue(retractMotor);

  extendMotor = motorExtendChar.value();
  retractMotor = motorContractChar.value();
  
  /* Start advertising BLE.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();
  
  myservoflex.attach(A1);  // attaches the servo
  myservoextend.attach(A0);  // attaches the servo 
  //myservoflex.write(40); 
  //myservoextend.write(40); 
  //Serial.begin(9600); // initialize Serial communication
  //while (!Serial);    // wait for the serial port to open
  timeMillis=millis();
  timeMillisMotionReset=millis();  
  pinMode(9, OUTPUT);
  digitalWrite(9,LOW);
  pinMode(10, INPUT); // set pin to input
  pinMode(8, INPUT);
  pinMode(7, INPUT);           
  digitalWrite(10, HIGH);       // turn on pullup resistors meaning pin will be HIGH and pin will only go LOW when connected to the switch's control pin (e.g. pin 9)
  digitalWrite(8, HIGH);       // turn on pullup resistors
  digitalWrite(7, HIGH);       // turn on pullup resistors
  
  CurieIMU.begin();
  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the accelerometer range to 250 degrees/second
  CurieIMU.setGyroRange(250);

 //read buttons
  extendButton=digitalRead(10);
  autoButton=digitalRead(8);
  flexButton=digitalRead(7);

  // if auto button is pressed down we are in manual mode, start manual mode with motors relaxed
  if (autoButton==LOW)
  {motorReading.setValue(manualRelax);}

  // if auto button is pressed up we are in auto mode, start auto mode with fingers extended
  else if (autoButton==HIGH)
  {
    extendMotor = 80;
    retractMotor = 120;
    myservoflex.write(retractMotor);  
    myservoextend.write(extendMotor);
  delayMode=1;
  timeMillisDelayMode=millis();}

    
}

void loop() {
  //
  //on loop reset the motor values
  extendMotor = motorExtendChar.value();
  retractMotor = motorContractChar.value();
  
  // read accelerometer measurements from device, scaled to the configured range
  CurieIMU.readAccelerometerScaled(ax, ay, az);
  CurieIMU.readGyroScaled(gx, gy, gz);

  //calculate absolute value of gyro
  gtot=sqrt(sq(gx-gx_bias)+sq(gy-gy_bias)+sq(gz-gz_bias));
  //filter gtot so quick noise is not detected as motion
  gtot_filt=gtot*0.2 + gtot_filt*0.8;

//  open();
//  TestX.write(gx-gx_bias);
//  TestY.write(gy-gy_bias);
//  TestZ.write(gz-gz_bias);
//  Serial.println(gx-gx_bias);
  //Serial.print("\t");
  //Serial.print(gy-gy_bias);
  //Serial.print("\t");
  //Serial.println(gz-gz_bias);
  //Serial.print("\t");
  //Serial.println(motionDetect);
  //Serial.println(gtot);

  //read time
  timeMillis=millis();
   
  //read buttons
  autoButton=digitalRead(8);
  extendButton=digitalRead(10);
  flexButton=digitalRead(7);


//detect if extend button is pressed while robot is in manual mode
  if ((autoButton==HIGH)&&(extendButton!=extendButtonOld)){
    extendButtonOld=extendButton;
    extendButtonChange=1;
  }
  
  //if extend button is pressed while robot is in manual mode then change characteristic values if extend button is pressed up or flexion if extend button pressed down 
  if(extendButton==LOW &&(extendButtonChange==1) && autoButton == HIGH ){
    /*
    myservoflex.write(extendMotor);
    myservoextend.write(retractMotor);
    */
    motorReading.setValue(manualExtend);
    extendButtonChange=0;
    //Serial.println("extend");
  }

  else if(flexButton==LOW &&(extendButtonChange==1) && autoButton == HIGH ){
    /*
    myservoflex.write(retractMotor);  
    myservoextend.write(extendMotor);
    */
    motorReading.setValue(manualContract);
    extendButtonChange=0;
  }

  //update motors based off of the new characteristic values presented
  if ( motorReading.value() == manualExtend && autoButton == HIGH){
     myservoflex.write(extendMotor);
    myservoextend.write(retractMotor);
    Serial.println("ManExtend");
  }

  if ( motorReading.value() == manualContract&& autoButton == HIGH){
    myservoflex.write(retractMotor);  
    myservoextend.write(extendMotor);
    Serial.println("ManFLex");
  }

  if ( motorReading.value() == manualRelax&& autoButton == HIGH){
    extendMotor = 80;
    retractMotor = 120;
    myservoflex.write(retractMotor);  
    myservoextend.write(extendMotor);
  }
  
  //reset gyro biases when user is not moving, so that when the user is at rest gtot is 0
  if((gtot<5)&&(timeMillis>timeMillisGyroReset+20000)){
    gx_bias=gx;
    gy_bias=gy;
    gz_bias=gz;
    timeMillisGyroReset=millis();
  }

//delay mode, don't trigger robot to move

//
if ((autoButton==LOW)&&(delayMode==1)&&(timeMillis<(timeMillisDelayMode+2000)))
{motionDetect=0;}


else{
  
  delayMode=0;
  //detect if auto button has been pressed; autoButton=0 is auto mode (button up), autoButton=1 is manual mode (button down)
  if (autoButton!=autoButtonOld){
    // if auto button is pressed down we are in manual mode, start manual mode with motors relaxed
    if (autoButton==LOW)
    {motorReading.setValue(manualRelax);}

    // if auto button is pressed up we are in auto mode, start auto mode with fingers extended
    
    if (autoButton==LOW)
    {
      extendMotor = 80;
      retractMotor = 120;
     myservoflex.write(retractMotor);  
    myservoextend.write(extendMotor);
    switchPosition=0;
    delayMode=1;
    timeMillisDelayMode=millis();}

    autoButtonOld=autoButton;
    extendButtonOld=extendButton;
    motionDetect=0;
    //delay(7000);
  }
  
  //detect if robot is in auto mode
  if (autoButton==LOW){
    //detect if angular motion is greater than 15 units. If so, keep resetting the reset counter while the user is moving
    if (gtot>30){
       motionDetect=1;
       timeMillisMotionReset=millis();
       Serial.println("autoTilMoveStopped");
    }
    
    //after moving, if the user isd stationary for long enough, trigger motor to move
    if ((motionDetect==1)&&(timeMillis>(timeMillisMotionReset+800))){
       if (switchPosition==1){
       myservoflex.write(retractMotor);
       myservoextend.write(extendMotor); 
       switchPosition=0;
       Serial.println("AutoFlex");
       }
       else{
       myservoflex.write(extendMotor);
       myservoextend.write(retractMotor); 
       switchPosition=1;
       Serial.println("AutoEx");
       }
       delayMode=1;
       timeMillisDelayMode=millis();
       Serial.println("AutoMode");
    }
  }

  //detect if extend button is pressed while robot is in manual mode
  
//  if ((autoButton==HIGH)&&(extendButton == LOW)){
//    extendButtonOld=extendButton;
//    extendButtonChange=1;
//  }
  

  //manual app control will work only if not in auto mode on the robot

  
}//else for delay mode

//updating battery info
// listen for BLE peripherals to connect:
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    //Serial.print("Connected to central: ");
    // print the central's MAC address:
   // Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(13, HIGH);

    // check the battery level every 200ms
    // as long as the central is still connected:
    if (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed, check the battery level:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
       //M updateBatteryLevel();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(13, LOW);
    //Serial.print("Disconnected from central: ");
    //Serial.println(central.address());
  }
  
}//end of void loop

void updateBatteryLevel() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the charge level of a battery.
  */
  int battery = analogRead(A0);
  int batteryLevel = map(battery, 0, 1023, 0, 100);

  if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
    Serial.print("Battery Level % is now: "); // print it
    Serial.println(batteryLevel);
    batteryLevelChar.setValue(batteryLevel);  // and update the battery level characteristic
    oldBatteryLevel = batteryLevel;           // save the level for next comparison
  }
}


/*
   Copyright (c) 2016 Intel Corporation.  All rights reserved.

   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

