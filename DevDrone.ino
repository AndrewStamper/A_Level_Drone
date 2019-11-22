
/*
	This section is the section which acts to link all of the diffrent peicees of the project together
	Firstly we have include librarys and the class ml.h which i have set previously.
	Next we have declarations of global variables which can be altered to afect how the drone flys.
	Next we have some setup systems for the gyroscope and other global variable declaration.

*/


#include <Servo.h>
#include <EEPROM.h>
#include "ML.h"
// ================================================================
// ===                      ADJUSTERS                           ===
// ================================================================

//Engine
double AltiAngleEngineSplit=0.4;             //This configure how much engine power is reserved for altitude correction and as such 1-this gives the amount resertved for angle
double ControllSplit=0.6;                    //This configures how much engine power is to be used by he stabalistaion algorithm- rest is set appropriately so as to keep motors spiun up

//ReturnCalculation
int MaxAngle=45;                             //maximum angle the drone will ecah before engine multipliers max.
double ReturnAdjust=20;                       //max ideal speed opf return
double MaxAngleReturnDeviance=35;            //value to denote point at which multipliers max out from being too far from perfect return line measured in degree/tick
double MaxAngleChangeEqualZero = 10;		//max amount the drone can move and the machine learning can start, for best results from ml we need to be close to zero, this dictates how close is enough

//VericalSpeedCalculation
double MaxVSpeed=100;                          //maximum vertical speed before engine multipliers max.
double MaxSpeedReturnDeviance=35;            //value to denote point at which multipliers max out from being too far from perfect return line measured in cm/tick
double AutoDescentSpeed=5;                  //the speed the drone will automatically descend at if controller disconects measured in cm/tick
double BaseAltimiterSpinUpValue=0.5;         //the base value which we use to start with for the drone being held stationary in the air ---EEPROM ONCE EDITED=LOCATION 0---
double MaxAltitudeChangeEqualZero=10;		//max amount the drone can move and the machine learning can start, for best results from ml we need to be close to zero, this dictates how close is enough


//IO
int MotorCount=4;                            //this is the number of engines connected to the drone at this time
int MaxMinEngineValues[2]={256,0};           //This determines max/min engine values
int MaxMinControllerInput[2]={2000,1000};    //This determines max/min controller input
boolean MotorSetup=false;                    //This configures SimonK esc's max and min engine values when set to true --REMOVE BLADES BEFORE USING THIS--
int LaunchSpeed=190;                         //configures at what motor speed the drone lifts off
int SpinupSpeed=145;                         //configures at what esc value the motors will spinup
double GyroAdjust=57.29;                     //determines multiplier required to take gyro output into degrees. if gyro reads in radiants this is 57.29
double AltiAdjust=100;                       //determines multiplier required to take altimiter output into meters. if gyro reads in cm this is 0.01
int MotorPin=4;                              //pin location of the first motor(next motor is this location+MotorPinSpace)
int MotorPinSpace=1;
int ControllerPin=42;                        //pin location of the first controller pin(next pin is this location+ControllerPinSpace)
int ControllerPinSpace=2;
boolean AltimiterConnected = false;          //is there an altimniter connected
boolean UltraSonicConnected = true;          //is there an ultrasonic sensor connected
int AltimiterPin=99;                         //pin location of the first data pin to altimiter
int AltimiterPinSpace=1;                     //seperation of altimiter pins
int UltraSonicOut=30;                        //pin location of ultrasonic ouput to sensor
int UltraSonicIn=32 ;                        //pin location of ultrasonic input to arduino
int EEPROMCount=20;                          //How many loops between updatibg the non volatile copy of the multipliers. if too many then wears the chipset, if too few then doesnt log enough.




// ================================================================
// ===                      GYRO PRE-SETUP                      ===
// ================================================================

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                           SETUP                          ===
// ================================================================

Servo mservo[7];
String Output;
int Yaw;
int Roll[2];
int Pitch[2];
int Height[2];
int GyroZero[2];
int LandAltitude;
double AltimiterML;
int AltimiterMLWeighting;
double RollPitchML[2];
int RollPitchMLWeighting[2];
int EEPROMUpdateCounter;


  
MLCurve  AltiCurve=MLCurve(0.5);//create instances of the class for each use case(altitude,pitch and roll)
MLCurve  PitchCurve=MLCurve(0.5);
MLCurve  RollCurve=MLCurve(0.5);


void setup() {

	/*
	    In setup() the drone configures its systems, recording the values of eeprom into global variables 
	moreover setup also configures the pinout of the drone and connects the gyro and altimiter(if connected)
	Setup() is run only once at power up and then the  loop() of the drone takes over and this controlls the drone.
	*/
	Serial.begin(14400);//setup connection baud rate to pc
double RollPitchML=(0.5,0.5);


//check if values have been saved in epprom memory, if they have read them in and save into global variables, if not then set values into eeprom and continue
EEPROMGget();
 
if (AltimiterML==0){
  EEPROM.put(0, BaseAltimiterSpinUpValue);
  EEPROM.update(8,300);

  EEPROM.update(16, 0.5);
  EEPROM.update(24, 0.5);

  EEPROM.update(32, 300);
  EEPROM.update(40, 300);


  EEPROMGget();

  }



GyroSetup();//setup gyro
Serial.println("Gyro Configured");
PinoutSetup();//setup pinnout(motors, ultrasonic sensor, controller etc.)
Serial.println("Data conections started");
AltimiterSetup();//setup altimiter
Serial.println("Altimiter Connected");

boolean ControllerConnected = true;
do{ //loop untill the controller has been connected so that the Drone doesnt start till controller has connected.
  double ControllerValues[4];// since i use passing by refrence i need to declare these values so as to run the controller() subroutine
bool KillSwitchOn;
int FlightMode;
 ControllerConnected=true;
 Controller( ControllerValues, KillSwitchOn, FlightMode, ControllerConnected);
  Serial.println("failed to connect to controller");
  Landed();//update variables such as altitude of ground, angle of ground etc.
}while (ControllerConnected==false);
Serial.println("setup complete");
}

  
void loop() {
  // put your main code here, to run repeatedly:
bool KillSwitchOn;
double YPRMultipliers[4];

FlightMultipliers(YPRMultipliers,KillSwitchOn);//calcualte the flight multipliers
/*
Serial.println("YPR1,2,3,4");
Serial.println(YPRMultipliers[0]);
Serial.println(YPRMultipliers[1]);
Serial.println(YPRMultipliers[2]);
Serial.println(YPRMultipliers[3]);
Serial.println("End ypr");

Serial.println();
Serial.println();
*/

if (KillSwitchOn==false) {//if the killswitch is off, then send the multiupliers to the mwright subroutine
	
MotorWright(YPRMultipliers);// pass over the variables for motor values
}
else{
	Serial.print("Kill enabled so does not send to motors");
for (int motor=0; motor <MotorCount; motor++){//send values=0 to the motors to shut them down.
mservo[motor].write(0);
}
Landed();
}

UpdateVariables();
}

void SerialOutput(){// all data is sent via one large serial output so as to minimise the affect of this adat transfer on the frequency of the refresh of the drone.
	
  Serial.print(Output);
  Output="";
  Serial.println("--------------------------------------------------------------------------------------------------------:");
  }


void Landed(){//update altitude of ground and also the gyrozero values which set the drone so it can understand precisely the angle which is flat, else if the gyro is mounted incorrectly this can cause errors.
Gyro();
Alti();
GyroZero[0] = Pitch[0] + GyroZero[0];
GyroZero[1] = Roll[0]+GyroZero[1];
  LandAltitude= Height[0];
  }


void UpdateVariables(){// due to the fianite number of times you can save to nand fash gates i only save once in every EEPROMCount number of loops to preserve life of memory
  
if (EEPROMUpdateCounter==EEPROMCount){
  EEPROMupdate();//update
  EEPROMUpdateCounter=0;
  }
else{
  EEPROMUpdateCounter+=1;//wait 
  }

  
  SerialOutput();
Roll[1]=Roll[0];//value[1] denotes the previous value so it can be used to calualte angular or verical velocity in the next loop of the program
Pitch[1]=Pitch[0];
Height[1]=Height[0];

}





  
