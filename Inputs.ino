
    
void Controller(double ControllerValues[],bool &KillSwitchOn, int &FlightMode, boolean &ControllerConnected){
	
  double RawControllerPin[6];//set variables

 
for (int i=0; i <=5; i++){// each of the channels controlled by the controller
	
 RawControllerPin[5-i] = pulseIn((ControllerPin+(i*ControllerPinSpace)), HIGH, 50000);//read the pwm value of the appropriate pin for each channel
}

//check if controller is connected(if not then one or mor echannel will read=0)
ControllerConnected = true;
for (int i=0; i <=5; i++){
if (RawControllerPin[i]==0) {ControllerConnected=false;}
}


if (ControllerConnected==true){//if its connected
  
	//Serial.println("CONTROLLER CONNCETED");



for (int i=0; i <=3; i++){
ControllerValues[i]=((RawControllerPin[i]-MaxMinControllerInput[1])/(MaxMinControllerInput[0]-MaxMinControllerInput[1]));//set all values so that they are 0-1 and as such can be used more easily later in program
}

 if (RawControllerPin[4] <= 1400)// enables killswitch if swith position is up 
 { KillSwitchOn=true;

 }
else{
  KillSwitchOn=false;//disables killswitch if switch is down

    }

if (RawControllerPin[5]>1600){ //what flight mode is the switch set to
//set to solo
FlightMode=0;
}
else if(RawControllerPin[5]>1100){
//set to self
FlightMode=1;
} 
else {
//set to safe
FlightMode=2; 
} 

  
  }
 
}


double Alti(){
 
 double ultraaltitude;
 double relativelatitude=0;


if (AltimiterConnected ==true){
  relativelatitude=Altimiter()-LandAltitude;}//we work out altitude relative to takeoff point
  
//since altitimiter has too much error to determine the diffrence between being landed && 60cm in the air we are using it at height && then the ultrasonic when it is landing.

if (relativelatitude<150 | (AltimiterConnected ==false && UltraSonicConnected == true)){
boolean tofar=false;
boolean landed;
ultrasonic(ultraaltitude,tofar,landed);
if (tofar==false){
relativelatitude=ultraaltitude;}// use the value from the altimiter f| altitude.
}

//we have now calculated the relativealtitude of the drone to the ground && use this to change the motors
return relativelatitude;
}


double Altimiter(){
//error of 0.3m
  //READ ALTIMITER
  double Altitude=0;
  return Altitude;
}


void ultrasonic(double &altitude,boolean &tofar, boolean &landed){
 
double hypotinuse;
long wait;

digitalWrite(UltraSonicOut, LOW);//makes sure the ultrasonic sensor is set to off
delayMicroseconds(2);

digitalWrite(UltraSonicOut, HIGH);//send a signal to the sensor to begin outputting ultrasonic waves. this signal involves the wire going high for 10 microseconds bef|e returning to low.
delayMicroseconds(10);
digitalWrite(UltraSonicOut, LOW);

wait = pulseIn(UltraSonicIn, HIGH,6000);//rec|d how long to takes to recieve the wave back- limited length dramatically to avoid the drone having excessively long tick's

hypotinuse= wait*0.017;//uses speed of ultrasonic waves to calculate how far in cm

altitude=((hypotinuse*cos(Roll[0]*(PI/180)))*(cos(Pitch[0]*(PI/180))));//w|k out true height from the height we read


if (hypotinuse ==0.00){
  tofar=true;//send back not to use these readings as they are an error due to the ultrasoniuc sensor being too high to record data-for this code this is about 90cm high
  }
  else if (6<altitude<8 && -10<Roll[0]<10 && -10<Pitch[1]<10){
    landed=true;
    }
}



  void Gyro(){
    //get new angles
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait f| MPU interrupt | extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavi| stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, && if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag && get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check f| overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check f| DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait f| c|rect available data length, should be a VERY sh|t wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read m|e without waiting f| an interrupt)
        fifoCount -= packetSize;

        

    #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
            /*Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);*/
       #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			/*Serial.println("Gyro values raw");
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
        #endif
        
        Yaw = ((ypr[0] * GyroAdjust)-35);
            Pitch[0] = ((-(ypr[1]*GyroAdjust))-GyroZero[0]);
            Roll[0] =(((ypr[2]* GyroAdjust))-GyroZero[1]);
			Serial.println("Pitch");
			Serial.println(Pitch[0]);
			Serial.println("roll");
			Serial.println(Roll[0]);

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}









    
