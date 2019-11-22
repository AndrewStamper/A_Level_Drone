void MotorWright(double YPRMultipliers[]){
int AlwaysOn;
int MotorValue[8];
  
if (MotorSetup==true){//the dorne allows the user to use this to configure the Electronic speed controllers auttomatically(only needs to be done once)
IntialEngineSetup();
}


AlwaysOn=SpinupSpeed-(0.5*(1-AltiAngleEngineSplit)*ControllSplit*MaxMinEngineValues[0]);//sets the minimum motor values so that if you are flying and pull the left trigger to 0 the drone doesnt cut out, but instead has just enough power to maintain some flight whilst it lands.

if ((AlwaysOn+(0.5*(1-AltiAngleEngineSplit)*ControllSplit*MaxMinEngineValues[0])+0.1*MaxMinEngineValues[0])>LaunchSpeed){//if values are too large then the dorn will continually try to fly up and wont ever be able to come down so this is accounted for and the spinup is aborted
	Serial.println("controllsplit too high will cause accident");
  loop();
  
}


// -     take the multipliers and use them to form the appropriate motor values 

//MINIMUM MOTORS SPEED WHEN WRIGHT 105
//TAKEOFF AT ABOUT 160
//250=max


for (int motor=0; motor <MotorCount; motor++){//for each motor
  double mypr;
  double mpitch;
  double mroll;
  double myaw;


//dependant on if motor is affected positively or negatively the individyual motors roll and putch multipliers are = to the drones roll/pitch multipliers or = 1-(the drones roll/pitch multipliers)
if (pitchlookup(motor,MotorCount)==1){
  mpitch=YPRMultipliers[1];}
else if (pitchlookup(motor,MotorCount)==2){
  mpitch=0.5;}
else{
  mpitch=1-YPRMultipliers[1];}

if (rolllookup(motor,MotorCount)==1){
  mroll=YPRMultipliers[0];}
else if (rolllookup(motor,MotorCount)==2){
  mroll=0.5;}
else{
mroll=1-YPRMultipliers[0];}


//if the motor is spinning clockwise or anticlockwise it is affected positively or negatively by the yaw mulktiplier and this loop works out which it is.
if ((motor+1) % 2 == 0){
  myaw=(1-YPRMultipliers[3]);
}
else{
  myaw=YPRMultipliers[3];
}

  mypr=(mpitch+mroll+myaw)/3;//the individual motors multiplier for its yaw, roll and pitch is caluclated here


double altimiteraffect;

if (double altitude=Alti()>10000 && AltimiterConnected ==true){//LIMIT ALTITUDE OF DRONE to 120m(for legal resaons (122 but we are shooting sligtly less so as to account for errors.))
altimiteraffect=(1-(pow(2,((altitude-10000)/1000))/4));}//will form an exponential grpah of how much it will reduce motors by as you exceede the maximum height.
else{
  altimiteraffect=1;
  }

  
int angleadjust=(((1-AltiAngleEngineSplit)*(ControllSplit*MaxMinEngineValues[0]))*mypr);//adjuster per motor for angle adjustment

int altiadjust=((AltiAngleEngineSplit*(ControllSplit*MaxMinEngineValues[0]))*abs(YPRMultipliers[2]));//adjuster per motor for altitude adjustment

MotorValue[motor]=((AlwaysOn+altiadjust+angleadjust)*altimiteraffect);//sets overall affect per motor
    }


//then wright to each motor
for (int motor=0; motor <MotorCount; motor++){
mservo[motor].write(MotorValue[motor]);

Serial.println("Motor:");
Serial.print(motor);
Serial.print("Value");
Serial.print((MotorValue[motor]));
    }

  
  }


  
void IntialEngineSetup(){
for (int motor=0; motor <MotorCount; motor++){
mservo[motor].write(MaxMinEngineValues[0]);}//spin all at max
delay(10000);//waits 10 secomds 
for (int motor=0; motor <MotorCount; motor++){
mservo[motor].write(MaxMinEngineValues[1]);}//spin all at minimum
delay(5000);//waits 5 secomds 
  }
