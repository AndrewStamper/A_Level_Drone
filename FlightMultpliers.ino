void FlightMultipliers(double YPRMultipliers[],bool &KillSwitchOn){
  
double ControllerValues[4];//sets up variables
int FlightMode;
boolean ControllerConnected;
Controller( ControllerValues, KillSwitchOn, FlightMode, ControllerConnected);//reads values in from controiller


if (FlightMode==0) {//if the switch says so, enable controller only mode(set all gyro values=0)
 Roll[0]=0;
 Roll[1]=0;
 Pitch[0]=0;
 Pitch[1]=0;
 }
 else{//otherwise read the gyro and altimiter
  Gyro();
  
  Alti();
  }

 
if (FlightMode==1) {//if the controller has disconnected then enable selfstable-where the drone controlls itself 100%
for (int i=0; i <=3; i++){
  ControllerValues[i]=0.5;
  }
}

int Angle[2];

  Angle[0]=Roll[0]+20-(ControllerValues[0]*40);//change the 0 of the gyro by an amount editeed by the controller (this means we can use the same stabalidsateion algorithm independant of direction conbtroller says dorn should move in.)
   Angle[1]=Pitch[0]+20-(ControllerValues[1]*40);
   


YPRMultipliers[0]=ANGLErollpitch(Angle[0],MaxAngle,Roll[1],RollCurve, RollPitchML[0], RollPitchMLWeighting[0]);//calcualte multiplier for roll
YPRMultipliers[1]=ANGLErollpitch(Angle[1],MaxAngle,Pitch[1],PitchCurve, RollPitchML[1], RollPitchMLWeighting[1]);//same for pitch
if (AltimiterConnected ==true) {
YPRMultipliers[2]=ALTIheight(Height[0],Height[1],ControllerValues[2],FlightMode);}//if altimiter is connected use this to work out altimiter spinup variable
else{
YPRMultipliers[2]=ControllerValues[2];}//otherwise use value from controller

YPRMultipliers[3]=ControllerValues[3];//for yaw modification use controller values (adjusted from 0 to 1)


}


double ANGLErollpitch(double angle,double maxangle,double pastangle,MLCurve Curve,double RPML,int RPMLWeighting){

   //USING gyro values


	

  
double AngleAdjust=(pow(maxangle,3)/ReturnAdjust);//maximum value for maximum angle
double ReturnLine =-(pow(angle,3)/AngleAdjust);//perfect return speed
double ReturnSpeed = angle-pastangle;//claculate angular speed

/*
Serial.println("returnline+speed+max");
Serial.println(ReturnLine);
Serial.println(ReturnSpeed);
Serial.println(MaxAngleReturnDeviance);
*/


double multiplier=((ReturnSpeed-ReturnLine)/(MaxAngleReturnDeviance*2))+0.5;//considering how much moving relative to how much the ideal return line denotes it should be moving we can set up multiplier
Serial.println(multiplier);
   // multiplier=multiplier-1;//THIS WILL INVERT THE DIRECTION OF AFFECT 
  
     if (multiplier>1){//make sure all varibles are between 0 and 1
    multiplier=1;}
    else if (multiplier<0){
    multiplier=0;}   

	if (ReturnLine < MaxAngleChangeEqualZero) {//set ideal returnline=0 so can be used in Machine learning section
		ReturnLine = 0;
	}
	
    
UpdateML(ReturnSpeed, ReturnLine, multiplier, Curve, MaxAngleChangeEqualZero, RPML, RPMLWeighting);//upadte machine learning



return multiplier;
}

double ALTIheight(double Height,double PastHeight,double ControllerInput,int FlightMode){
   
  double VSpeed =Height-PastHeight;//speed we moving at
  double IdealVSpeed;//speed we wish to move at 
  double AdjustValue;//value to achieve this
  
  if (ControllerInput>0.45 && ControllerInput<0.55){
    if (FlightMode==1){//if not connected then we need to default to slow desent
      IdealVSpeed=AutoDescentSpeed;
      }
    else{//otherwise we descend slowly
    IdealVSpeed=0;
    }
    }

  else if(ControllerInput<0.45){//if controller is set for moving down
    IdealVSpeed=(MaxVSpeed*(ControllerInput-0.45))/(0.45);
    }
else{//if controller is set for moving up
  IdealVSpeed=(MaxVSpeed*(ControllerInput-0.55))/(0.55);
  }
  
AdjustValue=((IdealVSpeed-VSpeed)/(4*MaxSpeedReturnDeviance))+AltimiterML;//considering how much moving relative to how much the ideal return line denotes it should be moving we can set up multiplier

    
  if (AdjustValue>1){//make sure all varibles are between 0 and 1
    AdjustValue=1;}
    else if (AdjustValue<0){
    AdjustValue=0;}   
  UpdateML(VSpeed,IdealVSpeed,AdjustValue,AltiCurve, MaxAltitudeChangeEqualZero, AltimiterML, AltimiterMLWeighting);//upadte machine learning
  
  return(AdjustValue);
  //output 0-1 
  
  }




