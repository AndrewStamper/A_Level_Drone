void UpdateML(double Speed,double &IdealSpeed,double &AdjustValue, MLCurve &Curve,double &MaxChangeEqualZero,double &ExistingValue,int &MLWeighting){
  boolean CurrentDirection=DirectionCalc(Speed);//work out current direction
  
  if(IdealSpeed==0 && Curve.TicksSinceLastMove>5 && Speed<MaxChangeEqualZero && Speed>-MaxChangeEqualZero && Curve.Recording==false){//starts the recording of this curve
    Curve.Direction=CurrentDirection;//sets direction
  Curve.Recording=true;//sets recording=true(so cant try and restart half way through)
  Curve.ArrayAdd(AdjustValue);//adds first value
  }
else if (Curve.Direction != CurrentDirection && Curve.Recording==true ){//if curve has completed and we need to update running the value
 double NewAdjustValue= Curve.CalculateNewAdjuster();

NewAdjustValue=(NewAdjustValue- ExistingValue)/(MLWeighting);//finds diffrence and then chnages this by the weighting oif the affects

ExistingValue +=NewAdjustValue;//add this to the value

MLWeighting+=1;//increase the mlweighting which denotes how much affect each new curve has(if many have been recorded then we must be close to ideal so need small improvements)

Curve.NextFreeMultiplierArray=0;//reset values
Curve.Recording==false;
  }
  
else if(Curve.Recording==true){//record
  Curve.ArrayAdd(AdjustValue);
  }
  
else if(Curve.Recording==false && Curve.NotMoving==true){//if counting ticks for ready to start
  if(IdealSpeed==0){//if still counting
  Curve.IncrementTicksSinceMove();
  }
  else if ( IdealSpeed!=0){//if we stop counting
    Curve.NotMoving=false;
    Curve.TicksSinceLastMove=0;
    }
}
else if (Curve.Recording==false && IdealSpeed==0 ){
  Curve.NotMoving=true;
  }


//thought about looking at past spin+affect of that but decided it woulnt work as it is too quick to react && as such would cause issues with wind etc
  }

boolean DirectionCalc(int Speed){//takes the speed and converts into a boolean direction
  if (Speed>0){
    return false;
    }
    else{
      return true;}
  }


void EEPROMupdate(){//update saved values
  
 
 EEPROM.update(0, AltimiterML);
  EEPROM.update(8, AltimiterMLWeighting);//must be spaced at gap of 8 to account for the use of doubles which use multiple bytes

  EEPROM.update(16, RollPitchML[0]);
  EEPROM.update(24, RollPitchML[1]);

  EEPROM.update(32, RollPitchMLWeighting[0]);
  EEPROM.update(40, RollPitchMLWeighting[1]);

  
  }

void EEPROMGget() {//read in saved values


	EEPROM.get(0, AltimiterML);
	EEPROM.get(8, AltimiterMLWeighting);//must be spaced at gap of 8 to account for the use of doubles which use multiple bytes

	EEPROM.get(16, RollPitchML[0]);
	EEPROM.get(24, RollPitchML[1]);

	EEPROM.get(32, RollPitchMLWeighting[0]);
	EEPROM.get(40, RollPitchMLWeighting[1]);


}


