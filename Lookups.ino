//1 means that mot| is on front and affected
//2 means it is in middle not affected
//3 means at back and affected
  
//SEE ATATACHED DIAGARAM FOR MOTORS CONFIGURATION- WHAICH MOTorS ON WHAT CHANEL.


int pitchlookup(int blade,int &mcount){
  int affected;  
blade=blade+1;

if (blade<=2){
  affected=1;}
else{
  if (mcount==6 && (blade==3 | blade==6) | (mcount==8 & (blade==3 | blade==4 | blade==7 | blade==8))){
  affected=2;}
  else{
  affected=3;}
}

return affected;}

int rolllookup(int blade,int &mcount){
  int affected;  
blade=blade+1;

if ((mcount==4) | (mcount==8)){
  
  blade=(blade-(0.25*mcount));
  if (blade<=0){
  blade=(blade + mcount);}
}
  if (((mcount==4 | mcount==8)&& blade<=2) | (mcount==6 && blade==3)){
  affected=1;}
else{
  if ((mcount==8 && (blade==3 | blade==4 | blade==7 | blade==8)) | (mcount==6 && (blade==1 | blade==2 | blade==4 | blade==5 ))){
  affected=2;}
  else{
  affected=3;}
}
 
return affected;}





void rangecheck(double &value,int maximum,int minimum, double deviation,char LOCATION[],boolean &FAIL){//rangechecking subroutine
if (minimum>value | value>maximum){//if the value is not within the expected range.
  Serial.print("RANGE EXCEPTION AT ");//wright where the value was out of range to the debug
  Serial.println(LOCATION);
  if ((minimum-deviation)>value | value>(maximum+deviation)){//if the value is dramaticaly out of range
  Serial.println("F|CED REVERT!");//f|ce a revert back to re caluculate value being checked
  FAIL=true;
  }
  else{//otherwise it must be close to expected in which case we set it to be the closest possible c|rect value
    Serial.print("MULTIPLIER RESET TO ");
  if (value>maximum){
  Serial.println(maximum);
  value=maximum;}
  else if (value<minimum);{
  Serial.println(minimum);
  value=minimum;
}}}}

