
#include <QTRSensors.h>

#define NUM_SENSORS   6     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs togo low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

/*Variables that may need to be changed*/

#define mazeWidth 35
#define basePwm 60
#define basePwmWall 40 //40
#define robotWidth 11.2
#define ticks90  6 //number of ticks to turn 90 degrees  //6 works fr maze
#define ticks10cm 20 //number of ticks to go forwards 10 cm



//----------------Define Variables-----------------//
int mode =0; //0 for line, 1 for maze without line, 2 for maze with line, 3 for line with obstacles
int sumLeft=0;
int sumRight=0;

const int pingPinLeft = 6; //SRF05 left
const int pingPinRight = 7; //SRF05 right
const int IRsensorpin = A2;                      // analog pin used to connect the sharp sensor
const int IRsamplingValue= 20;
const int encoderRight=A0;
const int encoderLeft=A1;
//values on cybertech floor
                                      
const float calibratedMax=1400;
const float calibratedMin=100;

//values on home floor
/*
const float calibratedMax=1400;
const float calibratedMin=100;
*/       
boolean insideMazeVar=false;
boolean LWhite=false;
boolean RWhite=false;
boolean outsideline=false;
boolean keepToLeftWall=true;
int turnNumber=1;
int cycles=0;
boolean countingCycles=false;
boolean startedOrNot=false;
QTRSensorsRC qtrrc((unsigned char[]) {15, 15, 10, 17, 18, 19}, // sensors 0 through 7 are connected to digital pins 3 through 10, respectively
  NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

//Motores//

const int motorDirA=12;  //left motor
const int breakA=9;
const int pwmA=3;
const int motorDirB=13;  //right motor
const int breakB=8;
const int pwmB=11;

// pid loop variables//
float error;
float lastError=0;
float lastPos;
float integral;
float PV;

float kp = 8;  //10 //8
float ki =0.015; //0.025 for 60 //0.04 for 80
float kd =8; //7.5  //5

float kpW=3; //4
float kdW=7; //3
 float kiW=0.01; //0.1
 int m1Speed, m2Speed;

boolean innerTrack=false;
boolean stuck=false;


//----------------Setup Variables-----------------//



void setup()
{
  //SRF05
  pinMode(pingPinLeft, OUTPUT);
  pinMode(pingPinRight, OUTPUT);
  pinMode(IRsensorpin, INPUT);
  
  //Motor Setup//
  pinMode(motorDirA, OUTPUT);
  pinMode(breakA, OUTPUT);
    
  pinMode(motorDirB, OUTPUT);
  pinMode(breakB, OUTPUT);
  
  pinMode(encoderRight,INPUT);
  pinMode(encoderLeft,INPUT);
  


  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);

}

//START FINAL WITH MODE 0


void loop() {
for(int k=0; k<10; k++){
  if(!insideMazeVar){
      checkIfInsideMaze();
      if(insideMazeVar){
      mode=1;
    
      }
  }


  
   digitalWrite(breakA, LOW);
   digitalWrite(breakB, LOW);
   
    startCheck();
       switch(mode)
   {
   case 0:
          if(startedOrNot){
              followLine();

          }
  break;
         
   case 1: 
          
        if(startedOrNot){
           followWall();

        }
  
           break;
           
   case 2:
       if(startedOrNot){
         followWallWithLine();
       }
  break;
case 3:

          if(startedOrNot){
              followLineWithObstacles();

          }


break;
 case 4:  
 if(getLeftEncoder()==1){
    Serial.println(getLeftEncoder());

 }
 // Serial.println(getRightEncoder());
 
 digitalWrite(breakA,LOW);
digitalWrite(breakB,LOW);


digitalWrite(motorDirA, HIGH);  // left motor
analogWrite(pwmA,basePwm);
digitalWrite(motorDirB,HIGH);
analogWrite(pwmB,basePwm);

 // Serial.println(getIRValue());
//Serial.println(getSRF05_right());
//Serial.println(getSRF05_left());
 break;
 
   } //
  sumLeft+=getLeftEncoder();
  sumRight+=getRightEncoder();
  
}

if(sumLeft==0 ||  sumRight==0){
getUnstuck();
}

 
} // void loop

float getLinePos(){
    qtrrc.read(sensorValues);
    float sum=0;
    float k=1;
    if(sensorValues[2]>calibratedMin){
      sum+=k*(-1.5f);
    }
    if(sensorValues[3]>calibratedMin){
      sum+=k*(-1.0f);
    }
    if(sensorValues[4]>calibratedMin){
      sum+=k*(1.0f);
    }
        if(sensorValues[5]>calibratedMin){
      sum+=k*(1.5f);
    }
    
    float pos=(sum);
    //to calibrate print sum
    if(sensorValues[2] >calibratedMin||sensorValues[3] >calibratedMin||sensorValues[4] >calibratedMin || sensorValues[5] >calibratedMin){
          lastPos=pos;
          outsideline=false;

    }else{
    outsideline=true;
      if(lastPos<0){
        lastPos=-1.0; 
      }else{
       lastPos=1.0;
      }//else
    }//else
    
    //Serial.println(lastPos);
    return lastPos;
}

/*----------- Follow Line-------------*/

void followLine(){

float positionLine=getLinePos();
       // Serial.println(positionLine);
if(getIRValue()>1){
 
error = positionLine; //gives an error between -1000 and 1000

int treshHold=2.5 ;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
if(abs(error)<treshHold){
 integral+=error; 
}else{
  integral=0;
}

PV = kp * error +ki*integral+ kd * (error - lastError);   //debe ser entero y a dentro del intervalo  baspwb 

lastError = error;


  
  if(PV>basePwm){
    PV=basePwm;
  }
  if(PV<-basePwm){
    PV=-basePwm;
  }
    
 if(abs(PV)>=5){ 
    
  int m1Speed = basePwm + (int)PV;  
  int m2Speed = basePwm - (int)PV;    


  if(!outsideline){
     digitalWrite(motorDirA, HIGH);  
     digitalWrite(motorDirB, LOW); 
  }else{
    if(error<0){
       digitalWrite(motorDirA, LOW);  
       digitalWrite(motorDirB, LOW); 
    }else{
       digitalWrite(motorDirA, HIGH);  
       digitalWrite(motorDirB, HIGH); 
    }

  }
     //set motor speeds
     analogWrite(pwmA, m1Speed);
     analogWrite(pwmB, m2Speed);
    



 }
       
}else{
    digitalWrite(breakA,HIGH);
  digitalWrite(breakB,HIGH);
  analogWrite(pwmB,0);
  analogWrite(pwmA, 0);
  delay(1000);
}

    
} // end follow line  
void followLineWithObstacles(){

float positionLine=getLinePos();
if(getIRValue()>18){
 
error = positionLine; //gives an error between -1000 and 1000

int treshHold=2.5 ;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
if(abs(error)<treshHold){
 integral+=error; 
}else{
  integral=0;
}

PV = kp * error +ki*integral+ kd * (error - lastError);   //debe ser entero y a dentro del intervalo  baspwb 

lastError = error;


  
  if(PV>basePwm){
    PV=basePwm;
  }
  if(PV<-basePwm){
    PV=-basePwm;
  }
    
 if(abs(PV)>=5){ 
    
  int m1Speed = basePwm + (int)PV;  
  int m2Speed = basePwm - (int)PV;    


  if(!outsideline){
     digitalWrite(motorDirA, HIGH);  
     digitalWrite(motorDirB, LOW); 
  }else{
    if(error<0){
       digitalWrite(motorDirA, LOW);  
       digitalWrite(motorDirB, LOW); 
   }else{
       digitalWrite(motorDirA, HIGH);  
       digitalWrite(motorDirB, HIGH); 
    }

  }
     //set motor speeds
     analogWrite(pwmA, m1Speed);
     analogWrite(pwmB, m2Speed);
    


 }
 

       
}else{
  digitalWrite(breakA,HIGH);
  digitalWrite(breakB,HIGH);
  analogWrite(pwmB,0);
  analogWrite(pwmA, 0);
  delay(500);     //500 for basePwm 60
  if(innerTrack){
    switchToRightTrack();
  }else{
    switchToLeftTrack();

  }
}
  

 
    
} // end follow line Obstacles 

void switchToRightTrack(){
  
digitalWrite(breakA,LOW);
digitalWrite(breakB,LOW);

for(int sumEncoder=0;sumEncoder<=(ticks90-1);sumEncoder+=getLeftEncoder()){  //for double the ticks of 90 degrees


digitalWrite(motorDirA, HIGH);  // left motor
analogWrite(pwmA,basePwm);
digitalWrite(motorDirB,HIGH);
analogWrite(pwmB,basePwm);

}
analogWrite(pwmA,0);
analogWrite(pwmB,0);

 digitalWrite(breakA,HIGH);
 digitalWrite(breakB,HIGH);

  driveForwardToLine();
  innerTrack=false;
}

///left track
void switchToLeftTrack(){
   
digitalWrite(breakA,LOW);
digitalWrite(breakB,LOW);

for(int sumEncoder=0;sumEncoder<=(ticks90-2);sumEncoder+=getRightEncoder()){  //for double the ticks of 90 degrees


digitalWrite(motorDirA, LOW);  // left motor
analogWrite(pwmA,basePwm);
digitalWrite(motorDirB,LOW);
analogWrite(pwmB,basePwm);

}
analogWrite(pwmA,0);
analogWrite(pwmB,0);

 digitalWrite(breakA,HIGH);
 digitalWrite(breakB,HIGH);
 driveForwardToLine();
 innerTrack=true;
  
}
void  driveForwardToLine(){
   digitalWrite(breakA,LOW);
 digitalWrite(breakB,LOW);
      digitalWrite(motorDirA, HIGH);  // left motor
    analogWrite(pwmA,basePwm);
    digitalWrite(motorDirB,LOW);
    analogWrite(pwmB,basePwm);
    delay(650);
  while((sensorValues[3]+sensorValues[4]) < 2*calibratedMin+1800){//calibratedMax*2-500
    qtrrc.read(sensorValues);
    digitalWrite(motorDirA, HIGH);  // left motor
    analogWrite(pwmA,basePwm);
    digitalWrite(motorDirB,LOW);
    analogWrite(pwmB,basePwm);
  }
  if(innerTrack){
    turnSharpLeft();
  }else{
    turnSharpRight();
  }

 digitalWrite(breakA,HIGH);
 digitalWrite(breakB,HIGH);
 analogWrite(pwmA,0);
 analogWrite(pwmB,0);


}



/*----------- Follow Wall-------------*/ 

void followWall(){


float integralWall=0;
PV=0;

 int distanceToWall_left= getSRF05_left();
int distanceToWall_right= getSRF05_right();

 float errorWall,lastErrorWall;
 
  if(keepToLeftWall){
         errorWall = distanceToWall_left-7;

   }else{
         errorWall = distanceToWall_right-10;
   }


 
   
int treshHold=30 ;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
if(abs(errorWall)<treshHold){
 integralWall+=errorWall; 
}else{
  integralWall=0;
}
 
 
  PV = kpW * errorWall +kiW*integral+ kdW * (errorWall - lastErrorWall);  
  lastErrorWall = errorWall;

  if(PV>basePwmWall){
    PV=basePwmWall;
  }
  if(PV<-basePwmWall){
    PV=-basePwmWall;
  } 
    
    int m1Speed = basePwmWall - (int)PV;  
    int m2Speed = basePwmWall + (int)PV;  
   
  
       
        //set motor speeds
         
         digitalWrite(motorDirA, HIGH); //left motor 
         analogWrite(pwmA, m1Speed);
          digitalWrite(motorDirB, LOW);  // right motor
         analogWrite(pwmB, m2Speed);
         
               
        
 if(getIRValue()<13){    
 
     if(getSRF05_right()>20){ //case
    
      turnSharpRight();
  
      }  
       
   if(getSRF05_left()<20 && getSRF05_right()<20){ //case dead end

   turnSharpRight();

    
   }
    
 }  


delay(10); 
 

} //end followWall

/*----------- Follow Wall With Line-------------*/ 

void followWallWithLine(){


float integralWall=0;
PV=0;

 int distanceToWall_left= getSRF05_left();
int distanceToWall_right= getSRF05_right();

 float errorWall,lastErrorWall;
 
  if(keepToLeftWall){
         errorWall = distanceToWall_left-8;

   }else{
         errorWall = distanceToWall_right-10;
   }


 
   
int treshHold=30 ;
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              
if(abs(errorWall)<treshHold){
 integralWall+=errorWall; 
}else{
  integralWall=0;
}
 
 
  PV = kpW * errorWall +kiW*integral+ kdW * (errorWall - lastErrorWall);  
  lastErrorWall = errorWall;

  if(PV>basePwmWall){
    PV=basePwmWall;
  }
  if(PV<-basePwmWall){
    PV=-basePwmWall;
  } 
    
    int m1Speed = basePwmWall - (int)PV;  
    int m2Speed = basePwmWall + (int)PV;  
   
  
       
        //set motor speeds
         
         digitalWrite(motorDirA, HIGH); //left motor 
         analogWrite(pwmA, m1Speed);
          digitalWrite(motorDirB, LOW);  // right motor
         analogWrite(pwmB, m2Speed);
         
               
        
 if(getIRValue()<13){    
 
     if(getSRF05_right()>20){ //case
    
      turnSharpRight();
  
      }  
       
   if(getSRF05_left()<20 && getSRF05_right()<20){ //case dead end

   turnSharpRight();

    
   }
    
 }  

 qtrrc.read(sensorValues);
if((sensorValues[3]+sensorValues[4]) > 2*calibratedMin+700){//calibratedMax*2-500
   
 
turn180(); 
 
    
}

delay(10); 
 

} //end followWallWithLine



/* --------Encoders---------*/

boolean getRightEncoder(){
  boolean temp=analogRead(encoderRight)>100;
  if(RWhite!=temp){
    RWhite=!RWhite;
      return true;
  }else{
    return false;
  }
  
}

boolean getLeftEncoder(){
  boolean temp=analogRead(encoderLeft)>100;
  if(LWhite!=temp){
    LWhite=!LWhite;
      return true;
  }else{
    return false;
  }
}
/*------------Driveforward------*/

void driveForward(int clicks){
  digitalWrite(motorDirA,HIGH); //left
  digitalWrite(motorDirB,LOW); //right
  digitalWrite(breakA,LOW);
  digitalWrite(breakB,LOW);
  analogWrite(pwmB,basePwmWall);
  analogWrite(pwmA, basePwmWall);
  for(int sumEncoder=0;sumEncoder<=(clicks);sumEncoder+=getLeftEncoder()){  //for double the ticks of 90 degrees


    digitalWrite(motorDirA, HIGH);  // left motor
    analogWrite(pwmA,basePwmWall);
    digitalWrite(motorDirB,LOW);
    analogWrite(pwmB,basePwmWall);

  }
    digitalWrite(breakA,LOW);
    digitalWrite(breakB,LOW);
    analogWrite(pwmA,0);
    analogWrite(pwmB,0);

}  
/*----------- Sharp Turns-------------*/

//____________turn left--------------//

void turnSharpLeft(){  

 

digitalWrite(breakB, HIGH); //stop right motor 
analogWrite(pwmB, 0);
digitalWrite(breakA, HIGH); //stop left motor 
analogWrite(pwmA, 0);
delay(50);
digitalWrite(breakA,LOW);
digitalWrite(breakB,LOW);

for(int sumEncoder=0;sumEncoder<=(ticks90-2);sumEncoder+=getLeftEncoder()){  //for double the ticks of 90 degrees


digitalWrite(motorDirA, LOW);  // left motor
analogWrite(pwmA,basePwm+25);
digitalWrite(motorDirB,LOW);
analogWrite(pwmB,basePwm+25);

}

delay(50);

 digitalWrite(breakA,HIGH);
 digitalWrite(breakB,HIGH);

}



//____________turn right--------------//
void turnSharpRight(){ // 

digitalWrite(breakB, HIGH); //stop right motor 
analogWrite(pwmB, 0);
digitalWrite(breakA, HIGH); //stop left motor 
analogWrite(pwmA, 0);
delay(50);
digitalWrite(breakA,LOW);
digitalWrite(breakB,LOW);

for(int sumEncoder=0;sumEncoder<=(1);sumEncoder+=getLeftEncoder()){  //for double the ticks of 90 degrees  //-2 for turning around objects


digitalWrite(motorDirA, HIGH);  // left motor
analogWrite(pwmA,basePwm+25);
digitalWrite(motorDirB,HIGH);
analogWrite(pwmB,basePwm+25);

}

 digitalWrite(breakA,HIGH);
 digitalWrite(breakB,HIGH);

}

//____________turn 180--------------//

void turn180(){

int sumEncoder;

digitalWrite(breakB, HIGH); //stop right motor 
analogWrite(pwmB, 0);
digitalWrite(breakA, HIGH); //stop left motor 
analogWrite(pwmA, 0);
delay(1000);
digitalWrite(breakA,LOW);
digitalWrite(breakB,LOW);

for(sumEncoder=0;sumEncoder<=(ticks90*2)+3;sumEncoder+=getLeftEncoder()){  //for double the ticks of 90 degrees

digitalWrite(motorDirA, HIGH);  // left motor
analogWrite(pwmA,basePwm+35);
digitalWrite(motorDirB,HIGH); //right motor
analogWrite(pwmB,basePwm+35);

}
 
 digitalWrite(breakA,HIGH);
 digitalWrite(breakB,HIGH);
 delay(700);
}







/*----------- drive straight forward until walls surrond the robot-------------*/

void checkIfInsideMaze(){
float mediumSRF05_right =0;
float mediumSRF05_left=0;
float sumSRF05_right, sumSRF05_left;


for(int i=0;i<10;i++){
sumSRF05_right+=sumSRF05_right;
sumSRF05_left+=sumSRF05_left;  
}
 
 
 mediumSRF05_right/=10;
 mediumSRF05_left/=10;

if(mediumSRF05_right<20 && mediumSRF05_right<20){

  insideMazeVar=true;
}else{
  insideMazeVar=false;
}


}



/*----------- Get SRF05 values-------------*/

int getSRF05_left() {
 
  long duration, inches, cmLeft;

  pinMode(pingPinLeft, OUTPUT);
  digitalWrite(pingPinLeft, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPinLeft, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPinLeft, LOW);

  
  pinMode(pingPinLeft, INPUT);
  duration = pulseIn(pingPinLeft, HIGH);


  cmLeft = microsecondsToCentimeters(duration);

  return cmLeft;
}
  
  int getSRF05_right() {
 
  long duration, inches, cmRight;

  pinMode(pingPinRight, OUTPUT);
  digitalWrite(pingPinRight, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPinRight, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPinRight, LOW);

  
  pinMode(pingPinRight, INPUT);
  duration = pulseIn(pingPinRight, HIGH);


  cmRight = microsecondsToCentimeters(duration);

  return cmRight;


}



/*----------- Get IR-Value-------------*/


int getIRValue(){
  int val = 0;                 // variable to store the values from sensor(initially zero)
  int distance = 0;           // variable to store the calculated distance from the sensor (initially zero)
  int meanValue= 0;

  // variable to store the values from sensor(initially zero)
  int tempSum = 0;
  for (int i=0;i<IRsamplingValue;i++){
    val = analogRead(IRsensorpin);    // reads the value of the sharp sensor
    distance = 4800/(val - 20);
    tempSum+=distance;
  }
  meanValue=tempSum/IRsamplingValue;
  return  abs(meanValue);
}

long microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
void startCheck(){
  if(!startedOrNot){
    if(getIRValue()>10){
      startedOrNot=true; 
    }
  }

}

boolean countIfStuck(){
  
  

  
return stuck;
}


void getUnstuck(){
  //drive backwards 
    digitalWrite(breakA,LOW);
    digitalWrite(breakB,LOW);
  for(int sumEncoder=0;sumEncoder<=(3);sumEncoder+=getLeftEncoder()){  //for double the ticks of 90 degrees


    digitalWrite(motorDirA, LOW);  // left motor
    analogWrite(pwmA,basePwmWall);
    digitalWrite(motorDirB,HIGH); //right motor
    analogWrite(pwmB,basePwmWall);
  }
    analogWrite(pwmA,0);
    analogWrite(pwmB,0);
    digitalWrite(breakA,HIGH);
    digitalWrite(breakB,HIGH);
    delay(10);
  
}
  
  
  
  
  

