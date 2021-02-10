#include<math.h>
#define a 10 //ARRAY LENGHT

//uptdate positions
double distancia1=0;
double distancia2=0;
double longitud=8;
double angleSensor;


// CURRENT AND DESIRED POSE
double x=0;
double y=0;
double theta=PI/4;


//WHEELS VARIABLES 
double radius=3.4;
double lenght=10.6;


void setup() {
  // put your setup code here, to run once:
  pinMode(16,OUTPUT);
  pinMode(17,INPUT);
  pinMode(18,OUTPUT);
  pinMode(19,INPUT);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  
//TRIGGER AND ECHO
  digitalWrite(16,1);
  delayMicroseconds(10);
  digitalWrite(16,0);
  double tiempo1=pulseIn(17,1);
  
  digitalWrite(18,1);
  delayMicroseconds(10);
  digitalWrite(18,0);
  double tiempo2=pulseIn(19,1);
  
  distancia1=340.0*tiempo1/(2*10000);  
  distancia2=340.0*tiempo2/(2*10000);

  if(distancia1>30) distancia1=0;
  if(distancia2>30) distancia2=0;
  
  updatePositionObstacles(distancia1,-PI/4);
  updatePositionObstacles(distancia2,0);
}

void updatePositionObstacles(double distancia,double angleSensor){
  double obstacle_x=(x+(longitud+distancia)*cos(angleSensor+theta));
  double obstacle_y=(y+(longitud+distancia)*sin(angleSensor+theta));
  if(angleSensor==-PI/4) Serial.print("OBSTACLE_1: ");
  else if(angleSensor==0) Serial.print("OBSTACLE_2: ");
  else Serial.print("OBSTACLE_3: ");
  
  if(distancia!=0){
    Serial.print("(");
    Serial.print(obstacle_x);
    Serial.print(",");
    Serial.print(obstacle_y);
    Serial.println(")");
  }
  else Serial.println("OBJETO NO DETECTADO");

}
