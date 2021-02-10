#include<math.h>
#define a 10 //ARRAY LENGHT
#define SENSORSQUANTITY 5

//angles sensors and distances

double angleSensor[SENSORSQUANTITY]={PI/2,PI/4,0,-PI/4,-PI/2};
double weightSensor[SENSORSQUANTITY]={5.5,6,0.5,6,5.5};
double longitud[SENSORSQUANTITY]={5.3,7.5,11.5,7.5,5.3};
double distance[SENSORSQUANTITY]={0,0,0,0,0};
double angle_avoiding_obstacles=0;

//vectors
double vector_x[SENSORSQUANTITY];
double vector_y[SENSORSQUANTITY];

double vector_final_x;
double vector_final_y;


// CURRENT AND DESIRED POSE
double x=0;
double y=0;
double theta=0;
double angle_desired=0;

// SPEEDS APPLIED TO THE MOTORS
double pwmR=0;
double pwmL=0;

//PID CONTROL
double error;
double k_p=25;
double k_i=0;
double k_d=0;
double delta_tiempo_promedio=0; 
double IntegralTerm=0;
double ProportionalTerm=0;
double DerivativeTerm=0;
double PreviousError=0;
//TRASLATIONAL AND ROTATIONAL SPEEDS
double v=0;
double w=0;

//TIME VARIABLES 
volatile unsigned tiempoActual;
volatile unsigned tiempoAnteriorDeMuestreoL=0;
volatile unsigned deltaDeTiempoL;
volatile unsigned tiempoAnteriorDeMuestreoR=0;
volatile unsigned deltaDeTiempoR;


//FINDING SPEEDS RELATED TO THE PWMS
double speedLWithNoise=0;
double speedRWithNoise=0;

double speedLWithoutNoise=0;
double speedRWithoutNoise=0;

//WHEELS AND PHYSICAL VARIABLES 
volatile int ticksL=0;
volatile int ticksR=0;

double radius=3.4;

int countTicksL=0;

int countTicksR=0;

int max_ticks_noise=4;
double lengthSensor=8;
double lenght=10.6;


//DISTANCES COVERED BY THE WHEELS
double distanceCoveredByLeft=0;
double distanceCoveredByRight=0;
double distanceCoveredByCentre=0;


//TICKS VARIABLES 
double totalTicks=20;
int ticksRAnterior=0;
int ticksLAnterior=0;
int deltaTicksL=0;
int deltaTicksR=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(3,INPUT);//R_ENCODER
  pinMode(2,INPUT);//L_ENCODER
  
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);  
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT); 
  
  //SENSORS
    pinMode(8,OUTPUT);
    pinMode(9,INPUT);//PI/2
    pinMode(12,OUTPUT);
    pinMode(13,INPUT);//PI/4
    pinMode(14,OUTPUT);
    pinMode(15,INPUT);//0
    pinMode(16,OUTPUT);
    pinMode(17,INPUT);//-PI/4
    pinMode(18,OUTPUT);
    pinMode(19,INPUT);//-PI/2  

  attachInterrupt(digitalPinToInterrupt(2),leftUpdate,FALLING);
  attachInterrupt(digitalPinToInterrupt(3),rightUpdate,FALLING);

}

void loop() {  
  
  //COUNTING TIME FOR THE CURRENT POSITION
  tiempoActual=millis(); 
   
  deltaDeTiempoL=tiempoActual-tiempoAnteriorDeMuestreoL;  
  deltaDeTiempoR=tiempoActual-tiempoAnteriorDeMuestreoR;

  if(deltaDeTiempoL>500) speedLWithoutNoise=0;
  if(deltaDeTiempoR>500) speedRWithoutNoise=0;
  delta_tiempo_promedio=(deltaDeTiempoL+deltaDeTiempoR)/2000.0;
  //

  //PID_REGULATOR
  v=75;//60
  w=(speedRWithoutNoise-speedLWithoutNoise)/lenght;
  angle_desired=angle_avoiding_obstacles;
  error=angle_desired-theta;
  ProportionalTerm=error;  
    
  DerivativeTerm=(error-PreviousError)*1000/delta_tiempo_promedio;
  w=k_p*ProportionalTerm+k_i*IntegralTerm+k_d*DerivativeTerm; //input=w_new-w_actual
  
  pwmR=v+(w*lenght/2);
  pwmL=v-(w*lenght/2);
  
  PreviousError=error;
  //
  MaximunAndMinimun(100,40);
  
  /*//Maximum and Minimum PWM analysis
  if(pwmR>80) analogWrite(11,80);
  else if(pwmR<40) analogWrite(11,40);
  else analogWrite(11,pwmR); 
 
  if(pwmL>80) analogWrite(10,80);
  else if(pwmL<40) analogWrite(10,40);
  else analogWrite(10,pwmL);*/
  
  
  // CONTROL OF THE SENSE OF THE RIGHT WHEEL  
  digitalWrite(7,1);
  digitalWrite(6,0);
  //
  
  // CONTROL OF THE SENSE OF THE LEFT WHEEL    
  digitalWrite(5,1);
  digitalWrite(4,0);
  //
/*
  Serial.print(x); 
  Serial.print(",");
  Serial.println(y);*/
   Serial.print("Vector to follow: ");
  Serial.print(distance[0]);
  Serial.print(",");
  Serial.print(distance[1]);
  Serial.print(",");
  Serial.print(distance[2]);
  Serial.print(",");
  Serial.print(distance[3]);
  Serial.print(",");
  Serial.print(distance[4]);
  Serial.println(")");
  //*UPDATING LATELY BECAUSE THE CAR STARTED TURNING LEFT WITHOUT REASON
  ////FINDING THE AVOIDING OBSTACLE ANGLE
  
  //RESTARTING THE VECTORS
  vector_final_x=0;
  vector_final_y=0;
  //
  
  int temp;
  //UPDATING VECTORS TO OBSTACLES
  for(int i=0;i<SENSORSQUANTITY;i++){    

    //FIXING THE PINES ERRORS
    if(i==0) temp=8;
    if(i==1) temp=10;
    //TRIGGER AND ECHO
    digitalWrite(temp+2*i,1);
    int TimeMicroBefore=micros();
    while((micros()-TimeMicroBefore)<10){      
    }
    digitalWrite(temp+2*i,0);    
    double tiempo=pulseIn(temp+1+2*i,1);

    //CALCULATING DISTANCES
    distance[i]=340.0*tiempo/(2*10000); 
    
    if(distance[i]>60) distance[i]=60;

    //A BIT OF GEOMETRY 
    vector_x[i]=(longitud[i]+distance[i])*cos(angleSensor[i]+theta);
    vector_y[i]=(longitud[i]+distance[i])*sin(angleSensor[i]+theta);
    vector_final_x+=weightSensor[i]*vector_x[i];
    vector_final_y+=weightSensor[i]*vector_y[i];   
  }
  angle_avoiding_obstacles=atan2(vector_final_y,vector_final_x);
  
  odometry(); 
}

void MaximunAndMinimun(double pwmmax,double pwmmin){
    if(pwmR>pwmmax) analogWrite(11,pwmmax);
    else if(pwmR<pwmmin) analogWrite(11,pwmmin);
    else analogWrite(11,pwmR); 
    
    if(pwmL>pwmmax) analogWrite(10,pwmmax);
    else if(pwmL<pwmmin) analogWrite(10,pwmmin);
    else analogWrite(10,pwmL);
  
}

void leftUpdate(){
  countTicksL++;
  ticksL++;
  //Este codigo es para tener un poco mas de precision
  if(ticksL==max_ticks_noise){
      tiempoAnteriorDeMuestreoL=tiempoActual;
      speedLWithNoise=2*PI*radius*ticksL*1000/(totalTicks*deltaDeTiempoL);
      noise_elimination_L(speedLWithNoise);
      ticksL=0;  
  } 
}
void noise_elimination_L(double vel){
  static double array1[a];
  static int counting=1;  
    speedLWithoutNoise=0;
    
    for(int i=0;i<a-1;i++){
      array1[i]=array1[i+1];
      speedLWithoutNoise+=array1[i];
    }
    array1[a-1]=vel;
    if(counting<10) speedLWithoutNoise=(speedLWithoutNoise+vel)/counting;
    else speedLWithoutNoise=(speedLWithoutNoise+vel)/a;
    counting++;
}
void rightUpdate(){
  countTicksR++;
  ticksR++;
  if(ticksR==max_ticks_noise){
    tiempoAnteriorDeMuestreoR=tiempoActual;
    speedRWithNoise=2*PI*radius*ticksR*1000/(totalTicks*deltaDeTiempoR); 
    noise_elimination_R(speedRWithNoise);
    ticksR=0;
  }
}

void noise_elimination_R(double vel){
  static int counting=1;
  static double array1[a];
    speedRWithoutNoise=0;
    for(int i=0;i<a-1;i++){
      array1[i]=array1[i+1];
      speedRWithoutNoise+=array1[i];
    }
    array1[a-1]=vel;
    if(counting<10) speedRWithoutNoise=(speedRWithoutNoise+vel)/counting;
    else speedRWithoutNoise=(speedRWithoutNoise+vel)/a;
    counting++;
    speedRWithoutNoise=vel;
  
}

void odometry(void){
  deltaTicksL=countTicksL-ticksLAnterior;
  deltaTicksR=countTicksR-ticksRAnterior;
  
  distanceCoveredByLeft=2*PI*deltaTicksL*radius/totalTicks;
  distanceCoveredByRight=2*PI*deltaTicksR*radius/totalTicks;
  distanceCoveredByCentre=(distanceCoveredByRight+distanceCoveredByLeft)/2;

  x=x+distanceCoveredByCentre*cos(theta);
  y=y+distanceCoveredByCentre*sin(theta);

  theta=theta+(distanceCoveredByRight-distanceCoveredByLeft)/lenght;
  theta=atan2(sin(theta),cos(theta));
  
  ticksRAnterior=countTicksR;
  ticksLAnterior=countTicksL;  
  
}
