#include<math.h>

#define a 10 //ARRAY LENGHT
#define C 1//SCALATOR 

// CURRENT AND DESIRED POSE
double x_goal=50;  
double y_goal=50;
double x=0;
double y=0;
double theta=PI;

double angle_desired=atan2(y_goal-y,x_goal-x);
// SPEEDS APPLIED TO THE MOTORS
double pwmR=0;
double pwmL=0;

//PID CONTROL
double error;
double k_p=25;//30
  
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

//WHEELS VARIABLES 
volatile int ticksL=0;
volatile int ticksR=0;

double radius=3.4;

int countTicksL=0;

int countTicksR=0;

int max_ticks_noise=4;


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

  attachInterrupt(digitalPinToInterrupt(2),leftUpdate,FALLING);
  attachInterrupt(digitalPinToInterrupt(3),rightUpdate,FALLING);

}

void loop() {  
  tiempoActual=millis(); 
   
  deltaDeTiempoL=tiempoActual-tiempoAnteriorDeMuestreoL;  
  deltaDeTiempoR=tiempoActual-tiempoAnteriorDeMuestreoR;

  //Esta parte es para solucionar que cuando el carro se detenga no tenga velocidad 0 
  if(deltaDeTiempoL>500) speedLWithoutNoise=0;
  if(deltaDeTiempoR>500) speedRWithoutNoise=0;

  //P_REGULATOR
  v=75;//2*sqrt(pow(y_goal-y,2)+pow(x_goal-x,2));
  w=(speedRWithoutNoise-speedLWithoutNoise)/lenght;

  angle_desired=atan2(y_goal-y,x_goal-x);
  error=angle_desired-theta;
  w=k_p*error;


  pwmR=C*(v+(w*lenght/2));
  pwmL=C*(v-(w*lenght/2));
  
  
  //Analysis of the nearest position
  if((pow(x_goal-x,2)+pow(y_goal-y,2))<9){
    analogWrite(11,0);
    analogWrite(10,0);
    exit(0);
  }

  else{
     //Maximum and Minimum PWM analysis
    MaximunAndMinimun(105,40);
  }
  
    // CONTROL VELOCIDAD LLANTA DERECHA
  
  digitalWrite(7,1);
  digitalWrite(6,0);

  
  // CONTROL VELOCIDAD LLANTA IZQUIERDA
  
  digitalWrite(5,1);
  digitalWrite(4,0);
  
  Serial.print(x); 
  Serial.print(",");
  Serial.println(y); 
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
//CODE TO SHOW SPEED IN A LCD
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
