#include<math.h>

#define a 10 //ARRAY LENGHT FOR FILTERING
#define C 1//SCALATOR 
#define SENSORSQUANTITY 5

//angles sensors and distances

double angleSensor[SENSORSQUANTITY]={PI/2,PI/4,0,-PI/4,-PI/2};
double weightSensor[SENSORSQUANTITY]={5.5,6,0.5,6,5.5};
double longitud[SENSORSQUANTITY]={5.3,7.5,11.5,7.5,5.3};
double distance[SENSORSQUANTITY]={0,0,0,0,0};
double angle_avoiding_obstacles=0;

//vectors and Hard Switches variables
double vector_x[SENSORSQUANTITY];
double vector_y[SENSORSQUANTITY];

double vectorAOx=0;
double vectorAOy=0;

double vectorGTGx=0;
double vectorGTGy=0;

double vector_final_x=0;
double vector_final_y=0;

double factorBehaviorsSwitches=0.5;

// CURRENT AND DESIRED POSE
double x=0;
double y=0;
double theta=PI/2;
double angle_desired=0;
double x_goal=-12;  
double y_goal=105;

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

int TimeMicroBefore=0;
/*//FIXING THE ERROR OF THE ODOMETRY
double speedBeforeSensors=0;
double thetaBeforeSensors=0;
double timeWaiting=0;
double AccumulatingErrorX=0;
double AccumulatingErrorY=0;
int logic=1;
double errorTime=0;*/

//EVENTS
bool atObstacle;
double distanceObstacle=25;
bool atGoal;
double distanceGoal;
bool unsafe;
double distanceUnsafe=8;
bool obstacleCleared;
double distanceCleared=30;
bool at_most1=0;


//FOLLOWING WALLS VARIABLES
int temp;
int least_distance_ind,least_distance_one_past_ind;
double least_distance=50,least_distance_one_past=50;
int howManySoFar=0;

double vector_x_par_wall,vector_y_par_wall,vector_x_per_wall,vector_y_per_wall,modulo_par_wall,modulo_per_wall;
double v_final_per_x,v_final_per_y,v_final_par_x,v_final_par_y,v_following_wall_x,v_following_wall_y;

double alpha=0.5,bheta=0.5;
double distanceWantedToTheWall=12;
double angle_following_walls=0;

//SlidingWises
double vector_x_par_wall_cw;
double vector_y_par_wall_cw;
double vector_x_par_wall_ccw;
double vector_y_par_wall_ccw;
bool wallDetected=0;
bool FirstTime=1;
bool ClockWise=0;
void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
  pinMode(3,INPUT);//R_ENCODER
  pinMode(2,INPUT);//L_ENCODER

  //DRIVER
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
   
  //events:  
  distanceGoal=pow(x_goal-x,2)+pow(y_goal-y,2);
  bool atGoal=distanceGoal<16;
  
  bool unsafe=0;
  bool atObstacle=0;
  int counting=0;
  
  MeasureAOangleAndFollowingWalls();
  for(int i=0;i<SENSORSQUANTITY;i++){
    if(distance[i]<distanceUnsafe) unsafe=1;
    if(distance[i]>distanceCleared) counting++;
    if(distance[i]<distanceObstacle) atObstacle=1;
  }
  bool obstacleCleared=(counting==SENSORSQUANTITY);
  at_most1=(counting>=(SENSORSQUANTITY-1));
  //PID_REGULATOR
  v=75;
  w=(speedRWithoutNoise-speedLWithoutNoise)/lenght;

  
  
  if(atGoal) Stop();
  else if(obstacleCleared) GoToGoal();  
  else if(unsafe) AvoidObstacles();
  else if(wallDetected) FollowingWalls();
  else if(atObstacle) BlendingAOandGTG();    

  //Analysis of the nearest position
  if(!atGoal){
     //Maximum and Minimum PWM analysis
    MaximunAndMinimun(105,40);
  }
  //PID  
  error=angle_desired-theta;
  ProportionalTerm=error;
    
  DerivativeTerm=(error-PreviousError)*1000/delta_tiempo_promedio;
  w=k_p*ProportionalTerm+k_i*IntegralTerm+k_d*DerivativeTerm; //input=w_new-w_actual
  
  pwmR=C*(v+(w*lenght/2));
  pwmL=C*(v-(w*lenght/2));
  
  PreviousError=error;
  //  

   odometry(); 

  Serial.print(x); 
  Serial.print(",");
  Serial.println(y);
  
  
  // CONTROL OF THE SENSE OF THE RIGHT WHEEL  
  digitalWrite(7,1);
  digitalWrite(6,0);
  //
  
  // CONTROL OF THE SENSE OF THE LEFT WHEEL    
  digitalWrite(5,1);
  digitalWrite(4,0);
  //
}
/*
bool ClearShot(){
  double mod=sqrt(pow(x_goal-x,2)+pow(y_goal-y,2));
  double vector_GTG_x=(x_goal-x)/mod;
  double vector_GTG_y=(y_goal-y)/mod;
  double mod2=sqrt(pow(vector_final_y,2)+pow(vector_final_x,2));
  double vector_AO_x=vector_final_x/mod2;
  double vector_AO_y=vector_final_y/mod2;

  double innerProduct=vector_GTG_x*vector_AO_x+vector_AO_y*vector_GTG_y;
  return innerProduct>0;  
}
in this code it doesn't work cuz ao is not too extreme
*/
bool slidingClockWise(){
  double mod=sqrt(pow(x_goal-x,2)+pow(y_goal-y,2));
  double vector_GTG_x=(x_goal-x)/mod;
  double vector_GTG_y=(y_goal-y)/mod;
  double mod2=sqrt(pow(vector_x_par_wall_cw,2)+pow(vector_y_par_wall_cw,2));
  double vector_cw_x=(vector_x_par_wall_cw)/mod2;
  double vector_cw_y=(vector_y_par_wall_cw)/mod2;
  double productoInner=vector_cw_x*vector_GTG_x+vector_GTG_y*vector_cw_y;
  return productoInner>=0;
  
}
void AvoidObstacles(void){
  angle_desired=angle_avoiding_obstacles;
}

void GoToGoal(void){
  angle_desired=atan2(y_goal-y,x_goal-x);
}
void FollowingWalls(void){
  angle_desired=angle_following_walls;
}
void MaximunAndMinimun(double pwmmax,double pwmmin){
    if(pwmR>pwmmax) analogWrite(11,pwmmax);
    else if(pwmR<pwmmin) analogWrite(11,pwmmin);
    else analogWrite(11,pwmR); 
    
    if(pwmL>pwmmax) analogWrite(10,pwmmax);
    else if(pwmL<pwmmin) analogWrite(10,pwmmin);
    else analogWrite(10,pwmL);
}
void Stop(void){
  analogWrite(11,0);
  analogWrite(10,0);
  exit(0);
}

void BlendingAOandGTG(){
  //COMBINIG AO AND GTG
  vectorGTGx=x_goal-x;
  vectorGTGy=y_goal-y;
  double modulo=sqrt(pow(vectorGTGx,2)+pow(vectorGTGy,2));
  vectorGTGx=(x_goal-x)/modulo;
  vectorGTGy=(y_goal-y)/modulo;
  
  vectorAOx=cos(angle_avoiding_obstacles);
  vectorAOy=sin(angle_avoiding_obstacles);

  vector_final_x=factorBehaviorsSwitches*vectorAOx+(1-factorBehaviorsSwitches)*vectorGTGx;
  vector_final_y=factorBehaviorsSwitches*vectorAOy+(1-factorBehaviorsSwitches)*vectorGTGy;

  angle_desired=atan2(vector_final_y,vector_final_x); 
}

void MeasureAOangleAndFollowingWalls(){
  int temp;
  vector_final_x=0;
  vector_final_y=0;
  least_distance=50;
  least_distance_one_past=50;
/*
  //Saving speeds Before the measuring of the distance sensors
  speedBeforeSensors=(speedRWithoutNoise+speedLWithoutNoise)/2;
  thetaBeforeSensors=theta;
  timeWaiting=50*pow(10,-6);
  errorTime=0;
*/  
  howManySoFar=0;
  //UPDATING VECTORS TO OBSTACLES
  for(int i=0;i<SENSORSQUANTITY;i++){    

    //FIXING THE PINES ERRORS
    if(i==0) temp=8;
    if(i==1) temp=10;
    
    //TRIGGER AND ECHO
    digitalWrite(temp+2*i,1);
    TimeMicroBefore=micros();
    while((micros()-TimeMicroBefore)<10){      
    }    
    digitalWrite(temp+2*i,0);    
    double tiempo=pulseIn(temp+1+2*i,1);
/*    errorTime+=tiempo;//time that the robots is losing the goal location*/
    //CALCULATING DISTANCES
    distance[i]=340.0*tiempo/(2*10000); 
    if(distance[i]>60)distance[i]=60;
    if(distance[i]>50) howManySoFar++;
    

    //A BIT OF GEOMETRY 
    vector_x[i]=(longitud[i]+distance[i])*cos(angleSensor[i]+theta);
    vector_y[i]=(longitud[i]+distance[i])*sin(angleSensor[i]+theta);
    vector_final_x+=weightSensor[i]*vector_x[i];
    vector_final_y+=weightSensor[i]*vector_y[i]; 
    if(least_distance>=distance[i]){
          least_distance=distance[i];
          least_distance_ind=i;
    }  
  }
  for(int i=0;i<SENSORSQUANTITY;i++){    
    if(i!=least_distance_ind){
      if(least_distance_one_past>=distance[i]){
        least_distance_one_past=distance[i];
        least_distance_one_past_ind=i;
      }    
    }
  }
  if(howManySoFar>2 && howManySoFar<5){
    if(least_distance_ind==0 ||least_distance_ind==3) least_distance_one_past_ind=least_distance_ind+1;
    if(least_distance_ind==1 ||least_distance_ind==4) least_distance_one_past_ind=least_distance_ind-1;
  }
  
  angle_avoiding_obstacles=atan2(vector_final_y,vector_final_x);


  vector_x_par_wall=-vector_x[least_distance_ind]+vector_x[least_distance_one_past_ind];
  vector_y_par_wall=-vector_y[least_distance_ind]+vector_y[least_distance_one_past_ind];
  vector_x_per_wall,vector_y_per_wall;
  modulo_par_wall=sqrt(pow(vector_x_par_wall,2)+pow(vector_y_par_wall,2));
  //v=v_least-((v_least.v_par_wall)/(|v_par_wall|^2))v_par_wall
  //v_least.v_par_wall=((vector_x[least_distance_ind]*vector_x_par_wall)+(vector_y[least_distance_ind]*vector_y_par_wall))
  //(|v_par_wall|^2))=(pow(vector_x_par_wall,2)+pow(vector_y_par_wall,2))
  vector_x_per_wall=vector_x[least_distance_one_past_ind]-(((vector_x[least_distance_one_past_ind]*vector_x_par_wall)+(vector_y[least_distance_one_past_ind]*vector_y_par_wall))/(pow(modulo_par_wall,2)))*vector_x_par_wall;
  vector_y_per_wall=vector_y[least_distance_one_past_ind]-(((vector_x[least_distance_one_past_ind]*vector_x_par_wall)+(vector_y[least_distance_one_past_ind]*vector_y_par_wall))/(pow(modulo_par_wall,2)))*vector_y_par_wall;
  modulo_per_wall=sqrt(pow(vector_x_per_wall,2)+pow(vector_y_per_wall,2));
  
  
  //v_final_per=v_per_wall-distanceWantedToTheWall*(v_final/|v_final|)
  v_final_per_x=vector_x_per_wall-distanceWantedToTheWall*vector_x_per_wall/modulo_per_wall;
  v_final_per_y=vector_y_per_wall-distanceWantedToTheWall*vector_y_per_wall/modulo_per_wall;

  v_final_par_x=vector_x_par_wall/modulo_par_wall;
  v_final_par_y=vector_y_par_wall/modulo_par_wall;

  v_following_wall_x=alpha*v_final_par_x+bheta*v_final_per_x;
  v_following_wall_y=alpha*v_final_par_y+bheta*v_final_per_y;

  if(FirstTime){
    FirstTime=0;        
    vector_x_par_wall_cw=cos(theta+PI/2);
    vector_y_par_wall_cw=sin(theta+PI/2);; 

    vector_x_par_wall_ccw=cos(theta-PI/2);
    vector_y_par_wall_ccw=sin(theta-PI/2);; ;
    if(slidingClockWise()) ClockWise=1;
    else ClockWise=0;    
  }
  if(distance[2]<25){
    wallDetected=1;
    /*
    vector_x_par_wall_cw=-vector_x[2]+vector_x[1];
    vector_y_par_wall_cw=-vector_y[2]+vector_y[1]; 

    vector_x_par_wall_ccw=-vector_x[3]+vector_x[4];
    vector_y_par_wall_ccw=-vector_y[3]+vector_y[4];

  */
    if(ClockWise) angle_following_walls=theta+PI/2;
    else angle_following_walls=theta-PI/2;
  }
  else{
    angle_following_walls=atan2(v_following_wall_y,v_following_wall_x);
    if(at_most1) wallDetected=0;
  }
 
 /* 
  //Solving Accumulating Error because of the time that the robot spend measuring distances
  timeWaiting+=errorTime*pow(10,-6);
  AccumulatingErrorX+=timeWaiting*speedBeforeSensors*cos(thetaBeforeSensors);
  AccumulatingErrorY+=timeWaiting*speedBeforeSensors*sin(thetaBeforeSensors);
 */ 
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
  IntegralTerm+=error*delta_tiempo_promedio;
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
