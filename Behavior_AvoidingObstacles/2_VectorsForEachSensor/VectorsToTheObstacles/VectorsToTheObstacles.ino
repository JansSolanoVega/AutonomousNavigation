#include<math.h>
#define a 10 //ARRAY LENGHT
#define SENSORSQUANTITY 5

//angles sensors and distances

double angleSensor[SENSORSQUANTITY]={PI/2,PI/4,0,-PI/4,-PI/2};
double weightSensor[SENSORSQUANTITY]={3,3,1,3,3};
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


//WHEELS VARIABLES 
double radius=3.4;
double lenght=10.6;


void setup() {
  // put your setup code here, to run once:
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
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  vector_final_x=0;
  vector_final_y=0;
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
    
    if(distance[i]>50) distance[i]=50;

    //A BIT OF GEOMETRY 
    vector_x[i]=(longitud[i]+distance[i])*cos(angleSensor[i]+theta);
    vector_y[i]=(longitud[i]+distance[i])*sin(angleSensor[i]+theta);
    vector_final_x+=weightSensor[i]*vector_x[i];
    vector_final_y+=weightSensor[i]*vector_y[i];   
  }
  angle_avoiding_obstacles=atan2(vector_final_y,vector_final_x);
  
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
  
  Serial.print("Angle Avoiding Obstacles: ");
  Serial.println(angle_avoiding_obstacles*180.0/PI);
  
}
