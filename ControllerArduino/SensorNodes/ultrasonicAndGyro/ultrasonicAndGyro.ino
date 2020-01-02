#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include<Wire.h>


#define TRIG_PIN 9
#define ECHO_PIN 10
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

const int MPU_addr=0x68; int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

int minVal=265; int maxVal=402;

int x, y, z;

//node_handler creation
ros::NodeHandle  nh_sensor;

//ultrasonic publisher
std_msgs::Float32 linearDist;
ros::Publisher linearDistanceTopic("/uno/linearDistance", &linearDist);
//gyro angular position publisher
std_msgs::Int16 currPosAngular;
ros::Publisher angularPosTopic("/uno/currPosAngular", &currPosAngular);


///***INTERRUPT ROUTING FOR GYRO***////
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

///***CURRENT LINEAR POSITION FUNCTION***////
float getCurrPosLinear(){
  // defines variables
  long duration;
  float distance;
  // Clears the trigPin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  // Calculating the distance
  distance= duration*0.034/2;

  return distance;
}

///**CURRENT ANGULAR POSITION FUNCTION***///
int getCurrPosAngular(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true); 
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read(); 
  int xAng = map(AcX,minVal,maxVal,-90,90); 
  int yAng = map(AcY,minVal,maxVal,-90,90); 
  int zAng = map(AcZ,minVal,maxVal,-90,90);

  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI); 
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);

  return z;
}


void setup() {

  Wire.begin(); 
  Wire.beginTransmission(MPU_addr); 
  Wire.write(0x6B); 
  Wire.write(0);
  Wire.endTransmission(true);

  nh_sensor.initNode();
  nh_sensor.advertise(linearDistanceTopic);
  nh_sensor.advertise(angularPosTopic);
 
  pinMode(TRIG_PIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHO_PIN, INPUT); // Sets the echoPin as an Input


}
void loop() {

   linearDist.data = getCurrPosLinear();
   linearDistanceTopic.publish(&linearDist);
   currPosAngular.data = getCurrPosAngular();
   angularPosTopic.publish(&currPosAngular);
   nh_sensor.spinOnce();
}
