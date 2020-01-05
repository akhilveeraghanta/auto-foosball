#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>

#define LED 13
/*Example sketch to control a stepper motor with A4988 stepper motor driver and Arduino without a library. More info: https://www.makerguides.com */
// Define stepper motor connections and steps per revolution:
//X direction pin on Longruner pcb for Y axis motor driver
#define dirPinLinear 6
//X step pin on Longruner pcb for Y axis motor driver
#define stepPinLinear 3
//X direction pin on Longruner pcb for X axis motor driver
#define dirPinAngular 5
//X step pin on Longruner pcb for X axis motor driver
#define stepPinAngular 2
//needs to be pulled low at beginning to enable the longruner
#define enable 8
//steps per revolution for Nema 17
#define stepsPerRevolution 200
//one rotation of motor is 40 linear mm
#define linearDistancePerRotation 40 
//5 motor steps is one mm in lenear distance
#define stepsPerMM 5 // steps per rev / linearDistancePerRotation
//currently the max speed for linear motion
#define maxSpeedLinear 750 
//currently the max speed for angular motion
#define maxSpeedAngular 2500 
//the rate at which all stepper motor accelerate (decrement size for pulseDelayAngular and pulseDelayLinear) 
#define accelStep 25   
//margin of error acceptable for the linear controller in mm
#define thresholdLinear 5 
//margin of error acceptable for the linear controller in degrees
#define thresholdAngular 20

//three different desired velocities for angular controller
#define slow  1
#define moderate 2
#define fast  3

//starting velocity for Linear movement Rod 1
int pulseDelayLinearOne = 2500; 

//linear position in cm and mm Rod 1
int currPosLinearOne = 0;
int desiredPosLinearOne = 40;

//starting velocity for angular movement Rod 1
int pulseDelayAngularOne = 2500;

//angular position in degrees Rod 1
int currPosAngularOne = 0;
int desiredPosAngularOne = 60;

//desired angular velocity for Rod 1, three options: slow, moderate, fast
int desiredAngularVelocityOne = slow;


//starting velocity for Linear movement Rod 2
int pulseDelayLinearTwo = 2500; 

//linear position in cm and mm Rod 2
int currPosLinearTwo = 0;
int desiredPosLinearTwo = 40;

//starting velocity for angular movement Rod 2
int pulseDelayAngularTwo = 2500;

//angular position in degrees Rod 2
int currPosAngularTwo = 0;
int desiredPosAngularTwo = 60;

//desired angular velocity for Rod 2, three options: slow, moderate, fast
int desiredAngularVelocityTwo = slow;


ros::NodeHandle  nh;


void setLinearPositionOne(std_msgs::Int32& msg){
     desiredPosLinearOne = msg.data;
   
}

void setAngularPositionOne(std_msgs::Int32& msg){
     desiredPosAngularOne = msg.data;
}

void setAngularVelocityOne(std_msgs::Int8& msg){
      
     if (msg.data == slow ||msg.data == moderate ||msg.data == fast){
      desiredAngularVelocityOne = msg.data;
     }
}

void setLinearPositionTwo(std_msgs::Int32& msg){
     desiredPosLinearTwo = msg.data;
   
}

void setAngularPositionTwo(std_msgs::Int32& msg){
     desiredPosAngularTwo = msg.data;
}

void setAngularVelocityTwo(std_msgs::Int8& msg){
      
     if (msg.data == slow ||msg.data == moderate ||msg.data == fast){
      desiredAngularVelocityTwo = msg.data;
     }
}

void getCurrAngularPosition(std_msgs::Int32& msg){
     currPosAngular = msg.data;
}

void getCurrPosLinear(std_msgs::Float32& msg){
     currPosLinear = msg.data;
}


//linear motion desired position callback
ros::Subscriber<std_msgs::Int32> setLinearPositionOneCallBack("desiredLinearPositionOne", &setLinearPositionOne);

//desired position angular callback
ros::Subscriber<std_msgs::Int32> setAngularPositionOneCallBack("desiredAngularPositionOne", &setAngularPositionOne);

//desired velocity angular callback
ros::Subscriber<std_msgs::Int8> setAngularVelocityOneCallBack("desiredAngularVelocityOne", &setAngularVelocityOne);

//linear motion desired position callback
ros::Subscriber<std_msgs::Int32> setLinearPositionTwoCallBack("desiredLinearPositionTwo", &setLinearPositionTwo);

//desired position angular callback
ros::Subscriber<std_msgs::Int32> setAngularPositionTwoCallBack("desiredAngularPositionTwo", &setAngularPositionTwo);

//desired velocity angular callback
ros::Subscriber<std_msgs::Int8> setAngularVelocityTwoCallBack("desiredAngularVelocityTwo", &setAngularVelocityTwo);


//current position linear callback
ros::Subscriber<std_msgs::Int32> currLinearPosOneCallBack("currLinearPositionOne", &currLinearPosOne);

//current position angular callback
ros::Subscriber<std_msgs::Int32> getCurrAngularPositionCallBack("currPosAngular", &getCurrAngularPosition);

//ultrasonic publisher linear position rod 1
std_msgs::Int32 currLinearPosOne;
ros::Publisher currLinearPosOneTopic("/uno/currLinearPositionOne", &currLinearPosOne);

//encoder angular position publisher rod 1
std_msgs::Int32 currAngularPosOne;
ros::Publisher currAngularPosOneTopic("/uno/currAngularPositionOne", &currAngularPosOne);

//encoder angular direction publisher rod 1
std_msgs::Int8 currAngularDirOne;
ros::Publisher currAngularDirOneTopic("/uno/currAngularDirectionOne", &currAngularDirOne);

//ultrasonic publisher linear position rod 2
std_msgs::Int32 currLinearPosTwo;
ros::Publisher currLinearPosTwoTopic("/uno/currLinearPositionTwo", &currLinearPosTwo);

//encoder angular position publisher rod 2
std_msgs::Int32 currAngularPosTwo;
ros::Publisher currAngularPosTwoTopic("/uno/currAngularPositionTwo", &currAngularPosTwo);

//encoder angular direction publisher rod 2
std_msgs::Int8 currAngularDirTwo;
ros::Publisher currAngularDirTwoTopic("/uno/currAngularDirectionTwo", &currAngularDirTwo);





//takes desired_pos as input (int type, mm units) and currPosLinear (float type in cm), and bool type dir (direction)
//converts desired position in mm to desired position in steps and performs move accordingly
//looks to see that the target position is within +/- 3 mm, otherwise adjusts

void positionControlLinear(int desiredPosLinear, float currPosLinear){

   //convert to an integer with units mm 
   currPosLinear = (int)(currPosLinear*10);
   
   bool switched_dirs = false;
   bool left = false;
   bool right = false;

   //if the direction to move is left
   if (desiredPosLinear - currPosLinear > 0){
        if (right){
          switched_dirs = true;
        }
        else{
          switched_dirs = false;
        }
        left  =  true;
   }
   //if the direction to move is right
   else{
        if (left){
          switched_dirs = true;
        }
        else{
          switched_dirs = false;
        }
        right = true;
   }
   
   int error = abs(desiredPosLinear - currPosLinear);
   
   
   //direction has changed, but still outside of threshold 
   //or could mean reset speed to start a new move
   if (switched_dirs && error > thresholdLinear){
        pulseDelayLinear = 2500;
        if (left){
          moveLeftLinear();
        }
        else{
          moveRightLinear();
        }
   }
   //satisfied with where we are, stop and wait
   else if (error <= thresholdLinear){
        pulseDelayLinear = 2500;
        haltLinear();
   }
   else{
        if (left){
          moveLeftLinear();
        }
        else{
          moveRightLinear();
        }
   }
   
    
}


//moveLeftLinear by however many steps
void moveLeftLinear(){
  
    digitalWrite(dirPinLinear, LOW);
    // These four lines result in 1 step:
    digitalWrite(stepPinLinear, HIGH);
    delayMicroseconds(pulseDelayLinear);
    digitalWrite(stepPinLinear, LOW);
    delayMicroseconds(pulseDelayLinear);
    //maxSpeed is actually the delay value, so logic is reversed
    if (pulseDelayLinear > maxSpeedLinear){
    pulseDelayLinear-=accelStep;
    }
}
//move right by however many steps
void moveRightLinear(){
        
 
    digitalWrite(dirPinLinear, HIGH);
    // These four lines result in 1 step:
    digitalWrite(stepPinLinear, HIGH);
    delayMicroseconds(pulseDelayLinear);
    digitalWrite(stepPinLinear, LOW);
    delayMicroseconds(pulseDelayLinear);
    if (pulseDelayLinear > maxSpeedLinear){
    pulseDelayLinear-=accelStep;
    } 
 }

void haltLinear(){
    digitalWrite(stepPinLinear, LOW);
}


//add the option for three different speeds 
//takes desired position in degrees, and current position in degrees
//has threshold of +/- thresholdAngular degrees margin of error
void positionControlAngular(int desiredPosAngular, int currPosAngular, int desiredAngularVelocity){

   int des = desiredPosAngular;
   int curr = currPosAngular;
   int error;
   bool switched_dirs = false;
   bool cw = false;
   bool ccw = false;



   //direction has changed, but still outside of threshold 
   //reset the speed to starting speed and correct
   //or could mean reset speed to start a new move
   if (error > thresholdAngular){
        pulseDelayAngular = 2500;
        if (cw){
          moveCWAngular();
        }
        else{
          moveCCWAngular();
        }
   }
   //satisfied with where we are, stop and wait
  
   else if (error <= thresholdAngular){
        pulseDelayAngular = 2500;
        haltAngular();
   }
   
    
}

//move cw by however many steps
void moveCWAngular(){
  
    digitalWrite(dirPinAngular, LOW);
    // These four lines result in 1 step:
    digitalWrite(stepPinAngular, HIGH);
    delayMicroseconds(pulseDelayAngular);
    digitalWrite(stepPinAngular, LOW);
    delayMicroseconds(pulseDelayAngular);
    //maxSpeedAngular is actually the delay value, so logic is reversed
    if (pulseDelayAngular > maxSpeedAngular){
    pulseDelayAngular-=accelStep;
    }
}
//move ccw by however many steps
void moveCCWAngular(){
        
 
    digitalWrite(dirPinAngular, HIGH);
    // These four lines result in 1 step:
    digitalWrite(stepPinAngular, HIGH);
    delayMicroseconds(pulseDelayAngular);
    digitalWrite(stepPinAngular, LOW);
    delayMicroseconds(pulseDelayAngular);
    if (pulseDelayAngular > maxSpeedAngular){
    pulseDelayAngular-=accelStep;
    } 
 }

void haltAngular(){
    digitalWrite(stepPinAngular, LOW);
}


void setup() {
    // Declare pins as output linear stepper
    pinMode(stepPinLinear, OUTPUT);
    pinMode(dirPinLinear, OUTPUT);
    
    // Declare pins as output: angular stepper
    pinMode(stepPinAngular, OUTPUT);
    pinMode(dirPinAngular, OUTPUT);

    //set enable pin for motor driver as output
    pinMode(enable, OUTPUT);

    //set enable to low enables the motor driver
    digitalWrite(enable, LOW);



    //ROS publisher and subscriber setup
    nh.initNode();
    nh.subscribe(setLinearPositionCallBack);
    nh.subscribe(getCurrLinearDistanceCallBack);
    nh.subscribe(getCurrAngularPositionCallBack);
    nh.subscribe(setAngularVelocityCallBack);
    nh.subscribe(setAngularPositionCallBack);

    
}
void loop() {

    //have to be getting feedback from linear closed loop system before actually doing anything
    /*if (currPosLinear != 0){
        positionControlLinear(desiredPosLinear, currPosLinear);
    }*/
    
    
    positionControlAngular(desiredPosAngular, currPosAngular, desiredAngularVelocity);   
    nh.spinOnce();
}
