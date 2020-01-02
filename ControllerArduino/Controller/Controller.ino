#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#define LED 13
/*Example sketch to control a stepper motor with A4988 stepper motor driver and Arduino without a library. More info: https://www.makerguides.com */
// Define stepper motor connections and steps per revolution:
//X direction pin on Longruner pcb for Y axis motor driver
#define dirPinLinear 6
//X step pin on Longruner pcb for Y axis motor driver
#define stepPinLinear 3
//X direction pin on Longruner pcb for Y axis motor driver
#define dirPinAngular 5
//X step pin on Longruner pcb for Y axis motor driver
#define stepPinAngular 2
//needs to be pulled low at beginning to enable the longruner
#define enable 8
//steps per revolution for Nema 17
#define stepsPerRevolution 200
#define linearDistancePerRotation 40 //in mm
#define stepsPerMM 5 // steps per rev / linearDistancePerRotation
#define maxSpeed 750  // this is actually the pulseWidth delay value
#define accelStep 25   // rate at which the delay between digital high and low is changed
#define threshold 5 //threshold of error allowed in mm

float currPosLinear = 3.0;
int pulseDelay = 2500;   //starting velocity for all movements
int desiredPosLinear = 3;


ros::NodeHandle  nh;



void setCurrDistance(std_msgs::Float32& msg){
     currPosLinear = msg.data;
}

void moveUnoRoutine(std_msgs::Int32& msg){
     desiredPosLinear = msg.data;
   
}


ros::Subscriber<std_msgs::Int32> moveUnoStepperCallBack("moveUnoStepper", &moveUnoRoutine);
ros::Subscriber<std_msgs::Float32> getCurrLinearDistanceCallBack("linearDistance", &setCurrDistance);


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
   //reset the speed to starting speed and correct
   //or could mean reset speed to start a new move
   if (switched_dirs && error > threshold){
        pulseDelay = 2500;
        if (left){
          moveLeftLinear();
        }
        else{
          moveRightLinear();
        }
   }
   //satisfied with where we are, stop and wait
   else if (error <= threshold){
        pulseDelay = 2500;
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


//add the option for three different speeds 
//takes desired position in degrees, and current position in degrees
//has threshold of +/- 2 degrees margin of error
void positionControlAngular(int desiredPosAngular, float currPosAngular){


   bool switched_dirs = false;
   bool cw = false;
   bool ccw = false;

   //if the direction to move is left
   if (desiredPosAngular - currPosAngular > 0){
        if (ccw){
          switched_dirs = true;
        }
        else{
          switched_dirs = false;
        }
        cw  =  true;
   }
   //if the direction to move is right
   else{
        if (cw){
          switched_dirs = true;
        }
        else{
          switched_dirs = false;
        }
        ccw = true;
   }
   
   int error = abs(desiredPosAngular - currPosAngular);
   
   
   //direction has changed, but still outside of threshold 
   //reset the speed to starting speed and correct
   //or could mean reset speed to start a new move
   if (switched_dirs && error > threshold){
        pulseDelay = 2500;
        if (cw){
          moveLeftAngular();
        }
        else{
          moveRightAngular();
        }
   }
   //satisfied with where we are, stop and wait
   else if (error <= threshold){
        pulseDelay = 2500;
        haltLinear();
   }
   else{
        if (cw){
          moveLeftAngular();
        }
        else{
          moveRightAngular();
        }
   }
   
    
}

//moveLeftLinear by however many steps
void moveLeftLinear(){
  
    digitalWrite(dirPinLinear, LOW);
    // These four lines result in 1 step:
    digitalWrite(stepPinLinear, HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(stepPinLinear, LOW);
    delayMicroseconds(pulseDelay);
    //maxSpeed is actually the delay value, so logic is reversed
    if (pulseDelay > maxSpeed){
    pulseDelay-=accelStep;
    }
}
//move right by however many steps
void moveRightLinear(){
        
 
    digitalWrite(dirPinLinear, HIGH);
    // These four lines result in 1 step:
    digitalWrite(stepPinLinear, HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(stepPinLinear, LOW);
    delayMicroseconds(pulseDelay);
    if (pulseDelay > maxSpeed){
    pulseDelay-=accelStep;
    } 
 }

void haltLinear(){
    digitalWrite(stepPinLinear, LOW);
    delayMicroseconds(pulseDelay);
}

//moveLeftLinear by many steps
void moveLeftAngular(){
  
    digitalWrite(dirPinAngular, LOW);
    // These four lines result in 1 step:
    digitalWrite(stepPinAngular, HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(stepPinAngular, LOW);
    delayMicroseconds(pulseDelay);
    //maxSpeed is actually the delay value, so logic is reversed
    if (pulseDelay > maxSpeed){
    pulseDelay-=accelStep;
    }
}
//move right by however many steps
void moveRightAngular(){
        
 
    digitalWrite(dirPinAngular, HIGH);
    // These four lines result in 1 step:
    digitalWrite(stepPinAngular, HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(stepPinAngular, LOW);
    delayMicroseconds(pulseDelay);
    if (pulseDelay > maxSpeed){
    pulseDelay-=accelStep;
    } 
 }

void haltAngular(){
    digitalWrite(stepPinAngular, LOW);
    delayMicroseconds(pulseDelay);
}


void setup() {
    // Declare pins as output:
    pinMode(stepPinLinear, OUTPUT);
    pinMode(enable, OUTPUT);
    pinMode(dirPinLinear, OUTPUT);
    pinMode(LED, OUTPUT);
  
    digitalWrite(enable, LOW);
    digitalWrite(dirPinLinear, LOW);
    //ROS publisher and subscriber setup
    nh.initNode();
    nh.subscribe(moveUnoStepperCallBack);
    nh.subscribe(getCurrLinearDistanceCallBack);
}
void loop() {

    //have to actually be getting values from ROS to start controlling
    if (currPosLinear != 0 && desiredPosLinear != 0){
        positionControlLinear(desiredPosLinear, currPosLinear);
    }

   
    nh.spinOnce();
}
