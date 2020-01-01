#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

#define LED 13
/*Example sketch to control a stepper motor with A4988 stepper motor driver and Arduino without a library. More info: https://www.makerguides.com */
// Define stepper motor connections and steps per revolution:
//X direction pin on Longruner pcb
#define dirPin 6
//X step pin on Longruner pcb
#define stepPin 3
//needs to be pulled low at beginning to enable the longruner
#define enable 8
//steps per revolution for Nema 17
#define stepsPerRevolution 200
#define linearDistancePerRotation 40 //in mm
#define stepsPerMM 5 // steps per rev / linearDistancePerRotation
#define maxSpeed 750  // this is actually the pulseWidth delay value
#define accelStep 25   // rate at which the delay between digital high and low is changed
#define threshold 5 //threshold of error allowed in mm

float currPosLinear = 0; //centered starting position, at 5 cm mark
int pulseDelay = 2500; 
int desiredPosLinear = 0;  //centered starting position, at 50 mm mark


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
        halt();
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
  
    digitalWrite(dirPin, LOW);
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelay);
    //maxSpeed is actually the delay value, so logic is reversed
    if (pulseDelay > maxSpeed){
    pulseDelay-=accelStep;
    }
}
//move right by however many steps
void moveRightLinear(){
        
 
    digitalWrite(dirPin, HIGH);
    // These four lines result in 1 step:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelay);
    if (pulseDelay > maxSpeed){
    pulseDelay-=accelStep;
    } 
 }

void halt(){
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelay);
}

 
void setup() {
    // Declare pins as output:
    pinMode(stepPin, OUTPUT);
    pinMode(enable, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(LED, OUTPUT);
  
    digitalWrite(enable, LOW);
    digitalWrite(dirPin, LOW);
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
