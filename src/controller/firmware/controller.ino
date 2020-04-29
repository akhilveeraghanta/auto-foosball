#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>

#define LED 13
/*Example sketch to control a stepper motor with A4988 stepper motor driver and Arduino without a library. More info: https://www.makerguides.com */
// Define stepper motor connections and steps per revolution:

//direction pin on Longruner pcb for Y axis motor driver
#define dirPinLinearOne 6
//step pin on Longruner pcb for Y axis motor driver
#define stepPinLinearOne 3

//direction pin on Longruner pcb for X axis motor driver
#define dirPinAngularOne 5
//step pin on Longruner pcb for X axis motor driver
#define stepPinAngularOne 2

//step pin on Longruner pcb for Z axis motor driver
#define dirPinLinearTwo 7
//step pin on Longruner pcb for Z axis motor driver
#define stepPinLinearTwo 4

//NOTE: double check the pins for the fourth axis 
//direction pin on Longruner pcb for N axis motor driver
#define dirPinAngularTwo 0
//step pin on Longruner pcb for N axis motor driver
#define stepPinAngularTwo 0

//needs to be pulled low at beginning to enable the longruner
#define enable 8
//steps per revolution for Nema 17
#define stepsPerRevolution 200
//one rotation of motor is 40 linear mm
#define linearDistancePerRotation 40 
//5 motor steps is one mm in lenear distance
#define stepsPerMM 5 // steps per rev / linearDistancePerRotation
//the rate at which all stepper motor accelerate (decrement size for pulseDelayAngular and pulseDelayLinear) 
#define accelStep 25   
//margin of error acceptable for the linear controller in mm
#define thresholdLinearOne 5 
//margin of error acceptable for the linear controller in degrees
#define thresholdAngularOne 10
//margin of error acceptable for the linear controller in mm
#define thresholdLinearTwo 5 
//margin of error acceptable for the linear controller in degrees
#define thresholdAngularTwo 10

//set maximum velocity for linear movements
#define maxSpeedLinear 750


////***ROD 1 telemetries****///
//starting velocity for Linear movement Rod 1
int pulseDelayLinearOne = 2500; 

//linear position in cm and mm Rod 1
int currPosLinearOne = 0;
int desiredPosLinearOne = 0;

//starting velocity for angular movement Rod 1
int pulseDelayAngularOne = 2500;

//angular position and direction in degrees Rod 1
int currPosAngularOne = 0;
int desiredPosAngularOne = 0;
int currDirAngularOne = 0;

//desired angular velocity for Rod 1,
int desiredAngularVelocityOne;


////***ROD 2 telemetries****///
//starting velocity for Linear movement Rod 2
int pulseDelayLinearTwo = 2500; 

//linear position in cm and mm Rod 2
int currPosLinearTwo = 0;
int desiredPosLinearTwo = 0;

//starting velocity for angular movement Rod 2
int pulseDelayAngularTwo = 2500;

//angular position and direction in degrees Rod 2
int currPosAngularTwo = 0;
int desiredPosAngularTwo = 0;
int currDirAngularTwo = 0;

//desired angular velocity for Rod 2
int desiredAngularVelocityTwo;


//nodehandler creation for ROS
ros::NodeHandle  nh;


//rod 1 callbacks for desired angular and linear positions as well as desired angular velocity
void setLinearPositionOne(std_msgs::Int32& msg){
     desiredPosLinearOne = msg.data;
   
}

void setAngularPositionOne(std_msgs::Int32& msg){
     desiredPosAngularOne = msg.data;
}

void setAngularVelocityOne(std_msgs::Int8& msg){

     //slow
     if (msg.data == 0){
        desiredAngularVelocityOne  = 2500;
     }
     //moderate
     else if (msg.data == 1){
        desiredAngularVelocityOne  = 1500;
     }
     //fast
     else if (msg.data == 2){
        desiredAngularVelocityOne  = 500;
     }
}

//rod 2 callbacks for desired angular and linear positions as well as desired angular velocity
void setLinearPositionTwo(std_msgs::Int32& msg){
     desiredPosLinearTwo = msg.data;
   
}

void setAngularPositionTwo(std_msgs::Int32& msg){
     desiredPosAngularTwo = msg.data;
}

void setAngularVelocityTwo(std_msgs::Int8& msg){
      
     //slow
     if (msg.data == 0){
        desiredAngularVelocityTwo  = 2500;
     }
     //moderate
     else if (msg.data == 1){
        desiredAngularVelocityTwo  = 1500;
     }
     //fast
     else if (msg.data == 2){
        desiredAngularVelocityTwo  = 500;
     }
}



///rod 1 callbacks for current angle, direction, and linear positions
void currLinearPosOne(std_msgs::Int32& msg){
     currPosLinearOne = msg.data;
}

void currAngularPosOne(std_msgs::Float32& msg){
     currPosAngularOne = msg.data;
}

void currAngularDirOne(std_msgs::Int8& msg){
     currDirAngularOne = msg.data;
}

//rod 2 callbacks for current angle, direction, and linear positions
void currLinearPosTwo(std_msgs::Int32& msg){
     currPosLinearTwo = msg.data;
}

void currAngularPosTwo(std_msgs::Float32& msg){
     currPosAngularTwo = msg.data;
}

void currAngularDirTwo(std_msgs::Int8& msg){
     currDirAngularTwo = msg.data;
}



//linear motion desired position rod 1 callback
ros::Subscriber<std_msgs::Int32> setLinearPositionOneCallBack("desiredLinearPositionOne", &setLinearPositionOne);

//desired position angular rod 1 callback
ros::Subscriber<std_msgs::Int32> setAngularPositionOneCallBack("desiredAngularPositionOne", &setAngularPositionOne);

//desired velocity angular rod 1 callback
ros::Subscriber<std_msgs::Int8> setAngularVelocityOneCallBack("desiredAngularVelocityOne", &setAngularVelocityOne);

//linear motion desired position rod 2 callback
ros::Subscriber<std_msgs::Int32> setLinearPositionTwoCallBack("desiredLinearPositionTwo", &setLinearPositionTwo);

//desired position angular rod 2 callback
ros::Subscriber<std_msgs::Int32> setAngularPositionTwoCallBack("desiredAngularPositionTwo", &setAngularPositionTwo);

//desired velocity angular rod 2 callback
ros::Subscriber<std_msgs::Int8> setAngularVelocityTwoCallBack("desiredAngularVelocityTwo", &setAngularVelocityTwo);


//current position linear rod 1 callback
ros::Subscriber<std_msgs::Int32> currLinearPosOneCallBack("currLinearPositionOne", &currLinearPosOne);

//current position angular  rod 1 callback
ros::Subscriber<std_msgs::Int32> currAngularPosOneCallBack("currAngularPositionOne", &currAngularPosOne);

//current direction angular  rod 1 callback
ros::Subscriber<std_msgs::Int8> currAngularDirOneCallBack("currAngularDirectionOne", &currAngularDirOne);

//current position linear rod 2 callback
ros::Subscriber<std_msgs::Int32> currLinearPosTwoCallBack("currLinearPositionTwo", &currLinearPosTwo);

//current position angular  rod 2 callback
ros::Subscriber<std_msgs::Int32> currAngularPosTwoCallBack("currAngularPositionTwo", &currAngularPosTwo);

//current direction angular  rod 2 callback
ros::Subscriber<std_msgs::Int8> currAngularDirTwoCallBack("currAngularDirectionTwo", &currAngularDirTwo);



//takes desired_pos as input (int type, mm units) and currPosLinear (int type in mm)
//converts desired position in mm to desired position in steps and performs move accordingly
//looks to see that the target position is within +/- 3 mm, otherwise adjusts

void positionControlLinearOne(int desiredPosLinearOne, float currPosLinearOne){

   
   bool switched_dirs = false;
   bool left = false;
   bool right = false;

   //if the direction to move is left
   if (desiredPosLinearOne - currPosLinearOne > 0){
        left  =  true;
   }
   //if the direction to move is right
   else{
        right = true;
   }
   
   int error = abs(desiredPosLinearOne - currPosLinearOne);
   
   
   //satisfied with where we are, stop and wait
   if (error <= thresholdLinearOne){
        pulseDelayLinearOne = 2500;
        haltLinearOne();
   }
   else{
        if (left){
          moveLeftLinearOne();
        }
        else{
          moveRightLinearOne();
        }
   }
   
    
}


//moveLeftLinear by however many steps
void moveLeftLinearOne(){
  
    digitalWrite(dirPinLinearOne, LOW);
    // These four lines result in 1 step:
    digitalWrite(stepPinLinearOne, HIGH);
    delayMicroseconds(pulseDelayLinearOne);
    digitalWrite(stepPinLinearOne, LOW);
    delayMicroseconds(pulseDelayLinearOne);
    //maxSpeed is actually the delay value, so logic is reversed
    if (pulseDelayLinearOne > maxSpeedLinear){
    pulseDelayLinearOne-=accelStep;
    }
}
//move right by however many steps
void moveRightLinearOne(){
        
 
    digitalWrite(dirPinLinearOne, HIGH);
    // These four lines result in 1 step:
    digitalWrite(stepPinLinearOne, HIGH);
    delayMicroseconds(pulseDelayLinearOne);
    digitalWrite(stepPinLinearOne, LOW);
    delayMicroseconds(pulseDelayLinearOne);
    if (pulseDelayLinearOne > maxSpeedLinear){
    pulseDelayLinearOne-=accelStep;
    } 
 }

void haltLinearOne(){
    digitalWrite(stepPinLinearOne, LOW);
}

//takes desired_pos as input (int type, mm units) and currPosLinear (int type in mm)
//converts desired position in mm to desired position in steps and performs move accordingly
//looks to see that the target position is within +/- 3 mm, otherwise adjusts

void positionControlLinearTwo(int desiredPosLinearTwo, float currPosLinearTwo){

   
   bool switched_dirs = false;
   bool left = false;
   bool right = false;

   //if the direction to move is left
   if (desiredPosLinearTwo - currPosLinearTwo > 0){
        left  =  true;
   }
   //if the direction to move is right
   else{
        right = true;
   }
   
   int error = abs(desiredPosLinearTwo - currPosLinearTwo);
   
   
   //satisfied with where we are, stop and wait
   if (error <= thresholdLinearTwo){
        pulseDelayLinearTwo = 2500;
        haltLinearTwo();
   }
   else{
        if (left){
          moveLeftLinearTwo();
        }
        else{
          moveRightLinearTwo();
        }
   }
   
    
}


//moveLeftLinear by however many steps
void moveLeftLinearTwo(){
  
    digitalWrite(dirPinLinearTwo, LOW);
    // These four lines result in 1 step:
    digitalWrite(stepPinLinearTwo, HIGH);
    delayMicroseconds(pulseDelayLinearTwo);
    digitalWrite(stepPinLinearTwo, LOW);
    delayMicroseconds(pulseDelayLinearTwo);
    //maxSpeed is actually the delay value, so logic is reversed
    if (pulseDelayLinearTwo > maxSpeedLinear){
    pulseDelayLinearTwo-=accelStep;
    }
}
//move right by however many steps
void moveRightLinearTwo(){
        
 
    digitalWrite(dirPinLinearTwo, HIGH);
    // These four lines result in 1 step:
    digitalWrite(stepPinLinearTwo, HIGH);
    delayMicroseconds(pulseDelayLinearTwo);
    digitalWrite(stepPinLinearTwo, LOW);
    delayMicroseconds(pulseDelayLinearTwo);
    if (pulseDelayLinearTwo > maxSpeedLinear){
    pulseDelayLinearTwo-=accelStep;
    } 
 }

void haltLinearTwo(){
    digitalWrite(stepPinLinearTwo, LOW);
}


//add the option for three different speeds 
//takes desired position in degrees, and current position in degrees
//has threshold of +/- thresholdAngular degrees margin of error
void positionControlAngularOne(int desiredPosAngularOne, int currPosAngularOne, int currDirAngularOne){

   bool cw = false;
   bool ccw = false;

  //if the direction of movement is cw
   if (currDirAngularOne == 0){
        cw  =  true;
   }
   //if the direction of movement is ccw
   else if (currDirAngularOne == 1){
        ccw = true;
   }
   
   int error = abs(desiredPosAngularOne - currPosAngularOne);
   
   
   //direction has changed, but still outside of threshold 
   //reset the speed to starting speed and correct
   //or could mean reset speed to start a new move
   if (error > thresholdAngularOne){
        pulseDelayAngularOne = 2500;
        if (cw){
          moveCWAngularOne();
        }
        else{
          moveCCWAngularOne();
        }
   }
   //satisfied with where we are, stop and wait
  
   else if (error <= thresholdAngularOne){
        pulseDelayAngularOne = 2500;
        haltAngularOne();
   }
   
    
}

//move cw by however many steps
void moveCWAngularOne(){
  
    digitalWrite(dirPinAngularOne, LOW);
    // These four lines result in 1 step:
    digitalWrite(stepPinAngularOne, HIGH);
    delayMicroseconds(pulseDelayAngularOne);
    digitalWrite(stepPinAngularOne, LOW);
    delayMicroseconds(pulseDelayAngularOne);
    //maxSpeedAngular is actually the delay value, so logic is reversed
    if (pulseDelayAngularOne >= desiredAngularVelocityOne){
    pulseDelayAngularOne-=accelStep;
    }
}
//move ccw by however many steps
void moveCCWAngularOne(){
        
 
    digitalWrite(dirPinAngularOne, HIGH);
    // These four lines result in 1 step:
    digitalWrite(stepPinAngularOne, HIGH);
    delayMicroseconds(pulseDelayAngularOne);
    digitalWrite(stepPinAngularOne, LOW);
    delayMicroseconds(pulseDelayAngularOne);
    if (pulseDelayAngularOne >= desiredAngularVelocityOne){
    pulseDelayAngularOne-=accelStep;
    } 
 }

void haltAngularOne(){
    digitalWrite(stepPinAngularOne, LOW);
}


//add the option for three different speeds 
//takes desired position in degrees, and current position in degrees
//has threshold of +/- thresholdAngular degrees margin of error
void positionControlAngularTwo(int desiredPosAngularTwo, int currPosAngularTwo,int currDirAngularTwo){

   bool cw = false;
   bool ccw = false;
   
    //if the direction of movement is cw
   if (currDirAngularTwo == 0){
        cw  =  true;
   }
   //if the direction of movement is ccw
   else if (currDirAngularTwo == 1){
        ccw = true;
   }
   
   int error = abs(desiredPosAngularTwo - currPosAngularTwo);
   
   
   //direction has changed, but still outside of threshold 
   //reset the speed to starting speed and correct
   //or could mean reset speed to start a new move
   if (error > thresholdAngularTwo){
        pulseDelayAngularTwo = 2500;
        if (cw){
          moveCWAngularTwo();
        }
        else{
          moveCCWAngularTwo();
        }
   }
   //satisfied with where we are, stop and wait
  
   else if (error <= thresholdAngularTwo){
        pulseDelayAngularTwo = 2500;
        haltAngularTwo();
   }
   
    
}

//move cw by however many steps
void moveCWAngularTwo(){
  
    digitalWrite(dirPinAngularTwo, LOW);
    // These four lines result in 1 step:
    digitalWrite(stepPinAngularTwo, HIGH);
    delayMicroseconds(pulseDelayAngularTwo);
    digitalWrite(stepPinAngularTwo, LOW);
    delayMicroseconds(pulseDelayAngularTwo);
    //maxSpeedAngular is actually the delay value, so logic is reversed
    if (pulseDelayAngularTwo >= desiredAngularVelocityTwo){
    pulseDelayAngularTwo-=accelStep;
    }
}
//move ccw by however many steps
void moveCCWAngularTwo(){
        
 
    digitalWrite(dirPinAngularTwo, HIGH);
    // These four lines result in 1 step:
    digitalWrite(stepPinAngularTwo, HIGH);
    delayMicroseconds(pulseDelayAngularTwo);
    digitalWrite(stepPinAngularTwo, LOW);
    delayMicroseconds(pulseDelayAngularTwo);
    if (pulseDelayAngularTwo >= desiredAngularVelocityTwo){
    pulseDelayAngularTwo-=accelStep;
    } 
 }

void haltAngularTwo(){
    digitalWrite(stepPinAngularTwo, LOW);
}

void setup() {
    // Declare pins as output linear motion rod 1 stepper
    pinMode(stepPinLinearOne, OUTPUT);
    pinMode(dirPinLinearOne, OUTPUT);
    
    // Declare pins as output: angular stepper rod 1 stepper
    pinMode(stepPinAngularOne, OUTPUT);
    pinMode(dirPinAngularOne, OUTPUT);

    // Declare pins as output linear motion rod 2 stepper
    pinMode(stepPinLinearTwo, OUTPUT);
    pinMode(dirPinLinearTwo, OUTPUT);
    
    // Declare pins as output: angular stepper rod 2 stepper
    pinMode(stepPinAngularTwo, OUTPUT);
    pinMode(dirPinAngularTwo, OUTPUT);
    
    //set enable pin for motor driver as output
    pinMode(enable, OUTPUT);

    //set enable to low enables the motor driver
    digitalWrite(enable, LOW);



    //ROS publisher and subscriber setup
    nh.initNode();
    //rod 1 desired positions
    nh.subscribe(setLinearPositionOneCallBack);
    nh.subscribe(setAngularPositionOneCallBack);
    nh.subscribe(setAngularVelocityOneCallBack);

    //rod 2 desired positions
    nh.subscribe(setLinearPositionTwoCallBack);
    nh.subscribe(setAngularPositionTwoCallBack);
    nh.subscribe(setAngularVelocityTwoCallBack);

    //rod 1 current positions
    nh.subscribe(currLinearPosOneCallBack);
    nh.subscribe(currAngularPosOneCallBack);
    nh.subscribe(currAngularDirOneCallBack);

    //rod 2 current positions
    nh.subscribe(currLinearPosTwoCallBack);
    nh.subscribe(currAngularPosTwoCallBack);
    nh.subscribe(currAngularDirTwoCallBack);
    
}
void loop() {

    //have to be getting feedback from linear closed loop system before actually doing anything
    //Linear position controller for rod 1
    if (currPosLinearOne != 0 && desiredPosLinearOne != 0){
        positionControlLinearOne(desiredPosLinearOne, currPosLinearOne);
    }

    //have to be getting feedback from linear closed loop system before actually doing anything
    //Linear position controller for rod 2
    if (currPosLinearTwo != 0 && desiredPosLinearTwo != 0){
        positionControlLinearTwo(desiredPosLinearTwo, currPosLinearTwo);
    }
    //have to be getting feedback from angular closed loop system before actually doing anything
    //Linear position controller for rod 1
    if (currPosAngularOne != 0 && desiredPosAngularOne != 0){
        positionControlAngularOne(desiredPosAngularOne, currPosAngularOne, currDirAngularOne);  
    }

    //have to be getting feedback from angular closed loop system before actually doing anything
    //Linear position controller for rod 2
    if (currPosAngularTwo != 0 && desiredPosAngularTwo != 0){
        positionControlAngularTwo(desiredPosAngularTwo, currPosAngularTwo, currDirAngularTwo);  
    }
  
    nh.spinOnce();
}
