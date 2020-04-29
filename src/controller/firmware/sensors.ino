#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>

//NOTE: These can be any digital pins
//pins for ultrasonic linear distance sensor rod one
#define TRIG_PIN_ONE 9
#define ECHO_PIN_ONE 10

//NOTE: These can be any digital pins
//pins for ultrasonic linear distance sensor rod two
#define TRIG_PIN_TWO 11
#define ECHO_PIN_TWO 12

//NOTE: The pins below may have to change depending on what is available
//NOTE: They can be any digital pins
// Rotary Encoder Input Defines Encoder 1
#define CLK_1 2
#define DT_1 3
#define SW_1 4

//NOTE: The pins below may have to change depending on what is available
//NOTE: They can be any digital pins
// Rotary Encoder Input Defines Encoder 2
#define CLK_2 5
#define DT_2 6
#define SW_2 7

//variables for encoder 1
int counter_1 = 0;
int currentStateCLK_1;
int lastStateCLK_1;
int currentDir_1;

//variables for encoder 2
int counter_2 = 0;
int currentStateCLK_2;
int lastStateCLK_2;
int currentDir_2;




//node_handler creation
ros::NodeHandle nh_sensor;

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



//returns the current linear position of rod one in mm 
int getCurrPosLinearOne(){
  // defines variables
  long duration;
  int distance;
  // Clears the trigPin
  digitalWrite(TRIG_PIN_ONE, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN_ONE, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_ONE, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN_ONE, HIGH);
  // Calculating the distance in mm
  distance= int(10*duration*0.034/2);

  return distance;
}


//returns the current linear position of rod one in mm 
int getCurrPosLinearTwo(){
  // defines variables
  long duration;
  int distance;
  // Clears the trigPin
  digitalWrite(TRIG_PIN_TWO, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN_TWO, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_TWO, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN_TWO, HIGH);
  // Calculating the distance in mm
  distance= int(10*duration*0.034/2);

  return distance;
}

//rotate every loop which one is published (even and odd with a counter

int getCurrPosAngularOne(){

  currentStateCLK_1 = digitalRead(CLK_1);
 
  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK_1 != lastStateCLK_1  && currentStateCLK_1 == 1){

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT_1) != currentStateCLK_1) {
      counter_1 --;
      currentDir_1 = 1;
    } else {
      // Encoder is rotating CW so increment
      counter_1 ++;
      currentDir_1 = 0;
    }
  }

  // Remember last CLK state
  lastStateCLK_1 = currentStateCLK_1;

}

int getCurrPosAngularTwo(){

  currentStateCLK_2 = digitalRead(CLK_2);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK_2 != lastStateCLK_2  && currentStateCLK_2 == 1){

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT_2) != currentStateCLK_2) {
      counter_2 --;
      currentDir_2 = 1;
    } else {
      // Encoder is rotating CW so increment
      counter_2 ++;
      currentDir_2 = 0;
    }
  }

  // Remember last CLK state
  lastStateCLK_2 = currentStateCLK_2;
}


int getCurrDirAngularOne(){
      return currentDir_1;
}


int getCurrDirAngularTwo(){
      return currentDir_2;
}


void setup() {

  //Set encoder for rod one as output
  pinMode(TRIG_PIN_ONE, OUTPUT); 
  pinMode(ECHO_PIN_ONE, INPUT); 

  //Set encoder for rod two as output
  pinMode(TRIG_PIN_TWO, OUTPUT); 
  pinMode(ECHO_PIN_TWO, INPUT); 

  // Set encoder 1 pins as inputs
  pinMode(CLK_1,INPUT);
  pinMode(DT_1,INPUT);
  pinMode(SW_1, INPUT_PULLUP);

  // Set encoder 2 pins as inputs
  pinMode(CLK_2,INPUT);
  pinMode(DT_2,INPUT);
  pinMode(SW_2, INPUT_PULLUP);


  //Get initial Value for Encoder 1 clk
  lastStateCLK_1 = digitalRead(CLK_1);
  
  //Get initial Value for Encoder 2 clk
  lastStateCLK_2 = digitalRead(CLK_2);

  
  nh_sensor.initNode();
  nh_sensor.advertise(currLinearPosOneTopic);
  nh_sensor.advertise(currLinearPosTwoTopic);
  nh_sensor.advertise(currAngularPosOneTopic);
  nh_sensor.advertise(currAngularPosTwoTopic);
  nh_sensor.advertise(currAngularDirOneTopic);
  nh_sensor.advertise(currAngularDirTwoTopic);
  
}
void loop() {

   //Linear position telemetries
   currLinearPosOne.data = getCurrPosLinearOne();
   currLinearPosTwo.data = getCurrPosLinearTwo();

   //Angular position telemetries
   currAngularPosOne.data = getCurrPosAngularOne();
   currAngularPosTwo.data = getCurrPosAngularTwo();

   //Angular direction telemetries (O for cw, 1 for ccw)
   //This segment should follow the angular position function calls as it shares global variables
   //with the getCurrPosAngular functions
   currAngularDirOne.data = getCurrDirAngularOne();
   currAngularDirTwo.data = getCurrDirAngularTwo();

   //publish all telemetries
   currLinearPosOneTopic.publish(&currLinearPosOne);
   currLinearPosTwoTopic.publish(&currLinearPosTwo);
   currAngularPosOneTopic.publish(&currAngularPosOne);
   currAngularPosTwoTopic.publish(&currAngularPosTwo);
   currAngularDirOneTopic.publish(&currAngularDirOne);
   currAngularDirTwoTopic.publish(&currAngularDirTwo);


   nh_sensor.spinOnce();
   
}
