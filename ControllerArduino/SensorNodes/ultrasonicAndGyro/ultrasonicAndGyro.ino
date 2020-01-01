
#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh_sensor;

// defines pins numbers
const int trigPin = 9;
const int echoPin = 10;
// defines variables
long duration;
float distance;

//ultrasonic publisher
std_msgs::Float32 linearDist;
ros::Publisher linearDistanceTopic("/uno/linearDistance", &linearDist);


float getUltraVal(){
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance= duration*0.034/2;

  return distance;
}


void setup() {
  
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

  nh_sensor.initNode();
  nh_sensor.advertise(linearDistanceTopic);
}
void loop() {

   linearDist.data = getUltraVal();
   linearDistanceTopic.publish(&linearDist);
   delay(100);
   nh_sensor.spinOnce();
}
