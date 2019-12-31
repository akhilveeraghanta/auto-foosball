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
    void setup() {
      // Declare pins as output:
      pinMode(stepPin, OUTPUT);
      pinMode(enable, OUTPUT);
      pinMode(dirPin, OUTPUT);

      digitalWrite(enable, LOW);
      digitalWrite(dirPin, LOW);

      
    }
    void loop() {
     
            for(int i = 0; i < stepsPerRevolution; i++)
            {
              // These four lines result in 1 step:
              digitalWrite(stepPin, HIGH);
              delayMicroseconds(2000);
              digitalWrite(stepPin, LOW);
              delayMicroseconds(2000);
            }
          
              digitalWrite(dirPin, HIGH);
            
            for(int i = 0; i < stepsPerRevolution; i++)
            {
              // These four lines result in 1 step:
              digitalWrite(stepPin, HIGH);
              delayMicroseconds(2000);
              digitalWrite(stepPin, LOW);
              delayMicroseconds(2000);
            }
          
            digitalWrite(dirPin, LOW);

  
   }
