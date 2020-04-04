 
#include <SPI.h>      //SPI library for communicate with the nRF24L01+
#include "RF24.h"     //The main library of the nRF24L01+
#include "HCPCA9685.h" // Include the HCPCA9685 library created by Andrew Davies
#define I2CAdd 0x40 // Default address of the PCA9685 Module
//Define packets to receive
uint16_t data[12];
//Define object from RF24 library - 8 and 10 are a digital pin numbers to which signals CE and CSN are connected
RF24 radio(9,10);
//Create a pipe addresses for the communicate
const uint64_t pipe = 0xE8E8F0F0E4LL;
// Variables used to store the Position of each Servos
// Data [0] is used to set servo 0 to center position when triggered 
int Servo0Position; //data 1 claw
int Servo1Position; //data 2 intermediate
int Servo2Position; //data 3 lever
int Servo3Position; //data 4 base
//For Motor Drive
int Servo4Position; //data 5
int Servo5Position; //data 6
//For Camera
int Servo6Position; //Pan Camera data 7
int Servo7Position; //Tilt Camera data 8
// Motor A Connections
int enA = 5;
int in1 = 4;
int in2 = 3;
// Motor B Connections
int enB = 6;
int in3 = 7;
int in4 = 8;
// Motor Speed Values - Start at zero
int MotorSpeed1 = 0;
int MotorSpeed2 = 0;
// Joystick Values - Start at 512 (middle position)
int joyposVert = 512;
int joyposHorz = 512;  
HCPCA9685 HCPCA9685(I2CAdd); // Define Library to use I2C communication

//ButtonSwitch
//int pbuttonPin;//Assigned pin 7 on transmitter
int light1 = A0;//External Lighting
int light2 = A1;//External Lighting
int val = 0;//push value from pbuttonPin
int lightON = 0; //Light status
int pushed = 0;//push status
//Buzzer
int LEDState = 0; //initial state of the output pin
//int LEDPin = 2; //Assign digital output pin (or analogPin)
int LEDPin = A2;
int buzzer;




void setup() 
{
 Serial.begin(9600);
  radio.begin();                    //Start the nRF24 communicate            
  radio.openReadingPipe(1, pipe);   //Sets the address of the transmitter to which the program will receive data.
  radio.startListening();  

  analogWrite(light1,0); //Make Analog Pin initially LOW
  analogWrite(light2,0);
  // Set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  // Start with motors disabled and direction forward
  // Motor A
  digitalWrite(enA, LOW);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Motor B
  digitalWrite(enB, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
HCPCA9685.Init(SERVO_MODE); // Set to Servo Mode. PWM mode for LED's
HCPCA9685.Sleep(false); // Wake up PCA9685 module
// Set Center value for the Servos
Servo0Position=150; //Set value for Claw Release
Servo1Position=200;
Servo2Position=150;
Servo3Position=250;
//Not needed for Motor Drive Vx and Vy (Forward & Backward)
//Servo4Positon=250;
//Servo5Positon=250;
Servo6Position=140; //For pan camera
Servo7Position=250;
// Move Servo at start to center position
HCPCA9685.Servo(0, Servo0Position); // Move Servo 0 
//HCPCA9685.Servo(1, Servo1Position);// Move Servo 1
HCPCA9685.Servo(2, Servo2Position);// Move Servo 2
HCPCA9685.Servo(3, Servo3Position);// Move Servo 3
//For Motor Drive Vx and Vy (Forward & Backward) not required
//HCPCA9685.Servo(4, Servo4Position);// Move Servo 4 
//HCPCA9685.Servo(5, Servo5Position);
HCPCA9685.Servo(8, Servo6Position); // Move Servo 6
HCPCA9685.Servo(14, Servo7Position); // Move Servo 7
//pinMode(LEDPin,OUTPUT);
}
void loop() 
{ if (radio.available()){
      radio.read(data, sizeof(data));   
if (!data[0]) { // If Joystick switch is clicked re-center selected Servos
delay(500); // delay for debouncing the joystick switch
// Set all Servos position variable back to center values (Comment out if not need to reset)
Servo0Position=140 ;//A value of 250 sets the servo to center position
//For Motor Drive 4 & 5 Servo4Position and Servo5Position
//Servo1Position = 250 ;
//Servo2Position=250;
//Servo3Position=250;
//Servo4Position=250;
//Servo5Position=250;
// Move Servo to center position
HCPCA9685.Servo(0, Servo0Position); // Move Servo 0 
//HCPCA9685.Servo(1, Servo1Position);// Move Servo 1
//HCPCA9685.Servo(2, Servo2Position);// Move Servo 2
//HCPCA9685.Servo(3, Servo3Position);// Move Servo 3
//For Motor Drive (not needed)
//HCPCA9685.Servo(4, Servo4Position);// Move Servo 4
//HCPCA9685.Servo(5, Servo5Position);// Move Servo 5
}
if (!data[9]) { // If Joystick switch is clicked re-center selected Servos
delay(500); // delay for debouncing the joystick switch
//Servo6Position=140;//A value of 250 sets the servo to center position
Servo7Position=280;//moves tilts camera
//HCPCA9685.Servo(8, Servo6Position); // Move Servo 6 
HCPCA9685.Servo(14, Servo7Position);// Move Servo 7
}
// if Joystick X axis in not in center move Servo 0 accordingly
if (data[1] > 800) 
{if (Servo0Position < 305) // Check if maximum movement of Servo ( from 10 to 420)
{Servo0Position++;} // Increase Servo 0 Position variable by 1
delay(10); // Decrease to make Servos move faster or increase for slower movement
HCPCA9685.Servo(0, Servo0Position); // Move Servo 0 Forward
}
if (data[1] < 300) 
{ if (Servo0Position > 150) // Check if lower limit is reached ( Assigned port 0 on PCA9685 module)
{Servo0Position--;}
delay(10); 
HCPCA9685.Servo(0, Servo0Position); // Move Servo 0 Backward
} 
// if Joystick Y axis in not in center move Servo 1 accordingly

if ( data[2] > 800) {
if (Servo1Position < 270)//250
{Servo1Position++;} 
delay(10);
HCPCA9685.Servo(2, Servo1Position); // Move Servo 1 (Assigned port 2 on PCA9685 module)
}
if ( data[2] < 300) { if (Servo1Position > 130) //150
{Servo1Position--;}
delay(10);
HCPCA9685.Servo(2, Servo1Position); // Move Servo 1
}
// if Joystick U axis in not in center move Servo 2 accordingly
if ( data[3] > 800) 
{if (Servo2Position < 350) 
{Servo2Position++;} 
delay(10);
HCPCA9685.Servo(4, Servo2Position); // Move Servo 2 (Assigned port 4 on PCA9685 module)
}
if ( data[3] < 300) 
{if (Servo2Position > 100) 
{Servo2Position--;}
delay(10);
HCPCA9685.Servo(4, Servo2Position); // Move Servo 2
}
// if Joystick V axis in not in center move Servo 3 accordingly
if ( data[4] > 800) 
{if (Servo3Position < 350) 
{Servo3Position++;} 
delay(10);
HCPCA9685.Servo(6, Servo3Position); // Move Servo 3
}
if ( data[4] < 300) 
{ if (Servo3Position > 20) 
{Servo3Position--;}
delay(10);
HCPCA9685.Servo(6, Servo3Position); // Move Servo 3
}
// if Joystick for camera in not in center move Servo 6 accordingly
if (data[7] > 800) 
{if (Servo6Position < 420) // Check if maximum movement of Servo ( from 10 to 420)
{Servo6Position--;} // Increase Servo 6 Position variable by 1
delay(10); // Decrease to make Servos move faster or increase for slower movement
HCPCA9685.Servo(8, Servo6Position); // Move Servo 0 Forward
}
if (data[7] < 300) 
{ if (Servo6Position > 10) // Check if lower limit is reached
{Servo6Position++;}
delay(10); 
HCPCA9685.Servo(8, Servo6Position); // Move Servo 6 Backward
} 
// if Joystick for camera in not in center move Servo 7 accordingly
if (data[8] > 800) 
{if (Servo7Position < 420) // Check if maximum movement of Servo ( from 10 to 420)
{Servo6Position++;} // Increase Servo 7 Position variable by 1
delay(10); // Decrease to make Servos move faster or increase for slower movement
HCPCA9685.Servo(14, Servo7Position); // Move Servo 7 Forward
}
if (data[8] < 300) 
{ if (Servo7Position > 10) // Check if lower limit is reached
{Servo7Position--;}
delay(10); 
HCPCA9685.Servo(14, Servo7Position); // Move Servo 7 Backward
} 
// Read the Joystick X and Y positions
  joyposVert = data[5]; 
  joyposHorz = data[6];
  // Determine if this is a forward or backward motion
  // Do this by reading the Vertical Value
  // Apply results to MotorSpeed and to Direction
  if (joyposVert < 460)
  {
    // This is Backward
    // Set Motor A backward
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    // Set Motor B backward
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    //Serial.print("Reverse");
    //Determine Motor Speeds
    // As we are going backwards we need to reverse readings
    joyposVert = joyposVert - 460; // This produces a negative number
    joyposVert = joyposVert * -1;  // Make the number positive
    MotorSpeed1 = map(joyposVert, 0, 460, 0, 255);
    MotorSpeed2 = map(joyposVert, 0, 460, 0, 255);
  }
  else if (joyposVert > 564)
  {
    // This is Forward
    // Set Motor A forward
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    // Set Motor B forward
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    //Serial.print("Forward   ");
    //Determine Motor Speeds
    MotorSpeed1 = map(joyposVert, 564, 1023, 0, 255);
    MotorSpeed2 = map(joyposVert, 564, 1023, 0, 255); 
  }
  else
  {
    // This is Stopped
    //Serial.println("Stopped   ");
    MotorSpeed1 = 0;
    MotorSpeed2 = 0; 
  }
  // Now do the steering
  // The Horizontal position will "weigh" the motor speed
  // Values for each motor
  if (joyposHorz < 460)
  {
    // Move Left
    // As we are going left we need to reverse readings
    joyposHorz = joyposHorz - 460; // This produces a negative number
    joyposHorz = joyposHorz * -1;  // Make the number positive
    // Map the number to a value of 255 maximum
    joyposHorz = map(joyposHorz, 0, 460, 0, 255);
    //Serial.print("   Right  ");   
    MotorSpeed1 = MotorSpeed1 - joyposHorz;
    MotorSpeed2 = MotorSpeed2 + joyposHorz;
    // Don't exceed range of 0-255 for motor speeds
    if (MotorSpeed2 > 255)MotorSpeed2 = 255;
    if (MotorSpeed1 < 0)MotorSpeed1 = 0;  
  }
  else if (joyposHorz > 564)
  {
    // Move Right
    // Map the number to a value of 255 maximum
    joyposHorz = map(joyposHorz, 564, 1023, 0, 255);
    //Serial.print("   Left  ");
    MotorSpeed1 = MotorSpeed1 + joyposHorz;
    MotorSpeed2 = MotorSpeed2 - joyposHorz;
    // Don't exceed range of 0-255 for motor speeds
    if (MotorSpeed2 < 0)MotorSpeed2 = 0;
    if (MotorSpeed1 > 255)MotorSpeed1 = 255;
  }
  // Adjust to prevent "buzzing" at very low speed
  if (MotorSpeed1 < 8)MotorSpeed1 = 0;
  if (MotorSpeed2 < 8)MotorSpeed2 = 0;
  // Set the motor speeds
  analogWrite(enB, MotorSpeed1);//Swap ENA and ENB to choose which wheel to turn
  analogWrite(enA, MotorSpeed2);
    //Display the Motor Control values in the serial monitor.
    //Serial.print("   Motor A: ");
    //Serial.print(MotorSpeed1);
    //Serial.print("   Motor B: ");
    //Serial.println(MotorSpeed2);
    //Serial.print("X:    ");
    //Serial.print(data[1]);
   // Serial.print("Y:    ");
    //Serial.println(data[2]);
   // Serial.print("U:    ");
    //Serial.print(data[3]);
    //Serial.print("     V:    ");
    //Serial.println(data[4]);
    //Serial.print("Vx:    ");
    //Serial.print(data[5]);
    //Serial.print("Vy:    ");
      //Serial.print("Camera Pan  ");
      //Serial.print(data[7]);
      //Serial.print("   Camera Tilt  ");
      //Serial.println(data[8]);

  val = data[10];// read the push button value
  Serial.println(data[10]);
  if(val == HIGH && lightON == LOW){
 const unsigned long interval9 = 100;  //ms  // Using millis instead of delay
  unsigned long last_sent9;            
  unsigned long now9 = millis();
  if (now9 - last_sent9 >= interval9){
      pushed = 1-pushed;
    //delay(100);
    }
  //last_sent9 = now9;//Turns it ON after set time interval? 100ms
  }    

  lightON = val;

      if(pushed == HIGH){
        Serial.println("Light ON");
        analogWrite(light1, 250);
        analogWrite(light2, 250);
       
      }else{
        Serial.println("Light OFF");
        analogWrite(light1,0);
        analogWrite(light2,0);
   
      }   
  // Using millis instead of delay  (can be used for debouncing)  
  const unsigned long interval10 = 100;  //ms  
  unsigned long last_sent10;            
  unsigned long now10 = millis();
  if (now10 - last_sent10 >= interval10){
  buzzer = data[11];
   Serial.println(data[11]);
   if (buzzer==0){analogWrite(LEDPin,250);
   //last_sent10 = now10;
   //Turns it on after set time interval? Ex: 100ms delay
   }
   else{analogWrite(LEDPin,0);}}

  /*unsigned long now = millis();
  if (now - last_sent >= interval) {   // If it's time to send a data, send it!
    last_sent = now;*
    replace delays with interval using millis*/


     
    }
  }
