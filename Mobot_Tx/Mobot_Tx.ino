/* Controlling Servos and motors using NRF24L01
 * by Webslinger2011
 * Transmitter
 */
 
#include <SPI.h>        //SPI library for communicate with the nRF24L01+
#include "RF24.h"       //The main library of the nRF24L01+

#define JoySwitch 4 // Joystick switch connected to Pin 4 on UNO -data 0
#define JoyX A0 // Joystick X pin connected to A0 on the UNO - data 1
#define JoyY A1 // Joystick Y pin connected to A1 on the UNO -data 2
#define JoyU A3 // data 3
#define JoyV A2 // data 4
#define TiltCam A4// For camera servo  //data 8
#define PanCam A5 //For camera servo // data 7
#define CamSwitch 8 //For centering camera pan function// data 9

//Button Switch
int pbuttonPin = 7;//Lighting
int buzzer = 5;//Buzzer pin



// Define Joystick Values - Start at 512 (middle position)
int joyVert = A7; // data 5
int joyHorz = A6; // data 6

//Define 10 data packets to be sent
uint16_t data[12]; // puts 12 objects in an array 

//Define object from RF24 library - 9 and 10 are a digital pin numbers to which signals CE and CSN are connected.
RF24 radio(9,10);

//Create a pipe addresses for the communicate                                    
const uint64_t pipe = 0xE8E8F0F0E4LL;

void setup(void){
  Serial.begin(9600);
  radio.begin();                 //Start the nRF24 communicate     
  radio.openWritingPipe(pipe);   //Sets the address of the receiver to which the program will send data.
  pinMode(JoySwitch, INPUT_PULLUP); // Pull SW to ground to activate
  pinMode(CamSwitch, INPUT_PULLUP);
  pinMode(pbuttonPin, INPUT_PULLUP);
  pinMode(buzzer,INPUT_PULLUP);
 
}

void loop(void){
   data[0] = digitalRead(JoySwitch); //Opens/Release claw
   data[1] = analogRead(JoyX); //Controls Forearm
   data[2] = analogRead(JoyY); //Controls Intermesiate
   data[3] = analogRead(JoyU); //Controls Main arm
   data[4] = analogRead(JoyV); //Controls Arm Base
   data[5] = analogRead(joyVert); //Drives mobot forward or backward
   data[6] = analogRead(joyHorz); //Drives mobot to turn
  data[7] = analogRead(PanCam); //Read values for camera pan servo A5
   //Display the Motor Control values in the serial monitor.
  data[8] = analogRead(TiltCam); //Read value for camera tilt servo A4
  data[9] = digitalRead(CamSwitch);//Read value for centering camera position D8
  data[10] = digitalRead(pbuttonPin);//Read value on pin 7 
  data[11] = digitalRead(buzzer);//Read value on pin 5
  //Serial.println(data[11]);
 
  radio.write(data, sizeof(data));// Send data to receiver
}
