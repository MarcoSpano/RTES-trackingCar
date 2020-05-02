#include <Servo.h>
#include "SR04.h"

#define TRIG_PIN 12
#define ECHO_PIN 11


#define ENABLE 5
#define DIRA 3
#define DIRB 4

#define STEERING 9

#define INPUT_SIZE 5

Servo servo;  // create servo object to control a servo

//ultrasonic sensor
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
long a;


char input[INPUT_SIZE + 1];
byte size;
char read_byte, startMarker = '<', endMarker = '>';
char * separator;
int componentId, componentValue;

int i = 0, speed = 0;
bool direction = false, change = false, getting = false;

void get_input() {
  if (Serial.available() > 0 && change != true) {
    read_byte = Serial.read();
    //Serial.println(read_byte);
    
    if(getting == true) {
      //if the message is bigger than our buffer we cancel it
      if(i > INPUT_SIZE) {
        i = 0;
        getting = false;
        
        //Serial.println("Non dovrebbe essere qua\n");
      }
      
      if(read_byte != endMarker) {
        input[i] = read_byte;
        i++;
        //Serial.println("Input[");
        //Serial.println(i);
        //Serial.println("] inserito\n");
      } else {
        // Add the final 0 to end the C string
        input[i] = 0;
        //Serial.println("parsiaml\n");
        parse_command();

        //ready to receive a new command
        getting = false;
        i = 0;
      }
    } else if(read_byte == startMarker) {
      //we start storing the command
      //Serial.println("inizio comando\n");
      getting = true;
    }
  }
}

void parse_command() {
    separator = strchr(input, ':');
    if (separator != 0)
    {
        // Actually split the string in 2: replace ':' with 0
        *separator = 0; //change ":" with "\0"
        componentId = atoi(input); //from 0 to separator
        ++separator;
        componentValue = atoi(separator); //from separator to the end
    }

    change = true;
    //Serial.println(componentId);
    //Serial.println(componentValue);
}

void motor_control() {
  speed = (componentValue % 128) * 2;
  direction = componentValue / 128;
  
  //Serial.println(direction);
  //Serial.println(speed);

  if(direction == 0) {
    digitalWrite(ENABLE,HIGH); // enable on
    analogWrite(ENABLE,speed);
    digitalWrite(DIRA,HIGH); //one way
    digitalWrite(DIRB,LOW);
    delay(50);
  } else {
    digitalWrite(ENABLE,HIGH); // enable on
    analogWrite(ENABLE,speed);
    digitalWrite(DIRA,LOW);  //reverse
    digitalWrite(DIRB,HIGH);
    delay(50);
  }
}

void steering_control() {
  servo.write(componentValue); 
}

//core

void setup() { 
  //motor ports
  pinMode(ENABLE,OUTPUT);
  pinMode(DIRA,OUTPUT);
  pinMode(DIRB,OUTPUT);

  //steering port
  servo.attach(STEERING);

  
  Serial.begin(9600);
  //Serial.setTimeout(50);
}

void loop() {

  //sensor output part
  a=sr04.Distance();
  Serial.println(a);
  
  
  //actuators input part
  get_input();

  if(change == true) {
    change = false;
    //Serial.println("faccio");
    if(componentId == 1) {
      motor_control();
    } else if(componentId == 2) {
      steering_control();
    }
    
    //METTERE SWITCH
    //if(componentId == 0)
    //{ //cambiare speed con component value, togliere sopra speed
  }

  //delay(20);
}
