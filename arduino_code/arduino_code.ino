#include <Servo.h>
#include <NewPing.h>

#define ENGINE_ID 1
#define SERVO_ID 2

#define TRIG_PIN 12
#define ECHO_PIN 11
#define MAX_CM_DIST 80

#define RELAY_PIN 3

#define STEERING 9

#define INPUT_SIZE 5

Servo servo;  // create servo object to control a servo

//ultrasonic sensor
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_CM_DIST);
long distance;

char input[INPUT_SIZE + 1];
char read_byte, startMarker = '<', endMarker = '>';
char * separator;
int componentId, componentValue;

int i = 0;
bool change = false, getting = false;

void get_input() {
  if (Serial.available() > 0 && change != true) {
    read_byte = Serial.read();
    
    if(getting == true) {
      //if the message is bigger than our buffer we cancel it
      if(i > INPUT_SIZE) {
        i = 0;
        getting = false;
      }
      
      if(read_byte != endMarker) {
        input[i] = read_byte;
        i++;
      } else {
        // Add the final 0 to end the C string
        input[i] = 0;

        parse_command();

        //ready to receive a new command
        getting = false;
        i = 0;
      }
    } else if(read_byte == startMarker) {
      //we start storing the command
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
}

void motor_control() {

  if(componentValue == 0) {
    digitalWrite(RELAY_PIN, LOW);   // turn the relay off
  } else {
    digitalWrite(RELAY_PIN, HIGH);  // turn the relay on
  }
}

void steering_control() {
  servo.write(componentValue); 
}

void setup() {
  //motor ports
  pinMode(RELAY_PIN, OUTPUT);

  //steering port
  servo.attach(STEERING);
  
  Serial.begin(57600);
  Serial.setTimeout(50);
}

void loop() {

  //sensor output part
  distance = sonar.ping_cm();
  if(distance != 0) //0 is a 'bad' value, that we will ignore
    Serial.println(distance);
  
  //actuators input part
  get_input();

  if(change == true) {
    change = false;

    if(componentId == ENGINE_ID) {
      motor_control();
    } else if(componentId == SERVO_ID) {
      steering_control();
    }
  }

  delay(10);
}
