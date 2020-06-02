#include <Servo.h>
#include <NewPing.h>

#define ENGINE_ID 1
#define SERVO_ID 2

#define TRIG_PIN 4
#define ECHO_PIN 5
#define MAX_CM_DIST 80

#define RELAY_PIN 3

#define STEERING 9
#define SERVO_RANGE 80
#define SERVO_CENTER 90

#define INPUT_SIZE 10

Servo servo;  // create servo object to control a servo

//ultrasonic sensor
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_CM_DIST);
long distance;


char input[INPUT_SIZE];
byte size;
char read_byte;
const char startMarker = '<', endMarker = '>';
char * separator;
int componentId, componentValue;

int i = 0, speed = 0;
bool direction = false, change = false, getting = false;

void get_input() {
  while (Serial.available() > 0 && change != true) {
    read_byte = Serial.read();
    //Serial.println(read_byte);

    if(getting == true) {
      
      //if the message is bigger than our buffer we cancel it
      if(i > INPUT_SIZE - 1) {
        i = 0;
        getting = false;
        
        //Serial.println("Non dovrebbe essere qua\n");
      }

      if((read_byte >= '0' && read_byte <= '9') || read_byte == ':') {
        input[i] = read_byte;
        i++;
        //Serial.println("Input[");
        //Serial.println(i);
        //Serial.println("] inserito\n");
      } else if(read_byte == endMarker) {
        //we stop storing the command
        // Add the final 0 to end the C string
        input[i] = 0;
        //Serial.println("parsiaml\n");

        //parsing the command to get the information
        parse_command();

        getting = false;
        i = 0;
      }
    }
  
    if(read_byte == startMarker) {
        //we start storing the command
        //Serial.println("inizio comando\n");
        getting = true;
        i = 0;
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
    //Serial.println("trovato");
    //Serial.println(componentId);
    //Serial.println(componentValue);
}

void motor_control() {

  if(componentValue == 0) {
    digitalWrite(RELAY_PIN, LOW);   // turn the relay on (HIGH is the voltage level)
  } else {
    digitalWrite(RELAY_PIN, HIGH);   // turn the relay on (HIGH is the voltage level)
  }
}

void steering_control() {
  if(componentValue >= SERVO_CENTER - (SERVO_RANGE / 2) && componentValue <= SERVO_CENTER + (SERVO_RANGE / 2)) {
    servo.write(componentValue); 
  }
}

//core

void setup() {
  //motor ports
  pinMode(RELAY_PIN, OUTPUT);

  //steering port
  servo.attach(STEERING);

  
  Serial.begin(9600);
  Serial.setTimeout(50);
}

void loop() {

  //sensor output part
  distance = sonar.ping_cm();
  if(Serial.availableForWrite() > 4) 
  {
    if(distance == 0) //0 is set when the closest obstacle is too far
      Serial.println(MAX_CM_DIST);
    else
      Serial.println(distance);
  }
  
  //actuators input part
  get_input();

  if(change == true) {
    change = false;
    //Serial.println("faccio");
    if(componentId == ENGINE_ID) {
      //Serial.println("motore\n");
      motor_control();
    } else if(componentId == SERVO_ID) {
      steering_control();
    }
    //Serial.println("Ricevuto inputs");
    //Serial.println(componentId);
    //Serial.println(componentValue);
    //METTERE SWITCH
    //if(componentId == 0)
    //{ //cambiare speed con component value, togliere sopra speed
  }

  delay(30);
}
