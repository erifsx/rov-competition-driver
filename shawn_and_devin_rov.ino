// this project depends on arduinojson https://arduinojson.org/v6/doc/installation/
#include <ArduinoJson.h>

#include <math.h>
#define FRONT_LEFT_INPUT    5
#define FRONT_RIGHT_INPUT   7
#define TOP_RIGHT_INPUT     8
#define TOP_LEFT_INPUT      4
#define BOTTOM_RIGHT_INPUT  6
#define BOTTOM_LEFT_INPUT   2
#define BACK_RIGHT_INPUT    9
#define BACK_LEFT_INPUT     3

#define INPUT_LEFT_STICK_X  A4
#define INPUT_LEFT_STICK_Y  A3
#define INPUT_RIGHT_STICK_X A2
#define INPUT_RIGHT_STICK_Y A1


#define STOPPED_MOTOR_SPEED 255
#define FULL_MOTOR_SPEED    0
#define DEADBAND            0.07f
float sin45 = sin(45.0 * (PI/180.0));
float cos45 = cos(45.0 * (PI/180.0));


typedef struct joy {
  float x;
  float y;
} Joy;

typedef struct holonomic_speeds {
  float top_right;
  float top_left;
  float bottom_right;
  float bottom_left;
} HolonomicSpeeds;

Joy left_stick;
Joy right_stick;


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(FRONT_LEFT_INPUT, OUTPUT);
  pinMode(FRONT_RIGHT_INPUT, OUTPUT);
  pinMode(TOP_RIGHT_INPUT, OUTPUT);
  pinMode(TOP_LEFT_INPUT, OUTPUT);
  pinMode(BOTTOM_RIGHT_INPUT, OUTPUT);
  pinMode(BOTTOM_LEFT_INPUT, OUTPUT);
  pinMode(BACK_RIGHT_INPUT, OUTPUT);
  pinMode(BACK_LEFT_INPUT, OUTPUT);
  Serial.begin(9600);

  left_stick.x = 0.f;
  left_stick.y = 0.f;
  right_stick.x = 0.f;
  right_stick.y = 0.f;
}

float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float clamp(float x) {
  return max(-1.0, min(1.0, x));
}

const float MAX_STICK = (1024.0/5.0 * 3.3);
const float CENTER_STICK = (MAX_STICK /2.0);

float map_analog_to_joy(float val) {
  val = max(-CENTER_STICK, min(CENTER_STICK, ((val - CENTER_STICK)))) / CENTER_STICK;
  if(abs(val) < DEADBAND) val = 0.0;
  return val;
}

// reads joystick values from serial in format of "j1x j1y j2x j2y"
// and assigns them to the corresponding globals
// and are values between [-1024, 1024]
void get_last_command() {
    left_stick.x = map_analog_to_joy(analogRead(INPUT_LEFT_STICK_X));
    left_stick.y = map_analog_to_joy(analogRead(INPUT_LEFT_STICK_Y));
    right_stick.x = map_analog_to_joy(analogRead(INPUT_RIGHT_STICK_X));
    right_stick.y = map_analog_to_joy(analogRead(INPUT_RIGHT_STICK_Y));
//    
}



/**
 * Sets the speed of motors m1 and m2 where m1 is the forward motor and m2 is the reverse motor
 * to a speed where speed is between -1 and 1
 */
void set_joined_motor_speed(int m1, int m2, float speed) {
  int m1Speed = STOPPED_MOTOR_SPEED;
  int m2Speed = STOPPED_MOTOR_SPEED; 
  
  if(speed == 0.0) {
  } else if(speed >= 0.f) {
    m1Speed = (int) map_float(speed, 0.f, 1.f, STOPPED_MOTOR_SPEED, FULL_MOTOR_SPEED);
  } else {
    speed = -speed;
    m2Speed = (int) map_float(speed, 0.f, 1.f, STOPPED_MOTOR_SPEED, FULL_MOTOR_SPEED);
  }

  //char buffer[100];
  //sprintf(buffer, "%d: %d, %d: %d\n", m1, m1Speed, m2, m2Speed);
  //Serial.println(buffer);
  analogWrite(m1, m1Speed);
  analogWrite(m2, m2Speed);
}

void update_forward_differential_arcade_drive() {
  // derived from https://github.com/wpilibsuite/allwpilib/blob/master/wpilibc/src/main/native/cpp/drive/DifferentialDrive.cpp::arcadeDrive
  float xSpeed = clamp(right_stick.y);
  float zRotation = clamp(right_stick.x);
  float leftMotorOutput;
  float rightMotorOutput;

  float maxInput = abs(max(abs(xSpeed), abs(zRotation))) * (xSpeed >= 0.0f ? 1.0f : -1.0f);

  if (xSpeed >= 0.0) {
    // First quadrant, else second quadrant
    if (zRotation >= 0.0) {
      leftMotorOutput = maxInput;
      rightMotorOutput = xSpeed - zRotation;
    } else {
      leftMotorOutput = xSpeed + zRotation;
      rightMotorOutput = maxInput;
    }
    } else {
    // Third quadrant, else fourth quadrant
    if (zRotation >= 0.0) {
      leftMotorOutput = xSpeed + zRotation;
      rightMotorOutput = maxInput;
    } else {
      leftMotorOutput = maxInput;
      rightMotorOutput = xSpeed - zRotation;
    }
  }

  set_joined_motor_speed(FRONT_LEFT_INPUT, BACK_LEFT_INPUT, clamp(leftMotorOutput));
  set_joined_motor_speed(FRONT_RIGHT_INPUT, BACK_RIGHT_INPUT, clamp(rightMotorOutput));
}

void update_vertical_holonomic_drive() {
  float x = clamp(left_stick.x);
  float y = clamp(left_stick.y);

  float rotated_x_axis = (cos45 * x) + (sin45 * y);
  float rotated_y_axis = (-sin45 * x) + (sin45 * y);

  set_joined_motor_speed(TOP_RIGHT_INPUT, BOTTOM_LEFT_INPUT, rotated_x_axis);
  set_joined_motor_speed(TOP_LEFT_INPUT, BOTTOM_RIGHT_INPUT, rotated_y_axis);
}



// the loop function runs over and over again forever
void loop() {
  get_last_command();
  update_vertical_holonomic_drive();
  update_forward_differential_arcade_drive();
}
