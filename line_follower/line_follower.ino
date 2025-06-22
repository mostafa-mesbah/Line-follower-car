// Motor driver pins
#define EN_A 10  // PWM pin for left motor
#define EN_B 11  // PWM pin for right motor
#define IN1 7    // Left motor forward
#define IN2 6    // Left motor backward
#define IN3 5    // Right motor forward
#define IN4 4    // Right motor backward

// Sensor pins
#define LEFT_SENSOR A0
#define MIDDLE_SENSOR A1
#define RIGHT_SENSOR A2

// Sensor thresholds (adjust based on calibration)
// HIGH values = dark/black (line), LOW values = white (background)
#define MIDDLE_THRESHOLD 600  // If reading > this, middle sensor sees line
#define MIN_MID_ERROR 460
// PID constants (tune these for optimal performance)
float kp = 1.0;  // Proportional gain
float ki = 1.5;   // Integral gain
float kd = 0.5;   // Derivative gain

// PID variables
int error_direction=0;
float accumulated_error = 0;
float max_accumulated_error = 255;
int current_error = 0;
int previous_error = 0;
unsigned long previous_time = 0;

// Function declarations
int calculate_error(int middle_reading);
unsigned long calculate_delta_time();
float calculate_p_gain(int error, float kp);
float calculate_i_gain(int error, float ki, unsigned long dt);
float calculate_d_gain(int error, int prev_error, float kd, unsigned long dt);
void moveForward(int speed);
void turnRight(int speed);
void turnLeft(int speed);
void stopMotors();

void setup() {
  // Initialize motor control pins
  pinMode(EN_A, OUTPUT);
  pinMode(EN_B, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize serial communication
  Serial.begin(9600);
  delay(3000);
  Serial.println("Line Follower Robot Initialized");
}

void loop() {
  // Read sensor values
  int left = analogRead(LEFT_SENSOR);
  int middle = analogRead(MIDDLE_SENSOR);
  int right = analogRead(RIGHT_SENSOR);

  // Calculate error
  current_error = calculate_error(middle);
  unsigned long delta_time = calculate_delta_time();

  // Calculate PID gains
  float p_gain = calculate_p_gain(current_error, kp);
  float i_gain = calculate_i_gain(current_error, ki, delta_time);
  float d_gain = calculate_d_gain(current_error, previous_error, kd, delta_time);

  // Calculate total gain
  int total_gain_val = constrain((int)(p_gain + i_gain + d_gain), -100, 100);
  if(abs(current_error) >MIN_MID_ERROR ){
    if(error_direction >0){
      turnLeft(constrain(total_gain_val,0,70));
    }
    else if(error_direction <0){
      turnRight(constrain(total_gain_val,0,70));
    }
  }
  else if(abs(current_error) < MIN_MID_ERROR ){
    moveForward(constrain(total_gain_val,50,65));
  }

  // Update previous error
  previous_error = current_error;

  // Debug output
  Serial.print("Left: ");
  Serial.print(left);
  Serial.print(" | Middle: ");
  Serial.print(middle);
  Serial.print(" | Right: ");
  Serial.print(right);
  Serial.print(" | Error: ");
  Serial.print(current_error);
  Serial.print(" | Gain: ");
  Serial.println(total_gain_val);

  delay(80);
}

int calculate_error(int middle_reading) {
  int error = MIDDLE_THRESHOLD-middle_reading;
  int left = analogRead(LEFT_SENSOR);
  int right = analogRead(RIGHT_SENSOR);
  if (left > right){
    //the car is shifted right so the it must spin left to get on the line
    error_direction=1;

  }
  else if(left < right){
    error_direction=-1;
  }
  return error;
  }
  

unsigned long calculate_delta_time() {
    unsigned long current_time = micros();
    unsigned long dt = (current_time >= previous_time) ? 
                       (current_time - previous_time) : 
                       (UINT32_MAX - previous_time + current_time + 1);
    previous_time = current_time;
    return dt;
}

float calculate_p_gain(int error, float kp) {
  return error * kp;
}

float calculate_i_gain(int error, float ki, unsigned long dt) {
  if (dt > 0) {
    accumulated_error += error * (dt / 1e6);
    accumulated_error = constrain(accumulated_error, -max_accumulated_error, max_accumulated_error);
  }
  return accumulated_error * ki;
}

float calculate_d_gain(int error, int prev_error, float kd, unsigned long dt) {
  if (dt > 0) {
    return kd * (error - prev_error) / (dt / 1e6);
  }
  return 0;
}

// Motor control functions
void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN_A, speed);
  analogWrite(EN_B, speed);
}

void turnRight(int speed) {
  // Right turn - left motor forward, right motor backward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(EN_A, speed);
  analogWrite(EN_B, speed);
}

void turnLeft(int speed) {
  // Left turn - left motor backward, right motor forward
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(EN_A, speed);
  analogWrite(EN_B, speed);

}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(EN_A, 0);
  analogWrite(EN_B, 0);
}