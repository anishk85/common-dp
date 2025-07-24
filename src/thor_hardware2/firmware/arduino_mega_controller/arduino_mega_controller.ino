FILE: firmware/arduino_mega_controller/arduino_mega_controller.ino
================================================================================
/*
 * Thor Hardware2 - Arduino Mega Controller Firmware
 * Controls 6x goBILDA 5203 motors via 3x TB6612FNG drivers
 * Communicates with Raspberry Pi via Serial
 */

#include <PID_v1.h>

// Communication protocol
const byte START_BYTE = 0xAA;
const byte END_BYTE = 0x55;
const int SERIAL_BAUD = 115200;
const int PACKET_TIMEOUT = 100; // ms

// Command types
enum CommandType {
  CMD_MOTOR_CONTROL = 0x01,
  CMD_READ_SENSORS = 0x02,
  CMD_EMERGENCY_STOP = 0x03,
  CMD_SYSTEM_RESET = 0x04,
  CMD_HEARTBEAT = 0x05,
  CMD_GET_STATUS = 0x06
};

// Motor configuration for goBILDA 5203
struct Motor {
  // TB6612FNG pins
  int pwm_pin;
  int dir_pin1;
  int dir_pin2;
  int stby_pin;
  
  // Encoder pins
  int encoder_a;
  int encoder_b;
  
  // Control variables
  double position;        // radians
  double velocity;        // rad/s
  double target_position; // radians
  double target_velocity; // rad/s
  double pwm_output;      // -255 to 255
  
  // PID controllers
  PID* position_pid;
  PID* velocity_pid;
  
  // Encoder
  volatile long encoder_count;
  double last_position;
  unsigned long last_time;
  
  // Status
  bool enabled;
  bool fault;
  double current;
};

// Motor array - 6 motors for 6-DOF arm
Motor motors[6];

// TB6612FNG driver configuration (3 drivers, 2 motors each)
// Driver 1: Motors 0,1 - Base rotation, Shoulder
// Driver 2: Motors 2,3 - Elbow, Wrist pitch  
// Driver 3: Motors 4,5 - Wrist roll, Gripper

void setup() {
  Serial.begin(SERIAL_BAUD);
  
  // Initialize motors
  initializeMotors();
  
  // Setup interrupts for encoders
  setupEncoderInterrupts();
  
  // Initialize PID controllers
  setupPIDControllers();
  
  Serial.println("Thor Hardware2 Controller Ready");
}

void loop() {
  // Check for incoming commands
  if (Serial.available()) {
    processSerialCommand();
  }
  
  // Update motor control
  updateMotorControl();
  
  // Safety checks
  performSafetyChecks();
  
  delay(1); // 1ms loop time
}

void initializeMotors() {
  // Motor 0 - Base rotation (Driver 1A)
  motors[0] = {3, 2, 4, 9, 18, 19, 0, 0, 0, 0, 0, nullptr, nullptr, 0, 0, 0, false, false, 0};
  
  // Motor 1 - Shoulder (Driver 1B) 
  motors[1] = {5, 7, 8, 9, 20, 21, 0, 0, 0, 0, 0, nullptr, nullptr, 0, 0, 0, false, false, 0};
  
  // Motor 2 - Elbow (Driver 2A)
  motors[2] = {6, 22, 24, 10, 26, 28, 0, 0, 0, 0, 0, nullptr, nullptr, 0, 0, 0, false, false, 0};
  
  // Motor 3 - Wrist pitch (Driver 2B)
  motors[3] = {11, 30, 32, 10, 34, 36, 0, 0, 0, 0, 0, nullptr, nullptr, 0, 0, 0, false, false, 0};
  
  // Motor 4 - Wrist roll (Driver 3A)  
  motors[4] = {12, 38, 40, 13, 42, 44, 0, 0, 0, 0, 0, nullptr, nullptr, 0, 0, 0, false, false, 0};
  
  // Motor 5 - Gripper (Driver 3B)
  motors[5] = {46, 48, 50, 13, 52, 53, 0, 0, 0, 0, 0, nullptr, nullptr, 0, 0, 0, false, false, 0};
  
  // Configure pins
  for (int i = 0; i < 6; i++) {
    pinMode(motors[i].pwm_pin, OUTPUT);
    pinMode(motors[i].dir_pin1, OUTPUT);
    pinMode(motors[i].dir_pin2, OUTPUT);
    pinMode(motors[i].stby_pin, OUTPUT);
    pinMode(motors[i].encoder_a, INPUT_PULLUP);
    pinMode(motors[i].encoder_b, INPUT_PULLUP);
    
    // Enable drivers
    digitalWrite(motors[i].stby_pin, HIGH);
    
    // Initialize encoder count
    motors[i].encoder_count = 0;
    motors[i].last_time = millis();
  }
}

void setupEncoderInterrupts() {
  // Attach interrupts for encoder A pins
  attachInterrupt(digitalPinToInterrupt(motors[0].encoder_a), encoder0ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[1].encoder_a), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[2].encoder_a), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[3].encoder_a), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[4].encoder_a), encoder4ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[5].encoder_a), encoder5ISR, CHANGE);
}

void setupPIDControllers() {
  // PID parameters for goBILDA 5203 motors
  double kp = 2.0, ki = 0.1, kd = 0.01;
  
  for (int i = 0; i < 6; i++) {
    motors[i].position_pid = new PID(&motors[i].position, &motors[i].pwm_output, 
                                    &motors[i].target_position, kp, ki, kd, DIRECT);
    motors[i].position_pid->SetMode(AUTOMATIC);
    motors[i].position_pid->SetOutputLimits(-255, 255);
    motors[i].position_pid->SetSampleTime(1);
  }
}

void processSerialCommand() {
  if (Serial.read() != START_BYTE) return;
  
  byte cmd_type = Serial.read();
  byte data_length = Serial.read();
  
  byte data[128];
  Serial.readBytes(data, data_length);
  
  byte checksum = Serial.read();
  byte end_byte = Serial.read();
  
  if (end_byte != END_BYTE) return;
  
  // Verify checksum
  byte calc_checksum = 0;
  for (int i = 0; i < data_length; i++) {
    calc_checksum ^= data[i];
  }
  if (calc_checksum != checksum) return;
  
  // Process command
  switch (cmd_type) {
    case CMD_MOTOR_CONTROL:
      handleMotorControlCommand(data, data_length);
      break;
    case CMD_READ_SENSORS:
      handleReadSensorsCommand();
      break;
    case CMD_EMERGENCY_STOP:
      handleEmergencyStop();
      break;
    case CMD_SYSTEM_RESET:
      handleSystemReset();
      break;
    case CMD_HEARTBEAT:
      handleHeartbeat();
      break;
    case CMD_GET_STATUS:
      handleGetStatus();
      break;
  }
}

void handleMotorControlCommand(byte* data, byte length) {
  if (length != 6 * 10) return; // 6 motors * 10 bytes each
  
  int offset = 0;
  for (int i = 0; i < 6; i++) {
    int motor_id = data[offset++];
    if (motor_id != i) continue;
    
    // Extract position (4 bytes)
    float pos;
    memcpy(&pos, &data[offset], 4);
    motors[i].target_position = pos;
    offset += 4;
    
    // Extract velocity (4 bytes)
    float vel;
    memcpy(&vel, &data[offset], 4);
    motors[i].target_velocity = vel;
    offset += 4;
    
    // Extract enable flag
    motors[i].enabled = (data[offset++] != 0);
  }
}

void handleReadSensorsCommand() {
  // Send motor states back
  byte response[6 * 17]; // 6 motors * 17 bytes each
  int offset = 0;
  
  for (int i = 0; i < 6; i++) {
    // Update position from encoder
    updateMotorPosition(i);
    
    response[offset++] = i; // Motor ID
    
    // Position (4 bytes)
    float pos = motors[i].position;
    memcpy(&response[offset], &pos, 4);
    offset += 4;
    
    // Velocity (4 bytes)
    float vel = motors[i].velocity;
    memcpy(&response[offset], &vel, 4);
    offset += 4;
    
    // Current (4 bytes) - simulated for now
    float current = motors[i].current;
    memcpy(&response[offset], &current, 4);
    offset += 4;
    
    // Encoder count (4 bytes)
    int32_t encoder = motors[i].encoder_count;
    memcpy(&response[offset], &encoder, 4);
    offset += 4;
    
    // Fault flag
    response[offset++] = motors[i].fault ? 1 : 0;
  }
  
  sendResponse(response, sizeof(response));
}

void handleEmergencyStop() {
  for (int i = 0; i < 6; i++) {
    motors[i].enabled = false;
    motors[i].pwm_output = 0;
    setMotorPWM(i, 0);
  }
  
  byte response = 1; // Success
  sendResponse(&response, 1);
}

void handleSystemReset() {
  // Reset all motor states
  for (int i = 0; i < 6; i++) {
    motors[i].encoder_count = 0;
    motors[i].position = 0;
    motors[i].velocity = 0;
    motors[i].target_position = 0;
    motors[i].target_velocity = 0;
    motors[i].enabled = false;
    motors[i].fault = false;
    motors[i].pwm_output = 0;
    setMotorPWM(i, 0);
  }
  
  byte response = 1; // Success
  sendResponse(&response, 1);
}

void handleHeartbeat() {
  byte response = 1; // Alive
  sendResponse(&response, 1);
}

void handleGetStatus() {
  // Send system status
  byte status[8];
  status[0] = 1; // System OK
  
  // Voltage (2 bytes) - simulated
  uint16_t voltage = 5000; // 5.0V in mV
  memcpy(&status[1], &voltage, 2);
  
  // Current (2 bytes) - simulated
  uint16_t current = 1500; // 1.5A in mA
  memcpy(&status[3], &current, 2);
  
  // Temperature (2 bytes) - simulated
  uint16_t temp = 2500; // 25.0°C in 0.1°C
  memcpy(&status[5], &temp, 2);
  
  status[7] = 0; // No errors
  
  sendResponse(status, sizeof(status));
}

void sendResponse(byte* data, int length) {
  Serial.write(START_BYTE);
  Serial.write((byte)length);
  Serial.write(data, length);
  
  // Calculate checksum
  byte checksum = 0;
  for (int i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  Serial.write(checksum);
  Serial.write(END_BYTE);
}

void updateMotorControl() {
  for (int i = 0; i < 6; i++) {
    if (!motors[i].enabled) {
      setMotorPWM(i, 0);
      continue;
    }
    
    // Update position from encoder
    updateMotorPosition(i);
    
    // Run PID controller
    motors[i].position_pid->Compute();
    
    // Apply PWM output
    setMotorPWM(i, (int)motors[i].pwm_output);
    
    // Simulate current measurement
    motors[i].current = abs(motors[i].pwm_output) / 255.0 * 2.0; // 0-2A range
  }
}

void updateMotorPosition(int motor_id) {
  // Convert encoder counts to radians (1440 CPR for goBILDA 5203)
  motors[motor_id].position = (double(motors[motor_id].encoder_count) / 1440.0) * 2.0 * PI;
  
  // Calculate velocity
  unsigned long current_time = millis();
  double dt = (current_time - motors[motor_id].last_time) / 1000.0;
  if (dt > 0) {
    motors[motor_id].velocity = (motors[motor_id].position - motors[motor_id].last_position) / dt;
  }
  
  motors[motor_id].last_position = motors[motor_id].position;
  motors[motor_id].last_time = current_time;
}

void setMotorPWM(int motor_id, int pwm_value) {
  // Clamp PWM value
  pwm_value = constrain(pwm_value, -255, 255);
  
  if (pwm_value > 0) {
    // Forward direction
    digitalWrite(motors[motor_id].dir_pin1, HIGH);
    digitalWrite(motors[motor_id].dir_pin2, LOW);
    analogWrite(motors[motor_id].pwm_pin, pwm_value);
  } else if (pwm_value < 0) {
    // Reverse direction
    digitalWrite(motors[motor_id].dir_pin1, LOW);
    digitalWrite(motors[motor_id].dir_pin2, HIGH);
    analogWrite(motors[motor_id].pwm_pin, -pwm_value);
  } else {
    // Stop
    digitalWrite(motors[motor_id].dir_pin1, LOW);
    digitalWrite(motors[motor_id].dir_pin2, LOW);
    analogWrite(motors[motor_id].pwm_pin, 0);
  }
}

void performSafetyChecks() {
  for (int i = 0; i < 6; i++) {
    // Check for overcurrent
    if (motors[i].current > 3.0) { // 3A limit
      motors[i].fault = true;
      motors[i].enabled = false;
      setMotorPWM(i, 0);
    }
    
    // Check position limits (basic range check)
    if (abs(motors[i].position) > 4.0) { // ~2.3 revolutions
      motors[i].fault = true;
      motors[i].enabled = false;
      setMotorPWM(i, 0);
    }
  }
}

// Encoder interrupt service routines
void encoder0ISR() { updateEncoder(0); }
void encoder1ISR() { updateEncoder(1); }
void encoder2ISR() { updateEncoder(2); }
void encoder3ISR() { updateEncoder(3); }
void encoder4ISR() { updateEncoder(4); }
void encoder5ISR() { updateEncoder(5); }

void updateEncoder(int motor_id) {
  bool a = digitalRead(motors[motor_id].encoder_a);
  bool b = digitalRead(motors[motor_id].encoder_b);
  
  // Quadrature decoding
  if (a == b) {
    motors[motor_id].encoder_count++;
  } else {
    motors[motor_id].encoder_count--;
  }
}
