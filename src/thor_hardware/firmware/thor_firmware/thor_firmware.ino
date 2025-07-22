/* File: thor_hardware/firmware/thor_firmware/thor_firmware.ino
========================================================================
This is the firmware that runs on the Arduino Mega. It reads encoders,
runs PID loops for each motor, and communicates with the Raspberry Pi.
*/

// --- Pin Definitions ---
// IMPORTANT: These are example pins. You MUST update them to match your wiring.
// The Arduino Mega has enough interrupt-capable pins for all encoders.
const int MOTOR_PINS[6][3] = {
  // PWM, DIR1, DIR2
  {2, 22, 23}, // Joint 1
  {3, 24, 25}, // Joint 2
  {4, 26, 27}, // Joint 3
  {5, 28, 29}, // Joint 4
  {6, 30, 31}, // Joint 5
  {7, 32, 33}  // Joint 6
};

const int ENCODER_PINS[6][2] = {
  // Channel A, Channel B (must be interrupt pins)
  {18, 19}, // Joint 1
  {20, 21}, // Joint 2
  {2, 3},   // Joint 3 (Using external interrupts 0 and 1)
  {17, 16}, // Joint 4
  {15, 14}, // Joint 5
  {38, 39}  // Joint 6
};

// --- PID Control ---
// You will need to tune these values for your specific robot arm!
double Kp = 5.0, Ki = 0.1, Kd = 0.5;

struct PIDController {
  double setpoint = 0.0;
  double integral = 0.0;
  double prev_error = 0.0;
};

PIDController pids[6];

// --- Encoder State ---
// volatile is crucial because these are modified in interrupt service routines
volatile long encoder_ticks[6] = {0, 0, 0, 0, 0, 0};

// --- Communication ---
const int BAUD_RATE = 57600;
unsigned long last_report_time = 0;
const int REPORT_INTERVAL_MS = 20; // Send position data 50 times/sec

void setup() {
  Serial.begin(BAUD_RATE);

  // Configure Motor Pins
  for (int i = 0; i < 6; i++) {
    pinMode(MOTOR_PINS[i][0], OUTPUT); // PWM
    pinMode(MOTOR_PINS[i][1], OUTPUT); // DIR1
    pinMode(MOTOR_PINS[i][2], OUTPUT); // DIR2
  }

  // Configure Encoder Pins and Interrupts
  for (int i = 0; i < 6; i++) {
    pinMode(ENCODER_PINS[i][0], INPUT_PULLUP);
    pinMode(ENCODER_PINS[i][1], INPUT_PULLUP);
  }
  
  // Attach interrupts for high-performance encoder reading
  // This is a simplified example. For 6 encoders, you would use a library
  // or more advanced pin change interrupt handling.
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[0][0]), readEncoder0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PINS[1][0]), readEncoder1, CHANGE);
  // ... and so on for the other encoders
}

void loop() {
  // 1. Check for incoming commands from Raspberry Pi
  if (Serial.available() > 0) {
    parseSerialCommand();
  }

  // 2. Update all PID controllers and drive motors
  for (int i = 0; i < 6; i++) {
    updatePID(i);
  }

  // 3. Periodically send current positions back to Raspberry Pi
  if (millis() - last_report_time > REPORT_INTERVAL_MS) {
    reportPositions();
    last_report_time = millis();
  }
}

void parseSerialCommand() {
  String command = Serial.readStringUntil('\n');
  if (command.startsWith("s ")) {
    // This is a setpoint command: "s j1 j2 j3 j4 j5 j6"
    // The values are in radians. We need to convert them to ticks.
    // NOTE: This parsing is simplified. A robust version would handle errors.
    command.remove(0, 2); // Remove "s "
    
    double rads_to_ticks = (28.0 * 100.0) / (2 * M_PI); // (CPR * GearRatio) / 2PI

    char* str = (char*)command.c_str();
    char* token = strtok(str, " ");
    for (int i = 0; i < 6 && token != NULL; i++) {
      double rad_setpoint = atof(token);
      pids[i].setpoint = rad_setpoint * rads_to_ticks;
      token = strtok(NULL, " ");
    }
  }
}

void updatePID(int joint_index) {
  long current_pos = encoder_ticks[joint_index];
  double error = pids[joint_index].setpoint - current_pos;

  pids[joint_index].integral += error;
  double derivative = error - pids[joint_index].prev_error;
  
  double output = Kp * error + Ki * pids[joint_index].integral + Kd * derivative;
  
  pids[joint_index].prev_error = error;

  // Drive the motor based on the PID output
  driveMotor(joint_index, output);
}

void driveMotor(int joint_index, double speed) {
  // Constrain speed to PWM range
  speed = constrain(speed, -255, 255);
  
  if (speed > 0) {
    digitalWrite(MOTOR_PINS[joint_index][1], HIGH);
    digitalWrite(MOTOR_PINS[joint_index][2], LOW);
  } else {
    digitalWrite(MOTOR_PINS[joint_index][1], LOW);
    digitalWrite(MOTOR_PINS[joint_index][2], HIGH);
  }
  analogWrite(MOTOR_PINS[joint_index][0], abs(speed));
}

void reportPositions() {
  Serial.print("p ");
  for (int i = 0; i < 6; i++) {
    Serial.print(encoder_ticks[i]);
    Serial.print(" ");
  }
  Serial.println();
}

// --- Interrupt Service Routines for Encoders ---
// This is a simplified example for the first two encoders.
// A full implementation would handle all six.
void readEncoder0() {
  int b = digitalRead(ENCODER_PINS[0][1]);
  if (b > 0) {
    encoder_ticks[0]++;
  } else {
    encoder_ticks[0]--;
  }
}

void readEncoder1() {
  int b = digitalRead(ENCODER_PINS[1][1]);
  if (b > 0) {
    encoder_ticks[1]++;
  } else {
    encoder_ticks[1]--;
  }
}