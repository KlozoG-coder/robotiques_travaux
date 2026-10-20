
#include <MeAuriga.h>

// Components
MeEncoderOnBoard leftMotor(SLOT1);
MeEncoderOnBoard rightMotor(SLOT2);
MeGyro gyro(0, 0x69);

// Constants
const unsigned long START_DELAY = 3000;
const float TARGET_DISTANCE = 100.0;
const int MOTOR_SPEED = 100;
const int TURN_SPEED = 80;

const float WHEEL_DIAMETER = 6.5;
const float GEAR_RATIO = 39.267;
const int MOTOR_PULSES = 9;

// States
enum AppState {
  WAIT_FOR_START,
  MOVE_FORWARD,
  TURN_LEFT,
  STOP
};

AppState appState = WAIT_FOR_START;

unsigned long currentTime = 0;

// Encoder interrupts
void leftMotorInterrupt() {
  if (digitalRead(leftMotor.getPortB()) == 0) {
    leftMotor.pulsePosMinus();
  } else {
    leftMotor.pulsePosPlus();
  }
}

void rightMotorInterrupt() {
  if (digitalRead(rightMotor.getPortB()) == 0) {
    rightMotor.pulsePosMinus();
  } else {
    rightMotor.pulsePosPlus();
  }
}

// Calculate encoder ticks
long calculateTicks(float distanceCm) {
  float ticksPerTurn = MOTOR_PULSES * GEAR_RATIO;
  float circumference = WHEEL_DIAMETER * PI;
  float numberOfTurns = distanceCm / circumference;

  return round(ticksPerTurn * numberOfTurns);
}

// Stop both motors
void stopMotors() {
  leftMotor.setMotorPwm(0);
  rightMotor.setMotorPwm(0);
}

// State 1: Wait 3 seconds
void waitForStartState(unsigned long ct) {

  static unsigned long lastTime = 0;
  static bool firstTime = true;

  if (firstTime) {
    stopMotors();
    lastTime = ct;

    Serial.println("Waiting 3 seconds");

    firstTime = false;
    return;
  }

  // TRANSITION
  if (ct - lastTime >= START_DELAY) {
    firstTime = true;
    appState = MOVE_FORWARD;
  }
}

// State 2: Move forward
void moveForwardState(unsigned long ct) {

  static bool firstTime = true;
  static long startLeftTicks = 0;
  static long startRightTicks = 0;

  if (firstTime) {
    startLeftTicks = leftMotor.getPulsePos();
    startRightTicks = rightMotor.getPulsePos();

    gyro.resetData();

    Serial.println("Moving forward");

    firstTime = false;
    return;
  }

  // EXECUTION
  long leftTicks = abs(leftMotor.getPulsePos() - startLeftTicks);

  long rightTicks = abs(rightMotor.getPulsePos() - startRightTicks);

  long averageTicks = (leftTicks + rightTicks) / 2;
  long targetTicks = calculateTicks(TARGET_DISTANCE);

  // TRANSITION
  if (averageTicks >= targetTicks) {
    stopMotors();
    firstTime = true;
    appState = TURN_LEFT;
    return;
  }

  // Gyro correction
  float correction = gyro.getAngleZ() * 3.0;

  int leftSpeed = constrain(MOTOR_SPEED - correction, 0, 255);
  int rightSpeed = constrain(MOTOR_SPEED + correction, 0, 255);

  leftMotor.setMotorPwm(-leftSpeed);
  rightMotor.setMotorPwm(rightSpeed);
}

// State 3: Turn left 90 degrees
void turnLeftState(unsigned long ct) {

  static bool firstTime = true;

  if (firstTime) {
    stopMotors();
    gyro.resetData();

    Serial.println("Turning left 90 degrees");

    firstTime = false;
    return;
  }

  // EXECUTION
  float currentAngle = gyro.getAngleZ();

  // TRANSITION
  if (abs(currentAngle) >= 90.0) {
    stopMotors();
    firstTime = true;
    appState = STOP;
    return;
  }

  // Turn in place
  leftMotor.setMotorPwm(TURN_SPEED);
  rightMotor.setMotorPwm(TURN_SPEED);
}

// Final stop
void stopState(unsigned long ct) {
  stopMotors();
}

// State manager
void stateManager(unsigned long ct) {

  switch (appState) {

    case WAIT_FOR_START:
      waitForStartState(ct);
      break;

    case MOVE_FORWARD:
      moveForwardState(ct);
      break;

    case TURN_LEFT:
      turnLeftState(ct);
      break;

    case STOP:
      stopState(ct);
      break;
  }
}

void setup() {

  Serial.begin(115200);

  leftMotor.setPulse(MOTOR_PULSES);
  rightMotor.setPulse(MOTOR_PULSES);

  leftMotor.setRatio(GEAR_RATIO);
  rightMotor.setRatio(GEAR_RATIO);

  attachInterrupt(
    leftMotor.getIntNum(),
    leftMotorInterrupt,
    RISING
  );

  attachInterrupt(
    rightMotor.getIntNum(),
    rightMotorInterrupt,
    RISING
  );

  gyro.begin();
  stopMotors();
}

void loop() {

  currentTime = millis();

  gyro.update();

  stateManager(currentTime);
}
