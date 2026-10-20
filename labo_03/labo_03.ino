
#include <MeAuriga.h>

// Labo 03 - DA: 2384980 

MeEncoderOnBoard leftMotor(SLOT1);
MeEncoderOnBoard rightMotor(SLOT2);
MeGyro gyro(0, 0x69);
MeUltrasonicSensor ultrasonicSensor(PORT_6);
MeRGBLed led(0, 12);

const unsigned long START_DELAY = 3000;
const unsigned long SENSOR_RATE = 50;

const float TARGET_DISTANCE = 100.0;
const float STOP_DISTANCE_CM = 30.0;
const float WHEEL_DIAMETER = 6.5;
const float GEAR_RATIO = 39.267;
const float GYRO_CORRECTION = 3.0;

const int MOTOR_PULSES = 9;
const int MOTOR_SPEED = 100;
const int TURN_SPEED = 80;

enum AppState {
  WAIT_FOR_START,
  MOVE_FORWARD,
  TURN_LEFT,
  MOVE_TO_OBSTACLE,
  STOP
};

AppState appState = WAIT_FOR_START;
unsigned long currentTime = 0;

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

long calculateTicks(float distanceCm) {
  float ticksPerTurn = MOTOR_PULSES * GEAR_RATIO;
  float circumference = WHEEL_DIAMETER * PI;
  return round((distanceCm / circumference) * ticksPerTurn);
}

void stopMotors() {
  leftMotor.setMotorPwm(0);
  rightMotor.setMotorPwm(0);
}

void moveStraight() {
  float correction = gyro.getAngleZ() * GYRO_CORRECTION;

  int leftSpeed = constrain(MOTOR_SPEED - correction, 0, 255);
  int rightSpeed = constrain(MOTOR_SPEED + correction, 0, 255);

  leftMotor.setMotorPwm(-leftSpeed);
  rightMotor.setMotorPwm(rightSpeed);
}

void clearLeds() {
  led.setColor(0, 0, 0, 0);
  led.show();
}

void showProgress(long currentTicks, long targetTicks) {
  int numberOfLeds = 0;

  if (targetTicks > 0) {
    numberOfLeds = (currentTicks * 7L) / targetTicks;
  }

  numberOfLeds = constrain(numberOfLeds, 0, 7);

  for (int ledNumber = 1; ledNumber <= 7; ledNumber++) {
    if (ledNumber <= numberOfLeds) {
      led.setColor(ledNumber, 0, 255, 0);
    } else {
      led.setColor(ledNumber, 0, 0, 0);
    }
  }

  led.show();
}

void showFirstSegmentLeds() {
  clearLeds();
  led.setColor(12, 0, 255, 0);
  led.show();
}

void showSecondSegmentLeds() {
  clearLeds();
  led.setColor(11, 0, 255, 0);
  led.show();
}

void waitForStartState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long lastTime = 0;

  if (firstTime) {
    stopMotors();
    clearLeds();
    lastTime = ct;
    firstTime = false;
    return;
  }

  if (ct - lastTime >= START_DELAY) {
    firstTime = true;
    appState = MOVE_FORWARD;
  }
}

void moveForwardState(unsigned long ct) {
  static bool firstTime = true;
  static long startLeftTicks = 0;
  static long startRightTicks = 0;

  if (firstTime) {
    stopMotors();

    startLeftTicks = leftMotor.getPulsePos();
    startRightTicks = rightMotor.getPulsePos();

    gyro.resetData();
    showFirstSegmentLeds();

    Serial.println("First segment");

    firstTime = false;
    return;
  }

  long leftTicks = abs(leftMotor.getPulsePos() - startLeftTicks);
  long rightTicks = abs(rightMotor.getPulsePos() - startRightTicks);
  long averageTicks = (leftTicks + rightTicks) / 2;
  long targetTicks = calculateTicks(TARGET_DISTANCE);

  showProgress(averageTicks, targetTicks);

  if (averageTicks >= targetTicks) {
    stopMotors();
    showProgress(targetTicks, targetTicks);

    firstTime = true;
    appState = TURN_LEFT;
    return;
  }

  moveStraight();
}

void turnLeftState(unsigned long ct) {
  static bool firstTime = true;

  if (firstTime) {
    stopMotors();
    gyro.resetData();

    Serial.println("Turning left");

    firstTime = false;
    return;
  }

  float currentAngle = gyro.getAngleZ();

  if (abs(currentAngle) >= 90.0) {
    stopMotors();

    firstTime = true;
    appState = MOVE_TO_OBSTACLE;
    return;
  }

  leftMotor.setMotorPwm(TURN_SPEED);
  rightMotor.setMotorPwm(TURN_SPEED);
}

void moveToObstacleState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long lastTime = 0;

  static long startLeftTicks = 0;
  static long startRightTicks = 0;
  static long targetTicks = 0;

  if (firstTime) {
    stopMotors();
    showSecondSegmentLeds();
    gyro.resetData();

    startLeftTicks = leftMotor.getPulsePos();
    startRightTicks = rightMotor.getPulsePos();

    float startDistance = ultrasonicSensor.distanceCm();

    if (startDistance <= 0 || startDistance >= 400) {
      Serial.println("Invalid distance");
      appState = STOP;
      return;
    }

    float travelDistance = startDistance - STOP_DISTANCE_CM;

    if (travelDistance < 0) {
      travelDistance = 0;
    }

    targetTicks = calculateTicks(travelDistance);

    lastTime = ct;
    firstTime = false;
    return;
  }

  if (ct - lastTime < SENSOR_RATE) {
    return;
  }

  lastTime = ct;

  float obstacleDistance = ultrasonicSensor.distanceCm();

  if (obstacleDistance <= 0 || obstacleDistance >= 400) {
    stopMotors();
    return;
  }

  long leftTicks = abs(leftMotor.getPulsePos() - startLeftTicks);
  long rightTicks = abs(rightMotor.getPulsePos() - startRightTicks);
  long averageTicks = (leftTicks + rightTicks) / 2;

  showProgress(averageTicks, targetTicks);

  if (obstacleDistance <= STOP_DISTANCE_CM) {
    stopMotors();
    showProgress(targetTicks, targetTicks);

    Serial.println("Obstacle reached");

    firstTime = true;
    appState = STOP;
    return;
  }

  moveStraight();
}

void stopState(unsigned long ct) {
  stopMotors();
}

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

    case MOVE_TO_OBSTACLE:
      moveToObstacleState(ct);
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

  attachInterrupt(leftMotor.getIntNum(), leftMotorInterrupt, RISING);
  attachInterrupt(rightMotor.getIntNum(), rightMotorInterrupt, RISING);

  gyro.begin();
  led.setpin(44);

  stopMotors();
  clearLeds();
}

void loop() {
  currentTime = millis();
  gyro.update();
  stateManager(currentTime);
}
