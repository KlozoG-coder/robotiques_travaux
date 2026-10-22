#include <MeAuriga.h>

// DA : 2384980

MeEncoderOnBoard leftMotor(SLOT1);
MeEncoderOnBoard rightMotor(SLOT2);
MeGyro gyro(0, 0x69);
MeUltrasonicSensor ultrasonicSensor(PORT_10);
MeRGBLed led(0, 12);

// Distances
const float TARGET_DISTANCE = 90.0;
const float STOP_DISTANCE = 30.0;
const float SLOW_DISTANCE = 50.0;

// Encodeurs
const float WHEEL_DIAMETER = 6.5;
const float GEAR_RATIO = 39.267;
const int MOTOR_PULSES = 9;

// Vitesses
const int MOTOR_SPEED = 100;
const int SLOW_SPEED = 65;
const int TURN_SPEED = 70;
const int TURN_SLOW_SPEED = 50;

// Correction de trajectoire
const float GYRO_CORRECTION = 1.5;
const float ENCODER_CORRECTION = 0.15;
const int MAX_CORRECTION = 35;

// Temps
const unsigned long START_DELAY = 3000;
const unsigned long SENSOR_RATE = 50;
const unsigned long BLINK_RATE = 50;
const unsigned long MAX_TURN_TIME = 6000;

// Machine a etats
enum AppState {
  WAIT_FOR_START,
  MOVE_FIRST_SEGMENT,
  TURN_LEFT,
  MOVE_SECOND_SEGMENT,
  TURN_RIGHT,
  MOVE_THIRD_SEGMENT,
  TURN_180,
  FINISHED,
  RETURN_THIRD_SEGMENT,
  RETURN_TURN_LEFT,
  RETURN_SECOND_SEGMENT,
  RETURN_TURN_RIGHT,
  RETURN_FIRST_SEGMENT,
  RETURN_FINISHED,
  TURN_ERROR
};

AppState appState = WAIT_FOR_START;

unsigned long currentTime = 0;

long startLeft = 0;
long startRight = 0;
long secondTargetTicks = 0;
long secondActualTicks = 0;
unsigned long rainbowLastTime = 0;
int rainbowStep = 0;

float startAngle = 0;

// Interruptions
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

// Convertir une distance en ticks
long calculateTicks(float distanceCm) {
  return round(distanceCm * MOTOR_PULSES * GEAR_RATIO / (WHEEL_DIAMETER * PI));
}

// Mesurer les ticks des roues
long getLeftTicks() {
  return abs(leftMotor.getPulsePos() - startLeft);
}

long getRightTicks() {
  return abs(rightMotor.getPulsePos() - startRight);
}

long getAverageTicks() {
  return (getLeftTicks() + getRightTicks()) / 2;
}

// Arreter les moteurs
void stopMotors() {
  leftMotor.setMotorPwm(0);
  rightMotor.setMotorPwm(0);
}

// Debut d'un segment
void startSegment() {
  stopMotors();

  startLeft = leftMotor.getPulsePos();
  startRight = rightMotor.getPulsePos();

  startAngle = gyro.getAngleZ();
}

// Avancer en ligne droite
void moveStraight(int speed) {
  const int RIGHT_MOTOR_TRIM = 3;

  float angle = gyro.getAngleZ() - startAngle;
  float correction = angle * GYRO_CORRECTION +
    (getLeftTicks() - getRightTicks()) * ENCODER_CORRECTION;

  correction = constrain(correction, -MAX_CORRECTION, MAX_CORRECTION);

  int leftSpeed = constrain(speed + correction, 0, 255);
  int rightSpeed = constrain(speed - correction + RIGHT_MOTOR_TRIM, 0, 255);

  leftMotor.setMotorPwm(-leftSpeed);
  rightMotor.setMotorPwm(rightSpeed);
}

void clearLeds() {
  led.setColor(0, 0, 0, 0);
  led.show();
}

void showSegmentLeds(int activeLed) {
  for (int i = 1; i <= 7; i++) {
    led.setColor(i, 0, 0, 0);
  }

  led.setColor(activeLed, 0, 255, 0);
  led.show();
}

void showProgress(long currentTicks, long targetTicks) {
  int numberOfLeds = 0;

  if (targetTicks > 0) {
    numberOfLeds = currentTicks * 7L / targetTicks;
  }

  numberOfLeds = constrain(numberOfLeds, 0, 7);

  for (int i = 1; i <= 7; i++) {
    if (i <= numberOfLeds) {
      led.setColor(i, 0, 255, 0);
    } else {
      led.setColor(i, 0, 0, 0);
    }
  }

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
    appState = MOVE_FIRST_SEGMENT;
  }
}

// Segment 1
void moveFirstSegmentState(unsigned long ct) {
  static bool firstTime = true;

  if (firstTime) {
    startSegment();
    showSegmentLeds(12);

    Serial.println("Segment 1");

    firstTime = false;
    return;
  }

  long currentTicks = getAverageTicks();
  long targetTicks = calculateTicks(TARGET_DISTANCE);

  showProgress(currentTicks, targetTicks);

  if (currentTicks >= targetTicks) {
    stopMotors();

    firstTime = true;
    appState = TURN_LEFT;
    return;
  }

  moveStraight(MOTOR_SPEED);
}

// Fonction commune aux pivots
void turnState(int direction, float targetAngle, AppState nextState, unsigned long ct) {
  static bool firstTime = true;
  static unsigned long turnStartTime = 0;
  static float turnStartAngle = 0;

  if (firstTime) {
    stopMotors();

    turnStartAngle = gyro.getAngleZ();
    turnStartTime = ct;

    Serial.print("Pivot de ");
    Serial.println(targetAngle);

    firstTime = false;
    return;
  }

  float currentAngle = gyro.getAngleZ() - turnStartAngle;

  float angle = abs(currentAngle);

  if (angle >= targetAngle) {
    stopMotors();

    Serial.println("Pivot termine");

    firstTime = true;
    appState = nextState;
    return;
  }

  if (ct - turnStartTime >= MAX_TURN_TIME) {
    stopMotors();

    Serial.println("ERREUR : pivot trop long");

    firstTime = true;
    appState = TURN_ERROR;
    return;
  }

  int speed = TURN_SPEED;

  if (angle >= targetAngle - 20.0) {
    speed = TURN_SLOW_SPEED;
  }

  leftMotor.setMotorPwm(direction * speed);
  rightMotor.setMotorPwm(direction * speed);
}

// Segment 2
void moveSecondSegmentState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long lastTime = 0;
  static bool distanceReady = false;
  static int currentSpeed = 0;

  if (firstTime) {
    startSegment();
    showSegmentLeds(11);

    distanceReady = false;
    currentSpeed = 0;
    secondTargetTicks = 0;

    lastTime = ct;
    firstTime = false;

    Serial.println("Segment 2");
    return;
  }

  if (ct - lastTime >= SENSOR_RATE) {
    lastTime = ct;

    float distance = ultrasonicSensor.distanceCm();

    Serial.print("Distance : ");
    Serial.println(distance);

    if (distance <= 0 || distance >= 400) {
      currentSpeed = 0;
    } else {
      if (!distanceReady) {
        secondTargetTicks = calculateTicks(distance - STOP_DISTANCE);
        distanceReady = true;
      }

      if (distance <= STOP_DISTANCE) {
        stopMotors();
        secondActualTicks = getAverageTicks();
        showProgress(1, 1);

        firstTime = true;
        appState = TURN_RIGHT;
        return;
      }

      if (distance <= SLOW_DISTANCE) {
        currentSpeed = SLOW_SPEED;
      } else {
        currentSpeed = MOTOR_SPEED;
      }
    }
  }

  showProgress(
    getAverageTicks(),
    secondTargetTicks
  );

  if (currentSpeed == 0) {
    stopMotors();
  } else {
    moveStraight(currentSpeed);
  }
}

// Segment 3
void moveThirdSegmentState(unsigned long ct) {
  static bool firstTime = true;

  if (firstTime) {
    startSegment();
    showSegmentLeds(10);

    Serial.println("Segment 3");

    firstTime = false;
    return;
  }

  long currentTicks = getAverageTicks();
  long targetTicks = calculateTicks(TARGET_DISTANCE);

  showProgress(currentTicks, targetTicks);

  if (currentTicks >= targetTicks) {
    stopMotors();

    firstTime = true;
    appState = TURN_180;
    return;
  }

  moveStraight(MOTOR_SPEED);
}

// Fin
void finishedState(unsigned long ct) {
  static bool firstTime = true;
  static unsigned long lastTime = 0;
  static bool ledsOn = false;

  if (firstTime) {
    stopMotors();
    clearLeds();

    lastTime = ct;
    ledsOn = false;
    firstTime = false;

    Serial.println("Mission terminee");
  }

  if (ct - lastTime >= BLINK_RATE) {
    lastTime = ct;
    ledsOn = !ledsOn;

    for (int i = 8; i <= 12; i++) {
      if (ledsOn) {
        led.setColor(i, 0, 255, 0);
      } else {
        led.setColor(i, 0, 0, 0);
      }
    }

    led.show();
  }
}

// Erreur de pivot
void turnErrorState() {
  stopMotors();
}

// Bonus 

void showRainbow(unsigned long ct) {
  if (ct - rainbowLastTime < 70) return;
  rainbowLastTime = ct;
  rainbowStep = (rainbowStep + 1) % 12;

  const byte colors[6][3] = {
    {255, 0, 0}, {255, 100, 0}, {255, 255, 0},
    {0, 255, 0}, {0, 0, 255}, {150, 0, 255}
  };
  for (int i = 1; i <= 12; i++) {
    int colorIndex = ((i - 1 + rainbowStep) / 2) % 6;
    led.setColor(i, colors[colorIndex][0], colors[colorIndex][1], colors[colorIndex][2]);
  }
  led.show();
}

void returnSegmentState(long targetTicks, AppState nextState, unsigned long ct) {
  static bool firstTime = true;
  if (firstTime) {
    startSegment();
    rainbowLastTime = ct - 70;
    firstTime = false;
    return;
  }

  showRainbow(ct);
  if (getAverageTicks() >= targetTicks) {
    stopMotors();
    firstTime = true;
    appState = nextState;
    return;
  }
  moveStraight(MOTOR_SPEED);
}

void returnFinishedState(unsigned long ct) {
  stopMotors();
  showRainbow(ct);
}

// Gestionnaire FSM
void stateManager(unsigned long ct) {
  switch (appState) {
    case WAIT_FOR_START:
      waitForStartState(ct);
      break;

    case MOVE_FIRST_SEGMENT:
      moveFirstSegmentState(ct);
      break;

    case TURN_LEFT:
      turnState(-1, 90.0, MOVE_SECOND_SEGMENT, ct);
      break;

    case MOVE_SECOND_SEGMENT:
      moveSecondSegmentState(ct);
      break;

    case TURN_RIGHT:
      turnState(1, 90.0, MOVE_THIRD_SEGMENT, ct);
      break;

    case MOVE_THIRD_SEGMENT:
      moveThirdSegmentState(ct);
      break;

    case TURN_180:
      turnState(1, 180.0, RETURN_THIRD_SEGMENT, ct);
      break;

    case FINISHED:
      finishedState(ct);
      break;

    case RETURN_THIRD_SEGMENT:
      returnSegmentState(calculateTicks(TARGET_DISTANCE), RETURN_TURN_LEFT, ct);
      break;

    case RETURN_TURN_LEFT:
      showRainbow(ct);
      turnState(-1, 90.0, RETURN_SECOND_SEGMENT, ct);
      break;

    case RETURN_SECOND_SEGMENT:
      returnSegmentState(secondActualTicks, RETURN_TURN_RIGHT, ct);
      break;

    case RETURN_TURN_RIGHT:
      showRainbow(ct);
      turnState(1, 90.0, RETURN_FIRST_SEGMENT, ct);
      break;

    case RETURN_FIRST_SEGMENT:
      returnSegmentState(calculateTicks(TARGET_DISTANCE), RETURN_FINISHED, ct);
      break;

    case RETURN_FINISHED:
      returnFinishedState(ct);
      break;

    case TURN_ERROR:
      turnErrorState();
      break;
  }
}
// Initialisation
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
