#include <AccelStepper.h>
#include <MultiStepper.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

// Step/dir pins (CNC shield)
AccelStepper stepperX(AccelStepper::DRIVER, 2, 5);
AccelStepper stepperY(AccelStepper::DRIVER, 3, 6);
AccelStepper stepperU(AccelStepper::DRIVER, 4, 7);
AccelStepper stepperV(AccelStepper::DRIVER, 12, 13);
MultiStepper steppers;

// IO
#define ENABLE_PIN 8
#define X_LIMIT_PIN 9
#define Y_LIMIT_PIN 10
#define U_LIMIT_PIN 11
#define V_LIMIT_PIN A3

struct MachineConfig {
  long cutMaxSpeed = 2500;       // steps/sec
  long homeSpeed = 3200;         // steps/sec
  int homeDirection = -1;        // -1 or +1
  uint8_t homeSensorX = 0;       // 0=X, 1=Y, 2=U, 3=V
  uint8_t homeSensorY = 1;
  uint8_t homeSensorU = 2;
  uint8_t homeSensorV = 3;
};

struct MoveTarget {
  long x;
  long y;
  long u;
  long v;
  int speed;
};

MachineConfig config;
long positions[4];
const uint8_t MOVE_QUEUE_CAPACITY = 45;
const uint8_t MAX_INPUT_LINE_LENGTH = 64;
const unsigned long HOME_SWITCH_DEBOUNCE_MS = 10;
const unsigned long HOME_TIMEOUT_MS = 120000;
const unsigned long HOME_STATUS_INTERVAL_MS = 100;
MoveTarget moveQueue[MOVE_QUEUE_CAPACITY];
uint8_t moveQueueHead = 0;
uint8_t moveQueueTail = 0;
uint8_t moveQueueCount = 0;
bool moveActive = false;
bool motionPaused = false;
bool uploadJobActive = false;
long uploadExpectedMoves = 0;
long uploadAcceptedMoves = 0;
long uploadCompletedMoves = 0;
MoveTarget activeMove;

void enableDrivers() {
  digitalWrite(ENABLE_PIN, LOW);
}

void releaseDrivers() {
  digitalWrite(ENABLE_PIN, HIGH);
}

void trimInPlace(char *value) {
  if (!value) return;

  size_t length = strlen(value);
  size_t start = 0;
  while (start < length && isspace((unsigned char)value[start])) {
    start++;
  }

  size_t end = length;
  while (end > start && isspace((unsigned char)value[end - 1])) {
    end--;
  }

  if (start > 0) {
    memmove(value, value + start, end - start);
  }
  value[end - start] = '\0';
}

void toUpperInPlace(char *value) {
  if (!value) return;
  while (*value) {
    *value = (char)toupper((unsigned char)*value);
    value++;
  }
}

bool parseLongValue(const char *value, long &outValue) {
  if (!value) return false;
  while (*value && isspace((unsigned char)*value)) {
    value++;
  }
  if (*value == '\0') return false;

  char *endPtr = nullptr;
  long parsed = strtol(value, &endPtr, 10);
  if (endPtr == value) return false;
  while (*endPtr) {
    if (!isspace((unsigned char)*endPtr)) {
      return false;
    }
    endPtr++;
  }
  outValue = parsed;
  return true;
}

bool limitTriggered(int pin) {
  return digitalRead(pin) == HIGH;
}

void printStatus() {
  Serial.print(F("OK STATUS X="));
  Serial.print(limitTriggered(X_LIMIT_PIN) ? 1 : 0);
  Serial.print(F(" Y="));
  Serial.print(limitTriggered(Y_LIMIT_PIN) ? 1 : 0);
  Serial.print(F(" U="));
  Serial.print(limitTriggered(U_LIMIT_PIN) ? 1 : 0);
  Serial.print(F(" V="));
  Serial.println(limitTriggered(V_LIMIT_PIN) ? 1 : 0);
}

void applyCutSpeed() {
  long speed = config.cutMaxSpeed;
  if (speed < 100) speed = 100;
  stepperX.setMaxSpeed(speed);
  stepperY.setMaxSpeed(speed);
  stepperU.setMaxSpeed(speed);
  stepperV.setMaxSpeed(speed);
}

void printConfig() {
  Serial.print(F("CFG cutMaxSpeed=")); Serial.println(config.cutMaxSpeed);
  Serial.print(F("CFG homeSpeed=")); Serial.println(config.homeSpeed);
  Serial.print(F("CFG homeDirection=")); Serial.println(config.homeDirection);
  Serial.print(F("CFG homeSensorX=")); Serial.println(config.homeSensorX);
  Serial.print(F("CFG homeSensorY=")); Serial.println(config.homeSensorY);
  Serial.print(F("CFG homeSensorU=")); Serial.println(config.homeSensorU);
  Serial.print(F("CFG homeSensorV=")); Serial.println(config.homeSensorV);
  Serial.println(F("OK GETCFG"));
}

void setConfigValue(char *key, char *value) {
  trimInPlace(key);
  trimInPlace(value);
  toUpperInPlace(key);
  long parsed = 0;
  if (!parseLongValue(value, parsed)) {
    Serial.println(F("ERR CFG invalid-number"));
    return;
  }

  if (strcmp(key, "CUTMAXSPEED") == 0) {
    if (parsed < 100) parsed = 100;
    config.cutMaxSpeed = parsed;
    applyCutSpeed();
    Serial.print(F("OK CFG cutMaxSpeed=")); Serial.println(config.cutMaxSpeed);
    return;
  }
  if (strcmp(key, "HOMESPEED") == 0) {
    if (parsed < 100) parsed = 100;
    config.homeSpeed = parsed;
    Serial.print(F("OK CFG homeSpeed=")); Serial.println(config.homeSpeed);
    return;
  }
  if (strcmp(key, "HOMEDIRECTION") == 0) {
    config.homeDirection = parsed >= 0 ? 1 : -1;
    Serial.print(F("OK CFG homeDirection=")); Serial.println(config.homeDirection);
    return;
  }
  if (strcmp(key, "HOMESENSORX") == 0) {
    if (parsed < 0) parsed = 0;
    if (parsed > 3) parsed = 3;
    config.homeSensorX = parsed;
    Serial.print(F("OK CFG homeSensorX=")); Serial.println(config.homeSensorX);
    return;
  }
  if (strcmp(key, "HOMESENSORY") == 0) {
    if (parsed < 0) parsed = 0;
    if (parsed > 3) parsed = 3;
    config.homeSensorY = parsed;
    Serial.print(F("OK CFG homeSensorY=")); Serial.println(config.homeSensorY);
    return;
  }
  if (strcmp(key, "HOMESENSORU") == 0) {
    if (parsed < 0) parsed = 0;
    if (parsed > 3) parsed = 3;
    config.homeSensorU = parsed;
    Serial.print(F("OK CFG homeSensorU=")); Serial.println(config.homeSensorU);
    return;
  }
  if (strcmp(key, "HOMESENSORV") == 0) {
    if (parsed < 0) parsed = 0;
    if (parsed > 3) parsed = 3;
    config.homeSensorV = parsed;
    Serial.print(F("OK CFG homeSensorV=")); Serial.println(config.homeSensorV);
    return;
  }

  Serial.println(F("ERR CFG unknown-key"));
}

int limitPinForSensorIndex(int sensorIndex) {
  switch (sensorIndex) {
    case 0: return X_LIMIT_PIN;
    case 1: return Y_LIMIT_PIN;
    case 2: return U_LIMIT_PIN;
    case 3: return V_LIMIT_PIN;
    default: return X_LIMIT_PIN;
  }
}

bool parseMove(char *parts[], int offset, long coords[4]) {
  for (int i = 0; i < 4; i++) {
    if (!parseLongValue(parts[offset + i], coords[i])) {
      return false;
    }
  }
  return true;
}

int parseAxisIndex(char *value) {
  if (!value) {
    return -1;
  }

  trimInPlace(value);
  toUpperInPlace(value);
  if (strcmp(value, "X") == 0) return 0;
  if (strcmp(value, "Y") == 0) return 1;
  if (strcmp(value, "U") == 0) return 2;
  if (strcmp(value, "V") == 0) return 3;
  return -1;
}

long currentAxisPosition(int axisIndex) {
  switch (axisIndex) {
    case 0: return positions[0];
    case 1: return positions[1];
    case 2: return positions[2];
    case 3: return positions[3];
    default: return 0;
  }
}

void clearMoveQueue() {
  moveQueueHead = 0;
  moveQueueTail = 0;
  moveQueueCount = 0;
  moveActive = false;
  motionPaused = false;
  uploadJobActive = false;
  uploadExpectedMoves = 0;
  uploadAcceptedMoves = 0;
  uploadCompletedMoves = 0;
}

void syncStepperPositionsFromLogical() {
  stepperX.setCurrentPosition(positions[0]);
  stepperY.setCurrentPosition(positions[1]);
  stepperU.setCurrentPosition(positions[2]);
  stepperV.setCurrentPosition(positions[3]);
}

void syncLogicalPositionsFromSteppers() {
  positions[0] = stepperX.currentPosition();
  positions[1] = stepperY.currentPosition();
  positions[2] = stepperU.currentPosition();
  positions[3] = stepperV.currentPosition();
}

bool enqueueMove(long coords[4], long targetSpeed) {
  if (moveQueueCount >= MOVE_QUEUE_CAPACITY) {
    return false;
  }

  moveQueue[moveQueueTail].x = coords[0];
  moveQueue[moveQueueTail].y = coords[1];
  moveQueue[moveQueueTail].u = coords[2];
  moveQueue[moveQueueTail].v = coords[3];
  moveQueue[moveQueueTail].speed = targetSpeed; // <--- NEW
  
  moveQueueTail = (moveQueueTail + 1) % MOVE_QUEUE_CAPACITY;
  moveQueueCount++;
  return true;
}

bool dequeueMove(MoveTarget &target) {
  if (moveQueueCount == 0) {
    return false;
  }

  target = moveQueue[moveQueueHead];
  moveQueueHead = (moveQueueHead + 1) % MOVE_QUEUE_CAPACITY;
  moveQueueCount--;
  return true;
}

bool targetMatchesLogicalPosition(const MoveTarget &target) {
  return
    target.x == positions[0] &&
    target.y == positions[1] &&
    target.u == positions[2] &&
    target.v == positions[3];
}

void startNextQueuedMove() {
  if (motionPaused || moveActive || moveQueueCount == 0) {
    return;
  }

  while (moveQueueCount > 0 && !motionPaused && !moveActive) {
    if (!dequeueMove(activeMove)) {
      return;
    }

    if (targetMatchesLogicalPosition(activeMove)) {
      completeActiveMove();
      continue;
    }

    enableDrivers();
    positions[0] = activeMove.x;
    positions[1] = activeMove.y;
    positions[2] = activeMove.u;
    positions[3] = activeMove.v;
    
    // --- NEW: Apply the dynamic speed for this specific segment ---
    long stepSpeed = activeMove.speed;
    if (stepSpeed < 100) stepSpeed = 100;
    stepperX.setMaxSpeed(stepSpeed);
    stepperY.setMaxSpeed(stepSpeed);
    stepperU.setMaxSpeed(stepSpeed);
    stepperV.setMaxSpeed(stepSpeed);
    // --------------------------------------------------------------

    steppers.moveTo(positions);
    moveActive = true;
    return;
  }
}
void completeActiveMove() {
  moveActive = false;
  uploadCompletedMoves++;

  Serial.print(F("OK MOVE "));
  Serial.print(activeMove.x); Serial.print(",");
  Serial.print(activeMove.y); Serial.print(",");
  Serial.print(activeMove.u); Serial.print(",");
  Serial.println(activeMove.v);

  if (uploadJobActive &&
      uploadCompletedMoves >= uploadExpectedMoves &&
      moveQueueCount == 0) {
    uploadJobActive = false;
    Serial.print(F("OK UPDONE "));
    Serial.println(uploadCompletedMoves);
  }
}

void serviceMoveQueue() {
  if (motionPaused) {
    return;
  }

  if (!moveActive) {
    startNextQueuedMove();
  }

  if (!moveActive) {
    return;
  }

  if (!steppers.run()) {
    completeActiveMove();
    startNextQueuedMove();
  }
}

void pauseMotion() {
  motionPaused = true;
  Serial.println(F("OK PAUSE"));
}

void resumeMotion() {
  motionPaused = false;
  startNextQueuedMove();
  Serial.println(F("OK RESUME"));
}

void abortMotion() {
  syncLogicalPositionsFromSteppers();
  steppers.moveTo(positions);
  syncStepperPositionsFromLogical();
  clearMoveQueue();

  Serial.print(F("OK ABORT "));
  Serial.print(positions[0]); Serial.print(",");
  Serial.print(positions[1]); Serial.print(",");
  Serial.print(positions[2]); Serial.print(",");
  Serial.println(positions[3]);
}

void executeMove(long coords[4]) {
  if (coords[0] == positions[0] &&
      coords[1] == positions[1] &&
      coords[2] == positions[2] &&
      coords[3] == positions[3]) {
    Serial.print(F("OK MOVE "));
    Serial.print(coords[0]); Serial.print(",");
    Serial.print(coords[1]); Serial.print(",");
    Serial.print(coords[2]); Serial.print(",");
    Serial.println(coords[3]);
    return;
  }

  enableDrivers();
  positions[0] = coords[0];
  positions[1] = coords[1];
  positions[2] = coords[2];
  positions[3] = coords[3];

  steppers.moveTo(positions);
  steppers.runSpeedToPosition();

  Serial.print(F("OK MOVE "));
  Serial.print(positions[0]); Serial.print(",");
  Serial.print(positions[1]); Serial.print(",");
  Serial.print(positions[2]); Serial.print(",");
  Serial.println(positions[3]);
}

void jogAxis(int axisIndex, long delta) {
  if (axisIndex < 0 || axisIndex > 3) {
    Serial.println(F("ERR JOG invalid-axis"));
    return;
  }
  if (delta == 0) {
    Serial.println(F("ERR JOG zero-delta"));
    return;
  }

  enableDrivers();

  AccelStepper *stepper = nullptr;
  switch (axisIndex) {
    case 0: stepper = &stepperX; break;
    case 1: stepper = &stepperY; break;
    case 2: stepper = &stepperU; break;
    case 3: stepper = &stepperV; break;
    default: break;
  }

  if (!stepper) {
    Serial.println(F("ERR JOG invalid-axis"));
    return;
  }

  syncStepperPositionsFromLogical();
  const long target = positions[axisIndex] + delta;
  long speed = config.cutMaxSpeed;
  if (speed < 100) speed = 100;
  stepper->setCurrentPosition(positions[axisIndex]);
  stepper->setMaxSpeed(speed);
  stepper->moveTo(target);
  stepper->setSpeed(delta > 0 ? speed : -speed);

  while (stepper->distanceToGo() != 0) {
    stepper->runSpeedToPosition();
  }

  positions[axisIndex] = target;

  Serial.print(F("OK JOG "));
  Serial.print(positions[0]); Serial.print(",");
  Serial.print(positions[1]); Serial.print(",");
  Serial.print(positions[2]); Serial.print(",");
  Serial.println(positions[3]);
}

void syncPosition(long coords[4]) {
  positions[0] = coords[0];
  positions[1] = coords[1];
  positions[2] = coords[2];
  positions[3] = coords[3];

  stepperX.setCurrentPosition(positions[0]);
  stepperY.setCurrentPosition(positions[1]);
  stepperU.setCurrentPosition(positions[2]);
  stepperV.setCurrentPosition(positions[3]);
  clearMoveQueue();

  Serial.print(F("OK SETPOS "));
  Serial.print(positions[0]); Serial.print(",");
  Serial.print(positions[1]); Serial.print(",");
  Serial.print(positions[2]); Serial.print(",");
  Serial.println(positions[3]);
}

const uint8_t HOME_AXIS_SEEKING = 0;
const uint8_t HOME_AXIS_DEBOUNCING = 1;
const uint8_t HOME_AXIS_DONE = 2;

bool isHomeAxisDone(uint8_t state) {
  return state == HOME_AXIS_DONE;
}

void updateHomeAxis(AccelStepper &stepper,
                    int limitPin,
                    uint8_t &state,
                    unsigned long &debounceStartedAtMs) {
  if (state == HOME_AXIS_DONE) {
    return;
  }

  const bool limitActive = limitTriggered(limitPin);

  if (state == HOME_AXIS_DEBOUNCING) {
    if (!limitActive) {
      state = HOME_AXIS_SEEKING;
      debounceStartedAtMs = 0;
      stepper.setSpeed(config.homeDirection * config.homeSpeed);
      return;
    }

    if (millis() - debounceStartedAtMs >= HOME_SWITCH_DEBOUNCE_MS) {
      stepper.setSpeed(0);
      state = HOME_AXIS_DONE;
    }
    return;
  }

  if (limitActive) {
    stepper.setSpeed(0);
    if (HOME_SWITCH_DEBOUNCE_MS == 0) {
      state = HOME_AXIS_DONE;
    } else {
      state = HOME_AXIS_DEBOUNCING;
      debounceStartedAtMs = millis();
    }
    return;
  }

  stepper.runSpeed();
}

void homeMachine() {
  enableDrivers();
  const unsigned long homeStartedAtMs = millis();
  unsigned long lastHomeStatusAtMs = 0;

  const int xHomeLimitPin = limitPinForSensorIndex(config.homeSensorX);
  const int yHomeLimitPin = limitPinForSensorIndex(config.homeSensorY);
  const int uHomeLimitPin = limitPinForSensorIndex(config.homeSensorU);
  const int vHomeLimitPin = limitPinForSensorIndex(config.homeSensorV);

  stepperX.setMaxSpeed(config.homeSpeed);
  stepperY.setMaxSpeed(config.homeSpeed);
  stepperU.setMaxSpeed(config.homeSpeed);
  stepperV.setMaxSpeed(config.homeSpeed);

  // --- STAGE 1: Home X and U (Horizontal) ---
  stepperX.setSpeed(config.homeDirection * config.homeSpeed);
  stepperU.setSpeed(config.homeDirection * config.homeSpeed);
  
  uint8_t xState = limitTriggered(xHomeLimitPin) ? HOME_AXIS_DEBOUNCING : HOME_AXIS_SEEKING;
  uint8_t uState = limitTriggered(uHomeLimitPin) ? HOME_AXIS_DEBOUNCING : HOME_AXIS_SEEKING;
  
  unsigned long xDebounceStartedAtMs = xState == HOME_AXIS_DEBOUNCING ? homeStartedAtMs : 0;
  unsigned long uDebounceStartedAtMs = uState == HOME_AXIS_DEBOUNCING ? homeStartedAtMs : 0;

  while (!isHomeAxisDone(xState) || !isHomeAxisDone(uState)) {
    const unsigned long nowMs = millis();
    if (lastHomeStatusAtMs == 0 || nowMs - lastHomeStatusAtMs >= HOME_STATUS_INTERVAL_MS) {
      printStatus();
      lastHomeStatusAtMs = nowMs;
    }
    if (nowMs - homeStartedAtMs >= HOME_TIMEOUT_MS) {
      applyCutSpeed(); clearMoveQueue(); Serial.println(F("ERR HOME timeout")); return;
    }
    updateHomeAxis(stepperX, xHomeLimitPin, xState, xDebounceStartedAtMs);
    updateHomeAxis(stepperU, uHomeLimitPin, uState, uDebounceStartedAtMs);
  }

  // --- STAGE 2: Home Y and V (Vertical) ---
  stepperY.setSpeed(config.homeDirection * config.homeSpeed);
  stepperV.setSpeed(config.homeDirection * config.homeSpeed);
  
  uint8_t yState = limitTriggered(yHomeLimitPin) ? HOME_AXIS_DEBOUNCING : HOME_AXIS_SEEKING;
  uint8_t vState = limitTriggered(vHomeLimitPin) ? HOME_AXIS_DEBOUNCING : HOME_AXIS_SEEKING;
  
  unsigned long yDebounceStartedAtMs = yState == HOME_AXIS_DEBOUNCING ? millis() : 0;
  unsigned long vDebounceStartedAtMs = vState == HOME_AXIS_DEBOUNCING ? millis() : 0;

  while (!isHomeAxisDone(yState) || !isHomeAxisDone(vState)) {
    const unsigned long nowMs = millis();
    if (lastHomeStatusAtMs == 0 || nowMs - lastHomeStatusAtMs >= HOME_STATUS_INTERVAL_MS) {
      printStatus();
      lastHomeStatusAtMs = nowMs;
    }
    if (nowMs - homeStartedAtMs >= HOME_TIMEOUT_MS) {
      applyCutSpeed(); clearMoveQueue(); Serial.println(F("ERR HOME timeout")); return;
    }
    updateHomeAxis(stepperY, yHomeLimitPin, yState, yDebounceStartedAtMs);
    updateHomeAxis(stepperV, vHomeLimitPin, vState, vDebounceStartedAtMs);
  }

  // --- STAGE 3: Finalize ---
  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperU.setCurrentPosition(0);
  stepperV.setCurrentPosition(0);
  positions[0] = 0; positions[1] = 0; positions[2] = 0; positions[3] = 0;
  
  applyCutSpeed();
  clearMoveQueue();
  printStatus();
  Serial.println(F("OK HOME"));
}

void dispatchLine(char *line) {
  trimInPlace(line);
  if (line[0] == '\0') {
    return;
  }

  char *parts[8];
  int partCount = 0;
  char *context = nullptr;
  char *token = strtok_r(line, ",", &context);
  while (token && partCount < 8) {
    trimInPlace(token);
    parts[partCount++] = token;
    token = strtok_r(nullptr, ",", &context);
  }
  if (partCount <= 0) return;

  char *command = parts[0];
  toUpperInPlace(command);
  long coords[4];

  // Backward-compatible raw move: X,Y,U,V
  if (partCount == 4 && parseMove(parts, 0, coords)) {
    executeMove(coords);
    return;
  }

  if (strcmp(command, "UPLOAD") == 0) {
    if (partCount < 2) {
      Serial.println(F("ERR UPLOAD invalid-format"));
      return;
    }

    long expected = 0;
    if (!parseLongValue(parts[1], expected) || expected <= 0) {
      Serial.println(F("ERR UPLOAD invalid-count"));
      return;
    }

    clearMoveQueue();
    uploadJobActive = true;
    uploadExpectedMoves = expected;
    Serial.print(F("OK UPLOAD "));
    Serial.print(uploadExpectedMoves);
    Serial.print(F(" CAP="));
    Serial.println(MOVE_QUEUE_CAPACITY);
    return;
  }

  if (strcmp(command, "QUEUE") == 0) {
    if (partCount < 5 || !parseMove(parts, 1, coords)) {
      Serial.println(F("ERR QUEUE invalid-format"));
      return;
    }
    
    // Default to the UI's base speed, but overwrite if Python sends a dynamic speed
    long targetSpeed = config.cutMaxSpeed; 
    if (partCount >= 6) {
        parseLongValue(parts[5], targetSpeed);
    }
    
    if (!uploadJobActive) {
      Serial.println(F("ERR QUEUE no-upload"));
      return;
    }
    if (uploadAcceptedMoves >= uploadExpectedMoves) {
      Serial.println(F("ERR QUEUE overflow"));
      return;
    }
    if (!enqueueMove(coords, targetSpeed)) {
      Serial.println(F("ERR QUEUE full"));
      return;
    }

    uploadAcceptedMoves++;
    Serial.print(F("OK QUEUE "));
    Serial.print(uploadAcceptedMoves);
    Serial.print("/");
    Serial.print(uploadExpectedMoves);
    Serial.print(F(" BUF="));
    Serial.println(moveQueueCount + (moveActive ? 1 : 0));
    startNextQueuedMove();
    return;
  }

  if (strcmp(command, "SETPOS") == 0) {
    if (partCount < 5 || !parseMove(parts, 1, coords)) {
      Serial.println(F("ERR SETPOS invalid-format"));
      return;
    }
    syncPosition(coords);
    return;
  }

  if (strcmp(command, "MOVE") == 0) {
    if (partCount < 5 || !parseMove(parts, 1, coords)) {
      Serial.println(F("ERR MOVE invalid-format"));
      return;
    }
    executeMove(coords);
    return;
  }

  if (strcmp(command, "JOG") == 0) {
    if (partCount < 3) {
      Serial.println(F("ERR JOG invalid-format"));
      return;
    }
    const int axisIndex = parseAxisIndex(parts[1]);
    long delta = 0;
    if (axisIndex < 0 || !parseLongValue(parts[2], delta)) {
      Serial.println(F("ERR JOG invalid-format"));
      return;
    }
    jogAxis(axisIndex, delta);
    return;
  }

  if (strcmp(command, "HOME") == 0) {
    homeMachine();
    return;
  }

  if (strcmp(command, "PAUSE") == 0) {
    pauseMotion();
    return;
  }

  if (strcmp(command, "RESUME") == 0) {
    resumeMotion();
    return;
  }

  if (strcmp(command, "ABORT") == 0) {
    abortMotion();
    return;
  }

  if (strcmp(command, "CFG") == 0) {
    if (partCount < 3) {
      Serial.println(F("ERR CFG invalid-format"));
      return;
    }
    setConfigValue(parts[1], parts[2]);
    return;
  }

  if (strcmp(command, "GETCFG") == 0) {
    printConfig();
    return;
  }

  if (strcmp(command, "STATUS") == 0) {
    printStatus();
    return;
  }

  if (strcmp(command, "RELEASE") == 0) {
    releaseDrivers();
    Serial.println(F("OK RELEASE"));
    return;
  }

  if (strcmp(command, "PING") == 0) {
    Serial.println(F("OK PONG"));
    return;
  }

  Serial.println(F("ERR unknown-command"));
}

void setup() {
  Serial.begin(115200);

  pinMode(ENABLE_PIN, OUTPUT);
  enableDrivers();

  pinMode(X_LIMIT_PIN, INPUT_PULLUP);
  pinMode(Y_LIMIT_PIN, INPUT_PULLUP);
  pinMode(U_LIMIT_PIN, INPUT_PULLUP);
  pinMode(V_LIMIT_PIN, INPUT_PULLUP);

  steppers.addStepper(stepperX);
  steppers.addStepper(stepperY);
  steppers.addStepper(stepperU);
  steppers.addStepper(stepperV);
  positions[0] = 0;
  positions[1] = 0;
  positions[2] = 0;
  positions[3] = 0;
  syncStepperPositionsFromLogical();

  applyCutSpeed();

  Serial.println(F("OK READY"));
  Serial.println(F("INFO Commands: MOVE, JOG, UPLOAD, QUEUE, SETPOS, HOME, PAUSE, RESUME, ABORT, CFG, GETCFG, STATUS, RELEASE, PING"));
}

void loop() {
  static char inputLine[MAX_INPUT_LINE_LENGTH];
  static uint8_t inputLength = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      inputLine[inputLength] = '\0';
      trimInPlace(inputLine);
      if (inputLine[0] != '\0') {
        dispatchLine(inputLine);
      }
      inputLength = 0;
      inputLine[0] = '\0';
    } else {
      if (inputLength < MAX_INPUT_LINE_LENGTH - 1) {
        inputLine[inputLength++] = c;
        inputLine[inputLength] = '\0';
      } else {
        inputLength = 0;
        inputLine[0] = '\0';
        Serial.println(F("ERR LINE overflow"));
      }
    }
  }

  serviceMoveQueue();
}
