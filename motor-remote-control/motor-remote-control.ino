#include <Arduino_FreeRTOS.h>

#include "semphr.h"

#define LeftMortorPin1 4
#define LeftMortorPin2 5
#define LeftMortorPwmPin 9
#define RightMortorPin1 6
#define RightMortorPin2 7
#define RightMortorPwmPin 10

#define kDistanceNear 60
#define kDistanceFar 80

// attach pins to HC-SR04
#define kEchoPin 3
#define kTrigPin 8

enum RemoteControl { kRemoteControlStop = 0, kRemoteControlGo = 1 };
enum MotorDirection {
  kMotorDirectionBackward = -1,
  kMotorDirectionStop = 0,
  kMotorDirectionForward = 1
};
enum RobotBehavior { kRobotBehaviorGo, kRobotBehaviorDodge };

SemaphoreHandle_t policy_signal;
SemaphoreHandle_t remote_control_signal;
SemaphoreHandle_t pwm_setup_signal;

enum RobotBehavior behavior_policy = kRobotBehaviorGo;
enum RemoteControl remote_control = kRemoteControlGo;
int motor_pwm = 0;
unsigned long echo_initial_time = 0;
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

// Defines two tasks for Blink & AnalogRead
void MotorControl(void *pvParameters);
void CollisionAvoidance(void *pvParameters);
void TaskSerial(void *pvParameters);
void TaskSetPWM(void *pvParameters);

void LeftMotorControl(enum MotorDirection motor_direction, int pwm);
void RightMotorControl(enum MotorDirection motor_direction, int pwm);

// The setup function runs once when you press reset or power the board
void setup() {

  policy_signal = xSemaphoreCreateBinary();
  remote_control_signal = xSemaphoreCreateBinary();
  pwm_setup_signal = xSemaphoreCreateBinary();

  Serial.begin(9600);

  pinMode(kEchoPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kEchoPin), HandleEcho, CHANGE);

  /* // Now set up two tasks to run independently. */
  xTaskCreate(MotorControl,
              "MortorControl", // A name just for humans
              256, // This stack size can be checked & adjusted by reading the
                   // Stack Highwater
              NULL,
              2, // Priority, with 1 being the highest, and 4 being the lowest.
              NULL);

  // Now set up two tasks to run independently.
  xTaskCreate(CollisionAvoidance,
              "CollisionAvoidance", // A name just for humans
              256, // This stack size can be checked & adjusted by reading the
                   // Stack Highwater
              NULL,
              2, // Priority, with 1 being the highest, and 4 being the lowest.
              NULL);

  xTaskCreate(TaskSerial,
              "Serial", // A name just for humans
              128, // This stack size can be checked & adjusted by reading the
                   // Stack Highwater
              NULL,
              1, // Priority, with 1 being the highest, and 4 being the lowest.
              NULL);

  xTaskCreate(TaskSetPWM,
              "SetPWM", // A name just for humans
              128, // This stack size can be checked & adjusted by reading the
                   // Stack Highwater
              NULL,
              3, // Priority, with 1 being the highest, and 4 being the lowest.
              NULL);

  // Now the task scheduler, which takes over control of scheduling individual
  // tasks, is automatically started.
}

void loop() {
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void LeftMotorControl(enum MotorDirection motor_direction, int pwm) {

  analogWrite(LeftMortorPwmPin, pwm);
  if (motor_direction == kMotorDirectionBackward) {
    digitalWrite(LeftMortorPin1, HIGH);
    digitalWrite(LeftMortorPin2, LOW);
  } else if (motor_direction == kMotorDirectionForward) {
    digitalWrite(LeftMortorPin1, LOW);
    digitalWrite(LeftMortorPin2, HIGH);
  } else {
    analogWrite(LeftMortorPwmPin, 0);
  }
}

void RightMotorControl(enum MotorDirection motor_direction, int pwm) {

  analogWrite(RightMortorPwmPin, pwm);
  if (motor_direction == kMotorDirectionBackward) {
    digitalWrite(RightMortorPin1, HIGH);
    digitalWrite(RightMortorPin2, LOW);
  } else if (motor_direction == kMotorDirectionForward) {
    digitalWrite(RightMortorPin1, LOW);
    digitalWrite(RightMortorPin2, HIGH);
  } else {
    analogWrite(RightMortorPwmPin, 0);
  }
}

void MotorControl(void *pvParameters) {

  (void)pvParameters;
  pinMode(LeftMortorPin1, OUTPUT);
  pinMode(LeftMortorPin2, OUTPUT);
  pinMode(LeftMortorPwmPin, OUTPUT);
  pinMode(RightMortorPin1, OUTPUT);
  pinMode(RightMortorPin2, OUTPUT);
  pinMode(RightMortorPwmPin, OUTPUT);

  TickType_t xLastWakeTime = xTaskGetTickCount();

  // initialize digital pin 13 as an output.
  // pinMode(LED_BUILTIN, OUTPUT);

  int local_motor_pwm = motor_pwm;
  enum RobotBehavior local_behavior_policy = behavior_policy;
  enum RemoteControl local_remote_control = remote_control;
  for (;;) // A Task shall never return or exit.
  {
    xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
    if (xSemaphoreTake(remote_control_signal, (TickType_t)10) == pdTRUE) {
      local_remote_control = remote_control;
    }
    if (local_remote_control == kRemoteControlStop) {
      LeftMotorControl(0, 0);
      RightMotorControl(0, 0);
      continue;
    }
    if (xSemaphoreTake(pwm_setup_signal, (TickType_t)10) == pdTRUE) {
      local_motor_pwm = motor_pwm;
    }
    Serial.println(local_motor_pwm);
    if (xSemaphoreTake(policy_signal, (TickType_t)10) == pdTRUE) {
      local_behavior_policy = behavior_policy;
    }
    if (local_behavior_policy == kRobotBehaviorGo) {
      LeftMotorControl(kMotorDirectionForward, local_motor_pwm);
      RightMotorControl(kMotorDirectionForward, local_motor_pwm);
    } else if (local_behavior_policy == kRobotBehaviorDodge) {
      LeftMotorControl(kMotorDirectionStop, 0);
      RightMotorControl(kMotorDirectionBackward, local_motor_pwm);
      // digitalWrite(LED_BUILTIN, LOW);
    }
  }
}

void SendPulse() {
  // Clears the kTrigPin condition
  digitalWrite(kTrigPin, LOW);
  delayMicroseconds(2);
  // Sets the kTrigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(kTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(kTrigPin, LOW);
}

void HandleEchoRisingEdge() { echo_initial_time = micros(); }

void HandleEchoFallingEdge() {
  long duration = micros() - echo_initial_time;
  int distance = duration * 0.0344 / 2;
  xHigherPriorityTaskWoken = pdFALSE;
  if (distance == 0 || distance > kDistanceFar) {
    behavior_policy = kRobotBehaviorGo;
    xSemaphoreGiveFromISR(policy_signal, &xHigherPriorityTaskWoken);
  } else if (distance < kDistanceNear) {
    behavior_policy = kRobotBehaviorDodge;
    xSemaphoreGiveFromISR(policy_signal, &xHigherPriorityTaskWoken);
  }
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void HandleEcho() {
  bool rising_edge = digitalRead(kEchoPin);
  if (rising_edge) {
    HandleEchoRisingEdge();
  } else {
    HandleEchoFallingEdge();
  }
}

void CollisionAvoidance(void *pvParameters) {

  (void)pvParameters;
  pinMode(kTrigPin, OUTPUT); // Sets the kTrigPin as an OUTPUT
  pinMode(kEchoPin, INPUT);  // Sets the echoPin as an INPUT
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
    SendPulse();
  }
}

void TaskSerial(void * /*pvParameters*/) {

  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) // A Task shall never return or exit.
  {
    xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
    if (Serial.available() > 0) {
      String command = Serial.readString();
      if (command.indexOf("+") != -1) {
        remote_control = kRemoteControlGo;
        xSemaphoreGive(remote_control_signal);
      } else if (command.indexOf("-") != -1) {
        remote_control = kRemoteControlStop;
        xSemaphoreGive(remote_control_signal);
      }
    }
  }
}

void TaskSetPWM(void * /*pvParameters*/) {

  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) // A Task shall never return or exit.
  {
    xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
    int sensor_value = analogRead(A0);
    motor_pwm = (int)(sensor_value / 1024.0 * 250);
    xSemaphoreGive(pwm_setup_signal);
  }
}
