#include <Arduino_FreeRTOS.h>
#include <PID_v1.h>

#include "semphr.h"

#define LeftMortorPin1 7
#define LeftMortorPin2 8
#define LeftMortorPwmPin 5
#define RightMortorPin1 9
#define RightMortorPin2 10
#define RightMortorPwmPin 6

#define kDistanceNear 60
#define kDistanceFar 80

// attach pins to HC-SR04
#define kRightWheelEncoderPin 2
#define kLeftWheelEncoderPin 3
#define kEchoPin 11
#define kTrigPin 4

#define USE_TIMER_1 true

#include "TimerInterrupt.h"
#define kTimer1IntervalMs 100
// 3.1415926 * 0.065 / 20 // pi * r * 2 / resolution
#define kMeterPerResolution 0.01

enum RemoteControl { kRemoteControlManual = 0, kRemoteControlAuto = 1 };
enum MotorDirection {
  kMotorDirectionBackward = -1,
  kMotorDirectionStop = 0,
  kMotorDirectionForward = 1
};
enum RobotBehavior { kRobotBehaviorGo, kRobotBehaviorDodge };

SemaphoreHandle_t policy_signal;
SemaphoreHandle_t wheel_speed_signal;
SemaphoreHandle_t remote_control_signal;
SemaphoreHandle_t pwm_setup_signal;

volatile enum RobotBehavior behavior_policy = kRobotBehaviorGo;
enum RemoteControl remote_control = kRemoteControlAuto;

enum MotorDirection motor_manual_direction = kMotorDirectionStop;
int motor_pwm = 0;      // Value is set based on potentiometer
int left_motor_pwm = 0; // Values are set based on remote control commands
int right_motor_pwm = 0;
double target_speed = 0.5; //[m/s]

volatile double wheel_speed = 0.0;
volatile double left_wheel_speed = 0.0;
volatile double right_wheel_speed = 0.0;
volatile unsigned long echo_initial_time = 0;
volatile double object_distance = 0.0;
volatile unsigned long right_wheel_encoder_cnt = 0;
volatile unsigned long left_wheel_encoder_cnt = 0;
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

void MotorControl(void *pvParameters);
void CollisionAvoidance(void *pvParameters);
void TaskSerial(void *pvParameters);
void TaskSetPWM(void *pvParameters);

void LeftMotorControl(enum MotorDirection motor_direction, int pwm);
void RightMotorControl(enum MotorDirection motor_direction, int pwm);

#define kMotorControlIntervalMs 150

#define dir(x)                                                                 \
  ((x) < 0 ? kMotorDirectionBackward                                           \
           : ((x) > 0 ? kMotorDirectionForward : kMotorDirectionStop))

// The setup function runs once when you press reset or power the board
void setup() {
  // pin change interrupt (example for D11: kEchoPin)
  // Interrupt Service Routine: ISR(PCINT0_vect) {} // end of PCINT0_vect
  PCMSK0 |= bit(PCINT3); // want pin 11
  PCIFR |= bit(PCIF0);   // clear any outstanding interrupts
  PCICR |= bit(PCIE0);   // enable pin change interrupts for D8 to D13

  policy_signal = xSemaphoreCreateBinary();
  wheel_speed_signal = xSemaphoreCreateBinary();
  remote_control_signal = xSemaphoreCreateBinary();
  pwm_setup_signal = xSemaphoreCreateBinary();

  Serial.begin(9600);

  ITimer1.init();
  if (ITimer1.attachInterruptInterval(kTimer1IntervalMs, HandleTimer)) {
    // Serial.print(F("Starting  ITimer OK, millis() = "));
    // Serial.println(millis());
  }

  pinMode(kEchoPin, INPUT_PULLUP);
  pinMode(kRightWheelEncoderPin, INPUT_PULLUP);
  pinMode(kLeftWheelEncoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(kRightWheelEncoderPin),
                  HandleRightWheelEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(kLeftWheelEncoderPin),
                  HandleLeftWheelEncoder, RISING);

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
              128, // This stack size can be checked & adjusted by reading the
                   // Stack Highwater
              NULL,
              2, // Priority, with 1 being the highest, and 4 being the lowest.
              NULL);

  xTaskCreate(TaskSerial,
              "Serial", // A name just for humans
              256, // This stack size can be checked & adjusted by reading the
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

  double local_wheel_speed = wheel_speed;
  // double local_target_speed = target_speed;
  int local_motor_pwm = motor_pwm;
  enum RobotBehavior local_behavior_policy = behavior_policy;
  enum RemoteControl local_remote_control = remote_control;
  enum MotorDirection local_motor_manual_direction = motor_manual_direction;

  // Specify the links and initial tuning parameters
  double Kp = 2.0, Ki = 1.0, Kd = 0;
  double output_pwm;
  PID speed_pid(&wheel_speed, &output_pwm, &target_speed, Kp, Ki, Kd, DIRECT);
  speed_pid.SetMode(AUTOMATIC);
  speed_pid.SetSampleTime(kMotorControlIntervalMs);
  speed_pid.SetOutputLimits(0, 1);

  for (;;) // A Task shall never return or exit.
  {
    /* Inspect our own high water mark on entering the task. */
    // Serial.print("Motor control highwater mark:");
    // Serial.println(uxTaskGetStackHighWaterMark(NULL));
    // Serial.print("object distance: ");
    // Serial.println(object_distance);
    xTaskDelayUntil(&xLastWakeTime,
                    kMotorControlIntervalMs / portTICK_PERIOD_MS);
    if (xSemaphoreTake(remote_control_signal, (TickType_t)10) == pdTRUE) {
      local_remote_control = remote_control;
      local_motor_manual_direction = motor_manual_direction;
      if (local_remote_control == kRemoteControlManual) {
        speed_pid.SetMode(MANUAL);
      } else {
        speed_pid.SetMode(AUTOMATIC);
      }
    }
    if (xSemaphoreTake(policy_signal, (TickType_t)10) == pdTRUE) {
      local_behavior_policy = behavior_policy;
    }
    if (local_remote_control == kRemoteControlManual) {
      if (local_behavior_policy == kRobotBehaviorDodge &&
          motor_manual_direction == kMotorDirectionForward) {
        LeftMotorControl(kMotorDirectionStop, 0);
        RightMotorControl(kMotorDirectionStop, 0);
        // digitalWrite(LED_BUILTIN, LOW);
        continue;
      }
      LeftMotorControl(local_motor_manual_direction, left_motor_pwm);
      RightMotorControl(local_motor_manual_direction, right_motor_pwm);
      continue;
    }
    if (xSemaphoreTake(pwm_setup_signal, (TickType_t)10) == pdTRUE) {
      local_motor_pwm = motor_pwm;
    }
    if (xSemaphoreTake(wheel_speed_signal, (TickType_t)10) == pdTRUE) {
      local_wheel_speed = wheel_speed;
    }
    if (abs(local_wheel_speed - target_speed) < 0.05) {
      local_wheel_speed = target_speed;
    }
    speed_pid.Compute();
    // Serial.print("left_speed:");
    // Serial.print(10 * left_wheel_speed);
    // Serial.print(",");
    // Serial.print("right_speed:");
    // Serial.println(10 * right_wheel_speed);
    if (local_behavior_policy == kRobotBehaviorGo) {
      LeftMotorControl(kMotorDirectionForward, output_pwm * local_motor_pwm);
      RightMotorControl(kMotorDirectionForward, output_pwm * local_motor_pwm);
    } else if (local_behavior_policy == kRobotBehaviorDodge) {
      LeftMotorControl(kMotorDirectionStop, 0);
      RightMotorControl(kMotorDirectionBackward, output_pwm * local_motor_pwm);
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
  object_distance = duration * 0.0344 / 2;

  xHigherPriorityTaskWoken = pdFALSE;
  if (object_distance == 0 || object_distance > kDistanceFar) {
    behavior_policy = kRobotBehaviorGo;
    xSemaphoreGiveFromISR(policy_signal, &xHigherPriorityTaskWoken);
  } else if (object_distance < kDistanceNear) {
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

ISR(PCINT0_vect) { HandleEcho(); } // end of PCINT0_vect

// the PWM pins:
// - Pins 5 and 6: controlled by Timer 0
// - Pins 9 and 10: controlled by timer 1
// - Pins 11 and 3: controlled by timer 2
void HandleTimer() {
  xHigherPriorityTaskWoken = pdFALSE;
  left_wheel_speed = 0.7 * left_wheel_speed + 0.3 * kMeterPerResolution *
                                                  right_wheel_encoder_cnt *
                                                  1000 / kTimer1IntervalMs;
  right_wheel_speed = 0.7 * right_wheel_speed + 0.3 * kMeterPerResolution *
                                                    right_wheel_encoder_cnt *
                                                    1000 / kTimer1IntervalMs;
  right_wheel_encoder_cnt = 0;
  left_wheel_encoder_cnt = 0;
  // Hack
  wheel_speed = right_wheel_speed;
  xSemaphoreGiveFromISR(wheel_speed_signal, &xHigherPriorityTaskWoken);
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void HandleLeftWheelEncoder() { left_wheel_encoder_cnt++; }
void HandleRightWheelEncoder() { right_wheel_encoder_cnt++; }

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
    // TODO(XP): Make the parsing faster!!!
    // Serial.print("serial highwater mark:");
    // Serial.println(uxTaskGetStackHighWaterMark(NULL));
    xTaskDelayUntil(&xLastWakeTime, 5 / portTICK_PERIOD_MS);
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('#');
      // Serial.println(command);
      int pos_s = 0;
      int pos_e = command.indexOf(';', pos_s);
      String local_mode = command.substring(pos_s, pos_e);
      if (pos_e == -1) {
        continue;
      }
      pos_s = pos_e + 1;
      pos_e = command.indexOf(';', pos_s);
      if (pos_e == -1) {
        continue;
      }
      String s_motor_direction = command.substring(pos_s, pos_e);
      pos_s = pos_e + 1;
      pos_e = command.indexOf(';', pos_s);
      if (pos_e == -1) {
        continue;
      }
      String s_left_motor_pwm = command.substring(pos_s, pos_e);
      pos_s = pos_e + 1;
      pos_e = command.indexOf(';', pos_s);
      if (pos_e == -1) {
        continue;
      }
      String s_right_motor_pwm = command.substring(pos_s, pos_e);
      left_motor_pwm = s_left_motor_pwm.toInt();
      right_motor_pwm = s_right_motor_pwm.toInt();
      if (local_mode == "A") {
        remote_control = kRemoteControlAuto;
        xSemaphoreGive(remote_control_signal);
      } else {
        left_motor_pwm = s_left_motor_pwm.toInt();
        right_motor_pwm = s_right_motor_pwm.toInt();
        if (s_motor_direction == "F") {
          motor_manual_direction = kMotorDirectionForward;
        } else if (s_motor_direction == "B") {
          motor_manual_direction = kMotorDirectionBackward;
        } else {
          motor_manual_direction = kMotorDirectionStop;
        }
        remote_control = kRemoteControlManual;
        xSemaphoreGive(remote_control_signal);
      }
    }
  }
}

void TaskSetPWM(void * /*pvParameters*/) {

  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) // A Task shall never return or exit.
  {
    // Serial.print("Set pwm highwater mark:");
    // Serial.println(uxTaskGetStackHighWaterMark(NULL));
    xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
    int sensor_value = analogRead(A0);
    motor_pwm = (int)(sensor_value / 1024.0 * 250);
    xSemaphoreGive(pwm_setup_signal);
  }
}
