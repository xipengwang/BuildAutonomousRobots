#include <Wire.h>
#include <math.h>

#include <Arduino_FreeRTOS.h>
#include <PID_v1.h>

#include "semphr.h"

#define MPU6050 0x68
#define LeftMortorPin1 7
#define LeftMortorPin2 8
#define LeftMortorPwmPin 5
#define RightMortorPin1 9
#define RightMortorPin2 10
#define RightMortorPwmPin 6

#define kDistanceNear 60
#define kDistanceFar 80

#define motor_pwm 230

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

enum MotorDirection {
  kMotorDirectionBackward = -1,
  kMotorDirectionStop = 0,
  kMotorDirectionForward = 1
};
enum RobotBehavior { kRobotBehaviorGo, kRobotBehaviorDodge };

SemaphoreHandle_t policy_signal;
SemaphoreHandle_t wheel_speed_signal;
SemaphoreHandle_t yaw_signal;

volatile enum RobotBehavior behavior_policy = kRobotBehaviorGo;

double left_target_speed = 0.5;  //[m/s]
double right_target_speed = 0.5; //[m/s]

volatile double left_wheel_speed = 0.0;
volatile double right_wheel_speed = 0.0;
volatile unsigned long echo_initial_time = 0;
volatile double object_distance = 0.0;
volatile unsigned long right_wheel_encoder_cnt = 0;
volatile unsigned long left_wheel_encoder_cnt = 0;
BaseType_t xHigherPriorityTaskWoken = pdFALSE;

double yaw_bias = 0.0;
double yaw = 0;
double pos_x = 0, pos_y = 0;

void MotorControl(void *pvParameters);
void CollisionAvoidance(void *pvParameters);
void HandleIMU(void *pvParameters);

void LeftMotorControl(enum MotorDirection motor_direction, int pwm);
void RightMotorControl(enum MotorDirection motor_direction, int pwm);

#define kMotorControlIntervalMs 150

#define dir(x)                                                                 \
  ((x) < 0 ? kMotorDirectionBackward                                           \
           : ((x) > 0 ? kMotorDirectionForward : kMotorDirectionStop))

// The setup function runs once when you press reset or power the board
void setup() {
  constexpr int kMinimalCaliCnt = 200;
  int cali_sample_cnt = 0;

  StartIMU();
  // TODO(XW): estimate the bias when speed is 0.
  while (cali_sample_cnt < kMinimalCaliCnt) {
    yaw_bias += QueryGyroZ();
    cali_sample_cnt++;
  }
  yaw_bias /= kMinimalCaliCnt;
  // pin change interrupt (example for D11: kEchoPin)
  // Interrupt Service Routine: ISR(PCINT0_vect) {} // end of PCINT0_vect
  PCMSK0 |= bit(PCINT3); // want pin 11
  PCIFR |= bit(PCIF0);   // clear any outstanding interrupts
  PCICR |= bit(PCIE0);   // enable pin change interrupts for D8 to D13

  policy_signal = xSemaphoreCreateBinary();
  wheel_speed_signal = xSemaphoreCreateBinary();
  yaw_signal = xSemaphoreCreateBinary();

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

  xTaskCreate(MotorControl, "MortorControl", 256, NULL, 2, NULL);

  xTaskCreate(CollisionAvoidance, "CollisionAvoidance", 128, NULL, 2, NULL);

  xTaskCreate(HandleIMU, "HandleIMU", 256, NULL, 2, NULL);

  Serial.begin(9600);

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

  // TickType_t xLastWakeTime = xTaskGetTickCount();

  // initialize digital pin 13 as an output.
  // pinMode(LED_BUILTIN, OUTPUT);

  double local_right_wheel_speed = right_wheel_speed;
  double local_left_wheel_speed = left_wheel_speed;
  double local_yaw = yaw;
  enum RobotBehavior local_behavior_policy = behavior_policy;

  // Specify the links and initial tuning parameters
  double Kp = 2.0, Ki = 1.0, Kd = 0;
  double l_output_pwm;
  double r_output_pwm;
  PID l_speed_pid(&local_left_wheel_speed, &l_output_pwm, &left_target_speed,
                  Kp, Ki, Kd, DIRECT);
  PID r_speed_pid(&local_right_wheel_speed, &r_output_pwm, &right_target_speed,
                  Kp, Ki, Kd, DIRECT);
  l_speed_pid.SetMode(AUTOMATIC);
  l_speed_pid.SetSampleTime(kMotorControlIntervalMs);
  l_speed_pid.SetOutputLimits(0, 1);
  r_speed_pid.SetMode(AUTOMATIC);
  r_speed_pid.SetSampleTime(kMotorControlIntervalMs);
  r_speed_pid.SetOutputLimits(0, 1);

  unsigned long current_time = 0, previous_time = 0;
  double previous_speed =
      (local_left_wheel_speed + local_right_wheel_speed) / 2;
  double previous_yaw = local_yaw;
  double current_speed = previous_speed;
  double current_yaw = previous_yaw;
  for (;;) // A Task shall never return or exit.
  {
    /* Inspect our own high water mark on entering the task. */
    // Serial.print("Motor control highwater mark:");
    // Serial.println(uxTaskGetStackHighWaterMark(NULL));
    // Serial.print("object distance: ");
    // Serial.println(object_distance);
    // xTaskDelayUntil(&xLastWakeTime,
    //                 kMotorControlIntervalMs / portTICK_PERIOD_MS);

    current_time = millis();
    if (current_time - previous_time < kMotorControlIntervalMs) {
      continue;
    }
    if (xSemaphoreTake(policy_signal, (TickType_t)10) == pdTRUE) {
      local_behavior_policy = behavior_policy;
    }

    if (xSemaphoreTake(wheel_speed_signal, (TickType_t)10) == pdTRUE) {
      local_right_wheel_speed = right_wheel_speed;
      local_left_wheel_speed = left_wheel_speed;
    }

    if (xSemaphoreTake(yaw_signal, (TickType_t)10) == pdTRUE) {
      local_yaw = yaw;
    }

    current_speed = (local_left_wheel_speed + local_right_wheel_speed) / 2;
    current_yaw = local_yaw;
    double move_dist = (current_speed + previous_speed) *
                       (current_time - previous_time) / 2000;
    double heading_change = current_yaw - previous_yaw;
    if (heading_change > 90) {
      heading_change -= 360;
    } else if (heading_change < -90) {
      heading_change += 360.0;
    }
    double heading_change_rad = heading_change * 3.1415926 / 180.0;
    double delta_x = move_dist;
    double delta_y = 0;
    if (heading_change != 0) {
      double R = move_dist / heading_change_rad;
      delta_x = R * sin(heading_change_rad);
      delta_y = R * (1 - cos(heading_change_rad));
    }
    if (heading_change_rad < 0) {
      delta_y *= -1;
    }
    double c = cos(previous_yaw * 3.1415926 / 180.0);
    double s = sin(previous_yaw * 3.1415926 / 180.0);
    pos_x += c * delta_x - s * delta_y;
    pos_y += s * delta_x + c * delta_y;

    Serial.print("y:");
    Serial.print(pos_y);
    Serial.print(",");
    Serial.print("x:");
    Serial.println(pos_x);
    previous_speed = current_speed;
    previous_yaw = current_yaw;
    previous_time = current_time;

    if (abs(local_left_wheel_speed - left_target_speed) < 0.05) {
      local_left_wheel_speed = left_target_speed;
    }
    if (abs(local_right_wheel_speed - right_target_speed) < 0.05) {
      local_right_wheel_speed = right_target_speed;
    }

    if (local_behavior_policy == kRobotBehaviorGo) {
      left_target_speed = 0.5;
      right_target_speed = 0.5;
    } else if (local_behavior_policy == kRobotBehaviorDodge) {
      left_target_speed = 0.3;
      right_target_speed = 0;
      // digitalWrite(LED_BUILTIN, LOW);
    }
    l_speed_pid.Compute();
    r_speed_pid.Compute();
    LeftMotorControl(kMotorDirectionForward, l_output_pwm * motor_pwm);
    RightMotorControl(kMotorDirectionForward, r_output_pwm * motor_pwm);
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
                                                  left_wheel_encoder_cnt *
                                                  1000 / kTimer1IntervalMs;
  right_wheel_speed = 0.7 * right_wheel_speed + 0.3 * kMeterPerResolution *
                                                    right_wheel_encoder_cnt *
                                                    1000 / kTimer1IntervalMs;
  right_wheel_encoder_cnt = 0;
  left_wheel_encoder_cnt = 0;
  // Hack
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
                             // TickType_t xLastWakeTime = xTaskGetTickCount();
  unsigned long current_time = 0, previous_time = 0;
  for (;;) {
    // Serial.print("collision highwater mark:");
    // Serial.println(uxTaskGetStackHighWaterMark(NULL));

    // xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
    current_time = millis();
    if (current_time - previous_time < 100) {
      continue;
    }
    previous_time = current_time;
    SendPulse();
  }
}

double WrapAngle(double x) {
  x = fmod(x + 180, 360);
  if (x < 0)
    x += 360;
  return x - 180;
}

void HandleIMU(void * /*pvParameters*/) {
  // TickType_t xLastWakeTime = xTaskGetTickCount();
  float previous_gyro_z = 0.0;
  float gyro_z = 0.0;
  bool first = true;
  unsigned long current_time, previous_time;
  for (;;) // A Task shall never return or exit.
  {
    // xTaskDelayUntil(&xLastWakeTime, 50 / portTICK_PERIOD_MS);
    if (millis() - previous_time < 50) {
      continue;
    }
    float gyro_z = QueryGyroZ() - yaw_bias;
    current_time = millis();
    if (first) {
      previous_gyro_z = gyro_z;
      previous_time = current_time;
      first = false;
      continue;
    }
    // Serial.print("gyro_z:");
    // Serial.print(gyro_z * 3.1415926 / 180.0);
    // Serial.print(",");
    // Serial.print("x:");
    // Serial.print(pos_x);
    // Serial.print(",");
    // Serial.print("y:");
    // Serial.print(pos_y);
    // Serial.print(",");
    // Serial.print("yaw:");
    // Serial.println(yaw * 3.1415926 / 180.0);
    current_time = millis();
    yaw = WrapAngle(yaw + (previous_gyro_z + gyro_z) *
                              (current_time - previous_time) / 2000);
    xSemaphoreGive(yaw_signal);
    previous_gyro_z = gyro_z;
    previous_time = current_time;
  }
}

void StartIMU() {
  Wire.begin(); // Initialize comunication
  Wire.beginTransmission(
      MPU6050);               // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);           // Talk to the register 6B
  Wire.write(0);              // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true); // end the transmission
}

double QueryGyroZ() {
  Wire.beginTransmission(MPU6050);
  Wire.write(0x47);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 2, true);
  if (Wire.available() == 2) {
    return (Wire.read() << 8 | Wire.read()) / 131.0;
  }
}
