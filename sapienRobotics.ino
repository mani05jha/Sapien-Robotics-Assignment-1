#include <ArduinoJson.h>
StaticJsonDocument<128> json;

const float Kp = 0.08;
const float Ki = 2;
const float Kd = 0.01;
volatile bool moving = false;

typedef struct {
  volatile int32_t setpoint;
  volatile int32_t Perror;
  volatile int32_t Encoder;
  volatile int32_t prevEncoder;
  volatile float Pcumlative;
  volatile int8_t output;
} pid;
pid left_motor_pid = { 0, 0, 0, 0, 0, 0 };
pid right_motor_pid = { 0, 0, 0, 0, 0, 0 };

typedef struct {
  const float wheel_radius;
  const float track_width;
  const uint16_t count_per_rev;
  const float time_interval;
  const float pi;
  volatile float linear_displacement;
  volatile float angular_displacement;
  volatile float x;
  volatile float y;
  volatile float theta;
  volatile float linear_velocity;
  volatile float angular_velocity;
} robot_data;

robot_data diff_drive_robot = { 0.035, 0.295, 2750, 0.02, M_PI, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

typedef struct {
  const uint8_t ENCA;
  const uint8_t ENCB;
  const uint8_t MF_PIN;
  const uint8_t MB_PIN;
  volatile uint8_t enc_last;
  volatile int32_t counter;
  volatile int32_t prev_counter;

} motor_data;
motor_data left_motor = { 23, 22, 33, 32, 0, 0, 0 };
motor_data right_motor = { 16, 17, 26, 25, 0, 0, 0 };

// Encoder lookup table to decode transitions
const int8_t ENC_STATES[] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

void IRAM_ATTR update_encoders_left() {
  left_motor.enc_last <<= 2;
  left_motor.enc_last |= ((digitalRead(left_motor.ENCA) << 1) | digitalRead(left_motor.ENCB));

  left_motor.counter += ENC_STATES[left_motor.enc_last & 0x0F];
}

void IRAM_ATTR update_encoders_right() {
  right_motor.enc_last <<= 2;
  right_motor.enc_last |= ((digitalRead(right_motor.ENCA) << 1) | digitalRead(right_motor.ENCB));

  right_motor.counter += ENC_STATES[right_motor.enc_last & 0x0F];
}

void update_odometry(void* args) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(20));
    diff_drive_robot.linear_displacement = (((2 * diff_drive_robot.pi * diff_drive_robot.wheel_radius * (left_motor.counter - left_motor.prev_counter)) / diff_drive_robot.count_per_rev) / 2) + (((2 * diff_drive_robot.pi * diff_drive_robot.wheel_radius * (right_motor.counter - right_motor.prev_counter)) / diff_drive_robot.count_per_rev) / 2);

    diff_drive_robot.angular_displacement = (((2 * diff_drive_robot.pi * diff_drive_robot.wheel_radius * (right_motor.counter - right_motor.prev_counter)) / diff_drive_robot.count_per_rev) / diff_drive_robot.track_width) - (((2 * diff_drive_robot.pi * diff_drive_robot.wheel_radius * (left_motor.counter - left_motor.prev_counter)) / diff_drive_robot.count_per_rev) / diff_drive_robot.track_width);

    left_motor.prev_counter = left_motor.counter;
    right_motor.prev_counter = right_motor.counter;

    diff_drive_robot.x += diff_drive_robot.linear_displacement * cos(diff_drive_robot.theta + (diff_drive_robot.angular_displacement / 2));
    diff_drive_robot.y += diff_drive_robot.linear_displacement * sin(diff_drive_robot.theta + (diff_drive_robot.angular_displacement / 2));
    diff_drive_robot.theta += diff_drive_robot.angular_displacement;
    while (diff_drive_robot.theta > diff_drive_robot.pi) {
      diff_drive_robot.theta -= 2 * diff_drive_robot.pi;
    }
    while (diff_drive_robot.theta < -diff_drive_robot.pi) {
      diff_drive_robot.theta += 2 * diff_drive_robot.pi;
    }
    diff_drive_robot.linear_velocity = (diff_drive_robot.linear_displacement / diff_drive_robot.time_interval);
    diff_drive_robot.angular_velocity = (diff_drive_robot.angular_displacement / diff_drive_robot.time_interval);
    Serial.printf("{x: %.2f, y: %.2f, Theta: %.2f, v: %.2f, w: %.2f\n", diff_drive_robot.x, diff_drive_robot.y, diff_drive_robot.theta, diff_drive_robot.linear_velocity, diff_drive_robot.angular_velocity);
  }
}

void pid_controller(void* args) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(33));
    if (moving) {
      left_motor_pid.Encoder = left_motor.counter;
      right_motor_pid.Encoder = right_motor.counter;
      pid_control_unit(&left_motor_pid);
      pid_control_unit(&right_motor_pid);
      if (left_motor_pid.output >= 0) { analogWrite(left_motor.MF_PIN, left_motor_pid.output); analogWrite(left_motor.MB_PIN, 0); }
      else { analogWrite(left_motor.MF_PIN, 0); analogWrite(left_motor.MB_PIN, abs(left_motor_pid.output)); }

      if (right_motor_pid.output >= 0) { analogWrite(right_motor.MF_PIN, right_motor_pid.output); analogWrite(right_motor.MB_PIN, 0); }
      else { analogWrite(right_motor.MF_PIN, 0); analogWrite(right_motor.MB_PIN, abs(right_motor_pid.output)); }
    } else {
      left_motor_pid.setpoint = 0;
      left_motor_pid.Encoder = left_motor.counter;
      left_motor_pid.prevEncoder = left_motor_pid.Encoder;
      left_motor_pid.Perror = 0;
      left_motor_pid.Pcumlative = 0.0;
      left_motor_pid.output = 0;
      analogWrite(left_motor.MF_PIN, 0);
      analogWrite(left_motor.MB_PIN, 0);

      right_motor_pid.setpoint = 0;
      right_motor_pid.Encoder = right_motor.counter;
      right_motor_pid.prevEncoder = right_motor_pid.Encoder;
      right_motor_pid.Perror = 0;
      right_motor_pid.Pcumlative = 0.0;
      right_motor_pid.output = 0;
      analogWrite(right_motor.MF_PIN, 0);
      analogWrite(right_motor.MB_PIN, 0);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(left_motor.ENCA, INPUT_PULLUP);
  pinMode(left_motor.ENCB, INPUT_PULLUP);
  pinMode(left_motor.MF_PIN, OUTPUT);
  pinMode(left_motor.MB_PIN, OUTPUT);

  pinMode(right_motor.ENCA, INPUT_PULLUP);
  pinMode(right_motor.ENCB, INPUT_PULLUP);
  pinMode(right_motor.MF_PIN, OUTPUT);
  pinMode(right_motor.MB_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(left_motor.ENCA), update_encoders_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_motor.ENCB), update_encoders_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_motor.ENCA), update_encoders_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_motor.ENCB), update_encoders_right, CHANGE);

  xTaskCreatePinnedToCore(update_odometry, "Odometry Updates", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(pid_controller, "PID Controller", 4096, NULL, 1, NULL, 1);
}

void loop() {
  uint8_t vL, vR;
  while (Serial.available() > 0) {
    String c = Serial.readStringUntil('\n');
    c += '\0';
    DeserializationError error = deserializeJson(json, c);
    if (!error) {
      left_motor_pid.setpoint = atoi(json["vL"]);
      right_motor_pid.setpoint = atoi(json["vR"]);
      if (left_motor_pid.setpoint == 0 &&  right_motor_pid.setpoint == 0) moving = false;
      else moving = true;
    }
  }
  delay(10);
}

void pid_control_unit(pid* controller) {
  int32_t value = controller->Encoder - controller->prevEncoder;
  int32_t error = (controller->setpoint - value);
  controller->Pcumlative = controller->Pcumlative + (error * 0.033);
  float derivative = (error - controller->Perror) / 0.033;
  int16_t output = Kp * error + Ki * controller->Pcumlative + Kd * derivative;
  Serial.print("counter: " + String(controller->Encoder));
  Serial.print(" value: " + String(value));
  Serial.print(" error: " + String(error));
  Serial.print(" integral: " + String(controller->Pcumlative));
  Serial.print(" derivative: " + String(derivative));
  Serial.println(" output: " + String(output));
  if (output > 255) output = 255;
  else if (output < -255) output = -255;
  controller->Perror = error;
  controller->prevEncoder = controller->Encoder;
  controller->output = output;
}
