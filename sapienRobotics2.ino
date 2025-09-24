//{"vL":"91","vR":"91"}
#include <ArduinoJson.h>
StaticJsonDocument<128> json;

float state_matrix[3][1] = { { 0 },
                             { 0 },
                             { 0 } };

float Pn[3][3] = { { 0, 0, 0 },
                   { 0, 0, 0 },
                   { 0, 0, 0 } };

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
  volatile int32_t output;
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
  const int frequency;
  const int resolution;
  volatile uint8_t enc_last;
  volatile int32_t counter;
  volatile int32_t prev_counter;
} motor_data;
motor_data left_motor = { 22, 23, 32, 33, 5000, 8, 0, 0, 0 };
motor_data right_motor = { 16, 17, 25, 26, 5000, 8, 0, 0, 0 };

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

void kalman_filter(bool jump) {
  //1. State Predict Xnp = AXn-1 + BUn + Wn, Wn = 0
  if (jump == 0) 
    goto predict_stage; 
  else 
    goto measurement_update_stage;

  predict_stage:
  static float A[3][3] = { { 1., 0., 0. },
                    { 0., 1., 0. },
                    { 0., 0., 1. } };
  static float Xn_1[3][1] = { { state_matrix[0][0] },
                       { state_matrix[1][0] },
                       { state_matrix[2][0] } };
  static float B[3][2] = { { cos(diff_drive_robot.theta + (diff_drive_robot.angular_displacement / 2.)), 0. },
                    { sin(diff_drive_robot.theta + (diff_drive_robot.angular_displacement / 2.)), 0. },
                    { 0., 1. } };
  static float Un[2][1] = { { diff_drive_robot.linear_displacement },
                     { diff_drive_robot.angular_displacement } };

  static float AXn_1[3][1] = { { 0. }, { 0. }, { 0. } };
  AXn_1[0][0] = A[0][0] * Xn_1[0][0] + A[0][1] * Xn_1[1][0] + A[0][2] * Xn_1[2][0];
  AXn_1[1][0] = A[1][0] * Xn_1[0][0] + A[1][1] * Xn_1[1][0] + A[1][2] * Xn_1[2][0];  // Fixed indexing
  AXn_1[2][0] = A[2][0] * Xn_1[0][0] + A[2][1] * Xn_1[1][0] + A[2][2] * Xn_1[2][0];  // Fixed indexing

  static float BUn[3][1] = { { 0. }, { 0. }, { 0. } };
  BUn[0][0] = B[0][0] * Un[0][0] + B[0][1] * Un[1][0];
  BUn[1][0] = B[1][0] * Un[0][0] + B[1][1] * Un[1][0];  // Fixed indexing
  BUn[2][0] = B[2][0] * Un[0][0] + B[2][1] * Un[1][0];  // Fixed indexing

  static float Xnp[3][1] = { { AXn_1[0][0] + BUn[0][0] },
                      { AXn_1[1][0] + BUn[1][0] },
                      { AXn_1[2][0] + BUn[2][0] } };

  //2. Process Covariance Pnp = (A2)Pn-1(A2t) + Qn, Qn = 0
  static float A2[3][3] = { { 1., 0., -diff_drive_robot.linear_displacement * sin(diff_drive_robot.theta + (diff_drive_robot.angular_displacement / 2)) },
                     { 0., 1., diff_drive_robot.linear_displacement * cos(diff_drive_robot.theta + (diff_drive_robot.angular_displacement / 2)) },
                     { 0., 0., 1. } };

  static float A2T[3][3] = { { 1., 0., 0. },
                      { 0., 1., 0. },
                      { -diff_drive_robot.linear_displacement * sin(diff_drive_robot.theta + (diff_drive_robot.angular_displacement / 2)), diff_drive_robot.linear_displacement * cos(diff_drive_robot.theta + (diff_drive_robot.angular_displacement / 2)), 1. } };

  static float A2_Pn_1[3][3] = { { A2[0][0] * Pn[0][0] + A2[0][1] * Pn[1][0] + A2[0][2] * Pn[2][0], A2[0][1] * Pn[0][1] + A2[0][1] * Pn[1][1] + A2[0][2] * Pn[2][1], A2[0][1] * Pn[0][2] + A2[0][1] * Pn[1][2] + A2[0][2] * Pn[2][2] },
                          { A2[1][0] * Pn[0][0] + A2[1][1] * Pn[1][0] + A2[1][2] * Pn[2][0], A2[1][0] * Pn[0][1] + A2[1][1] * Pn[1][1] + A2[1][2] * Pn[2][1], A2[1][1] * Pn[0][2] + A2[1][1] * Pn[1][2] + A2[1][2] * Pn[2][2] },
                          { A2[2][0] * Pn[0][0] + A2[2][1] * Pn[1][0] + A2[2][2] * Pn[2][0], A2[2][0] * Pn[0][1] + A2[2][1] * Pn[1][1] + A2[2][2] * Pn[2][1], A2[2][1] * Pn[0][2] + A2[2][1] * Pn[1][2] + A2[2][2] * Pn[2][2] } };

  static float Pnp[3][3] = { { A2_Pn_1[0][0] * A2T[0][0] + A2_Pn_1[0][1] * A2T[1][0] + A2_Pn_1[0][2] * A2T[2][0], A2_Pn_1[0][1] * A2T[0][1] + A2_Pn_1[0][1] * A2T[1][1] + A2_Pn_1[0][2] * A2T[2][1], A2_Pn_1[0][1] * A2T[0][2] + A2_Pn_1[0][1] * A2T[1][2] + A2_Pn_1[0][2] * A2T[2][2] },
                      { A2_Pn_1[1][0] * A2T[0][0] + A2_Pn_1[1][1] * A2T[1][0] + A2_Pn_1[1][2] * A2T[2][0], A2_Pn_1[1][0] * A2T[0][1] + A2_Pn_1[1][1] * A2T[1][1] + A2_Pn_1[1][2] * A2T[2][1], A2_Pn_1[1][1] * A2T[0][2] + A2_Pn_1[1][1] * A2T[1][2] + A2_Pn_1[1][2] * A2T[2][2] },
                      { A2_Pn_1[2][0] * A2T[0][0] + A2_Pn_1[2][1] * A2T[1][0] + A2_Pn_1[2][2] * A2T[2][0], A2_Pn_1[2][0] * A2T[0][1] + A2_Pn_1[2][1] * A2T[1][1] + A2_Pn_1[2][2] * A2T[2][1], A2_Pn_1[2][1] * A2T[0][2] + A2_Pn_1[2][1] * A2T[1][2] + A2_Pn_1[2][2] * A2T[2][2] } };

  measurement_update_stage:
  //3. Measurement Yn = CXnmeasurement + Zn, Zn = 0
  static float C[3][3] = { { 1., 0., 0. },
                    { 0., 1., 0. },
                    { 0., 0., 1. } };
  static float Xnmeasured[3][1] = { { diff_drive_robot.x },
                             { diff_drive_robot.y },
                             { diff_drive_robot.theta } };

  static float Yn[3][1] = { { C[0][0] * Xnmeasured[0][0] + C[0][1] * Xnmeasured[1][0] + C[0][2] * Xnmeasured[2][0] },
                     { C[1][0] * Xnmeasured[0][0] + C[1][1] * Xnmeasured[1][0] + C[1][2] * Xnmeasured[2][0] },
                     { C[2][0] * Xnmeasured[0][0] + C[2][1] * Xnmeasured[1][0] + C[2][2] * Xnmeasured[2][0] } };

  //4. Kalman Gain KG = Pnp/(Pnp + R)
  static float R[3][3] = { { 0.05, 0, 0 },
                    { 0, 0.05, 0 },
                    { 0, 0, 0.005 } };

  static float Pn_R[3][3] = { { Pnp[0][0] + R[0][0], Pnp[0][1] + R[0][1], Pnp[0][2] + R[0][2] },
                       { Pnp[1][0] + R[1][0], Pnp[1][1] + R[1][1], Pnp[1][2] + R[1][2] },
                       { Pnp[2][0] + R[2][0], Pnp[2][1] + R[2][1], Pnp[2][2] + R[2][2] } };

  static float KG[3][3] = { { Pnp[0][0] / Pn_R[0][0], Pnp[0][1] / Pn_R[0][1], Pnp[0][2] / Pn_R[0][2] },
                     { Pnp[1][0] / Pn_R[1][0], Pnp[1][1] / Pn_R[1][1], Pnp[1][2] / Pn_R[1][2] },
                     { Pnp[2][0] / Pn_R[2][0], Pnp[2][1] / Pn_R[2][1], Pnp[2][2] / Pn_R[2][2] } };

  //5. State Estimate Xe = Xnp + KG(Yn - Xnp)
  static float Yn_Xnp[3][1] = { { Yn[0][0] - Xnp[0][0] },
                         { Yn[1][0] - Xnp[1][0] },
                         { Yn[2][0] - Xnp[2][0] } };

  static float KG_Yn_Xnp[3][1] = { { KG[0][0] * Yn_Xnp[0][0] + KG[0][1] * Yn_Xnp[1][0] + KG[0][2] * Yn_Xnp[2][0] },
                            { KG[1][0] * Yn_Xnp[0][0] + KG[1][1] * Yn_Xnp[1][0] + KG[1][2] * Yn_Xnp[2][0] },
                            { KG[2][0] * Yn_Xnp[0][0] + KG[2][1] * Yn_Xnp[1][0] + KG[2][2] * Yn_Xnp[2][0] } };

  state_matrix[0][0] = Xnp[0][0] + KG_Yn_Xnp[0][0];
  state_matrix[1][0] = Xnp[1][0] + KG_Yn_Xnp[1][0];
  state_matrix[2][0] = Xnp[2][0] + KG_Yn_Xnp[2][0];

  //6. Process Covariance Pn = (I-KG)Pnp
  static float I_KG[3][3] = { { 1 - KG[0][0], 0 - KG[0][1], 0 - KG[0][2] },
                       { 0 - KG[1][0], 1 - KG[1][1], 0 - KG[1][2] },
                       { 0 - KG[2][0], 0 - KG[2][1], 1 - KG[2][2] } };

  Pn[0][0] = I_KG[0][0] * Pnp[0][0] + I_KG[0][1] * Pnp[1][0] + I_KG[0][2] * Pnp[2][0];
  Pn[0][1] = I_KG[0][0] * Pnp[0][1] + I_KG[0][1] * Pnp[1][1] + I_KG[0][2] * Pnp[2][1];
  Pn[0][2] = I_KG[0][0] * Pnp[0][2] + I_KG[0][1] * Pnp[1][2] + I_KG[0][2] * Pnp[2][2];
  Pn[1][0] = I_KG[1][0] * Pnp[0][0] + I_KG[1][1] * Pnp[1][0] + I_KG[1][2] * Pnp[2][0];
  Pn[1][1] = I_KG[1][0] * Pnp[0][1] + I_KG[1][1] * Pnp[1][1] + I_KG[1][2] * Pnp[2][1];
  Pn[1][2] = I_KG[1][0] * Pnp[0][2] + I_KG[1][1] * Pnp[1][2] + I_KG[1][2] * Pnp[2][2];
  Pn[2][0] = I_KG[2][0] * Pnp[0][0] + I_KG[2][1] * Pnp[1][0] + I_KG[2][2] * Pnp[2][0];
  Pn[2][1] = I_KG[2][0] * Pnp[0][1] + I_KG[2][1] * Pnp[1][1] + I_KG[2][2] * Pnp[2][1];
  Pn[2][2] = I_KG[2][0] * Pnp[0][2] + I_KG[2][1] * Pnp[1][2] + I_KG[2][2] * Pnp[2][2];
}

void update_odometry(void* args) {
  while (true) {
    vTaskDelay(pdMS_TO_TICKS(20));
    
    kalman_filter(0);
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
    kalman_filter(1);
    diff_drive_robot.angular_velocity = (diff_drive_robot.angular_displacement / diff_drive_robot.time_interval);
    Serial.printf("{x: %.2f, y: %.2f, theta: %.2f, v: %.2f, w: %.2f}\n", diff_drive_robot.x, diff_drive_robot.y, diff_drive_robot.theta, diff_drive_robot.linear_velocity, diff_drive_robot.angular_velocity);
    Serial.printf("{X: %.2f, Y: %.2f, Theta: %.2f, v: %.2f, w: %.2f}\n", state_matrix[0][0], state_matrix[1][0], state_matrix[2][0], diff_drive_robot.linear_velocity, diff_drive_robot.angular_velocity);
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
      if (left_motor_pid.output >= 0) {
        analogWrite(left_motor.MF_PIN, left_motor_pid.output);
        analogWrite(left_motor.MB_PIN, 0);
      } else {
        analogWrite(left_motor.MF_PIN, 0);
        analogWrite(left_motor.MB_PIN, abs(left_motor_pid.output));
      }

      if (right_motor_pid.output >= 0) {
        analogWrite(right_motor.MF_PIN, right_motor_pid.output);
        analogWrite(right_motor.MB_PIN, 0);
      } else {
        analogWrite(right_motor.MF_PIN, 0);
        analogWrite(right_motor.MB_PIN, abs(right_motor_pid.output));
      }
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

  xTaskCreatePinnedToCore(update_odometry, "Odometry Updates", 10000, NULL, 1, NULL, 1);
  // xTaskCreatePinnedToCore(pid_controller, "PID Controller", 4096, NULL, 1, NULL, 1);
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

      if (left_motor_pid.setpoint > 0) {
        analogWrite(left_motor.MF_PIN, left_motor_pid.setpoint);
        analogWrite(left_motor.MB_PIN, 0);
      } else {
        analogWrite(left_motor.MF_PIN, 0);
        analogWrite(left_motor.MB_PIN, abs(left_motor_pid.setpoint));
      };

      if (right_motor_pid.setpoint > 0) {
        analogWrite(right_motor.MF_PIN, right_motor_pid.setpoint);
        analogWrite(right_motor.MB_PIN, 0);
      } else {
        analogWrite(right_motor.MF_PIN, 0);
        analogWrite(right_motor.MB_PIN, abs(right_motor_pid.setpoint));
      };

      if (left_motor_pid.setpoint == 0 && right_motor_pid.setpoint == 0) moving = false;
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
  if (output > 255) output = 255;
  else if (output < -255) output = -255;
  Serial.print("counter: " + String(controller->Encoder));
  Serial.print(" value: " + String(value));
  Serial.print(" error: " + String(error));
  Serial.print(" integral: " + String(controller->Pcumlative));
  Serial.print(" derivative: " + String(derivative));
  Serial.println(" output: " + String(output));
  controller->Perror = error;
  controller->prevEncoder = controller->Encoder;
  controller->output = output;
}
