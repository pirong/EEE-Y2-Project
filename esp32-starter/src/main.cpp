#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <TimerInterrupt_Generic.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <step.h>

#define ADC_5V_CURRENT   32  // ADC pin for 5V current (INA122)      
#define ADC_MOTOR        34  // ADC pin for motor current (TLV2372)
#define ADC_VBAT         35  // ADC pin for battery voltage  
#define ADC_5V_VOLTAGE   39  // ADC pin for 5V rail voltage

// Hotspot parameters
const char* ssid = "IMAS";
const char* pass = "Sami1111";
WebSocketsServer webSocket(81);

// Robot measurements
const float WHEEL_RADIUS = 0.0325;
const float WHEEL_BASE = 0.115;

// Desired robot speeds
const float FORWARD_SPEED_MPS = 0.1625;
const float ANGULAR_SPEED_RADPS = 0.5;

// Corresponding motor speeds
const float FORWARD_SPEED = FORWARD_SPEED_MPS / WHEEL_RADIUS;
const float ANGULAR_SPEED = ANGULAR_SPEED_RADPS * WHEEL_BASE / WHEEL_RADIUS;

// Safety constraint
const float MAX_TILT_ANGLE_RAD = 0.03;

// The Stepper pins
const int STEPPER1_DIR_PIN  = 16;
const int STEPPER1_STEP_PIN = 17;
const int STEPPER2_DIR_PIN  = 4;
const int STEPPER2_STEP_PIN = 14;
const int STEPPER_EN_PIN    = 15; 

// ADC pins
const int ADC_CS_PIN        = 5;
const int ADC_SCK_PIN       = 18;
const int ADC_MISO_PIN      = 19;
const int ADC_MOSI_PIN      = 23;

// Diagnostic pin for oscilloscope
const int TOGGLE_PIN        = 32;

// Some time intervals
const int PRINT_INTERVAL      = 110;
const int LOOP_INTERVAL_INNER = 10;
const int LOOP_INTERVAL_OUTER = 100;
const int ODOM_INTERVAL = 10; //ASK LOUIS
const int POWER_INTERVAL = 200;
const int STEPPER_INTERVAL_US = 20;

const float VREF = 4.096;

// Complementary filter constant
const float alpha = 0.98;

// PID constants for tilt control (inner loop)
const float Kp_tilt = 3000.0;
const float Ki_tilt = 0.0;
const float Kd_tilt = 0.08;

// PID constants for speed control (outer loop)
const float Kp_speed = -3.0;
const float Ki_speed = -1.0;
const float Kd_speed = -0.05;

// PID constants for angular speed control
const float Kp_ang = 1.0;
const float Ki_ang = 0.0;
const float Kd_ang = 0.0;

// Constants for current measurement
const float SHUNT_5V = 0.01;     // Ω
const float GAIN_5V = 105.0;     // INA122 gain
const float SHUNT_MOTOR = 0.1;   // Ω
const float GAIN_MOTOR = 20.0;   // TLV gain

// Constants for voltage measurement
const float VBAT_DIVIDER_RATIO = 5.0; // Example: 100k/(100k+400k)
const float V5_DIVIDER_RATIO = 2.0;   // Example: 10k/10k

const float ADC_REF_VOLTAGE = 3.28;
const int ADC_RESOLUTION = 4095;

//Global objects
ESP32Timer ITimer(3);
Adafruit_MPU6050 mpu;
step step1(STEPPER_INTERVAL_US, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
step step2(STEPPER_INTERVAL_US, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

// These variables should be stored outside void loop because we use their previous values
float desired_speed = 0.0;
float prev_desired_speed = 0.0;
float desired_tilt = 0.0;
float desired_angular_velocity = 0.0;
float tilt_angle = 0.0;
float speed_integral = 0.0, tilt_integral = 0.0, ang_integral = 0.0;
float prev_speed_error = 0.0, prev_tilt_error = 0.0, prev_ang_error = 0.0;
bool first_run = true;

// Odom initalization
Pose2D robot_pose;
int prev_left_steps = 0;
int prev_right_steps = 0;

// Velocity commands initialization
float V_Control_Input = 0.0;
float omega_Control_Input = 0.0;
enum ControlMode { MANUAL, AUTONOMOUS };
//ControlMode currentMode = MANUAL;
ControlMode currentMode = AUTONOMOUS;

// Interrupt Service Routine for motor update
// Note: ESP32 doesn't support floating point calculations in an ISR
bool TimerHandler(void * timerNo) {
  static bool toggle = false;
  step1.runStepper();
  step2.runStepper();
  digitalWrite(TOGGLE_PIN, toggle);
  toggle = !toggle;
  return true;
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[%u] Connected\n", num);
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected\n", num);
      break;
    case WStype_TEXT: {
  String msg = String((char*)payload);

  //  mde switch command
  if (msg == "MODE:MANUAL") {
    currentMode = MANUAL;
    Serial.println("Switched to MANUAL mode.");
    return;
  } else if (msg == "MODE:AUTONOMOUS") {
    currentMode = AUTONOMOUS;
    Serial.println("Switched to AUTONOMOUS mode.");
    return;
  }

  // MANUAL mode
  if (currentMode == MANUAL) {
    StaticJsonDocument<100> doc;
    DeserializationError err = deserializeJson(doc, payload);
    if (!err && doc.containsKey("x") && doc.containsKey("y")) {
      float x = doc["x"];
      float y = doc["y"];
      x = constrain(x, -1.0, 1.0);
      y = constrain(y, -1.0, 1.0);
      float V = y * 0.2;
      float omega = x * 0.2;
      if (V < 0) omega = -omega;
      V_Control_Input = V;
      omega_Control_Input = omega;
      Serial.printf("Received joystick: V = %.3f, ω = %.3f\n", V, omega);
    } else {
      Serial.println("Invalid JSON or missing joystick fields.");
    }
  }
  break;
}

    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);

  analogSetAttenuation(ADC_11db);

  pinMode(TOGGLE_PIN, OUTPUT);

  // Try to initialize Accelerometer/Gyroscope
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  //Attach motor update ISR to timer to run every STEPPER_INTERVAL_US μs
  if (!ITimer.attachInterruptInterval(STEPPER_INTERVAL_US, TimerHandler)) {
    Serial.println("Failed to start stepper interrupt");
    while (1) delay(10);
  }

  // Enable the stepper motor drivers
  pinMode(STEPPER_EN_PIN, OUTPUT);
  digitalWrite(STEPPER_EN_PIN, LOW);

  // Set up ADC and SPI
  pinMode(ADC_CS_PIN, OUTPUT);
  digitalWrite(ADC_CS_PIN, HIGH);
  SPI.begin(ADC_SCK_PIN, ADC_MISO_PIN, ADC_MOSI_PIN, ADC_CS_PIN);

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nConnected! IP: " + WiFi.localIP().toString());

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {

  webSocket.loop();
    static unsigned long printTimer = 0, loopTimerOuter = 0, loopTimerInner = 0, odomTimer = 0, powerTimer = 0;
  static unsigned long prev_time_o = millis(), prev_time_i = millis();
  
  // Read cmd_vel from Raspberry Pi in autonomous mode
  if (currentMode == AUTONOMOUS && Serial.available()) {
    String rosCmd = Serial.readStringUntil('\n');
    float v = 0.0, w = 0.0;
    Serial.println("autonomous");
    if (sscanf(rosCmd.c_str(), "%f,%f", &v, &w) == 2) {
      V_Control_Input = constrain(v, -0.2, 0.2);
      omega_Control_Input = constrain(w, -0.2, 0.2);
      // Serial.printf("ROS command: V = %.3f, ω = %.3f\n", v, w);
    }
  }
  
  // Print power parameters to serial and send to app via WebSocket
  if (millis() > powerTimer){
    powerTimer += POWER_INTERVAL;
    // --- Read ADC values ---
    int raw_5V_current = analogRead(ADC_5V_CURRENT);
    int raw_motor = analogRead(ADC_MOTOR);
    int raw_vbat = analogRead(ADC_VBAT);
    int raw_5V_voltage = analogRead(ADC_5V_VOLTAGE);

    // --- Convert to voltages ---
    float v_5V_current = (raw_5V_current / (float)ADC_RESOLUTION) * ADC_REF_VOLTAGE ;
    float v_motor = (raw_motor / (float)ADC_RESOLUTION) * ADC_REF_VOLTAGE;
    float v_vbat = (raw_vbat / (float)ADC_RESOLUTION) * ADC_REF_VOLTAGE * VBAT_DIVIDER_RATIO;
    float v_5V = (raw_5V_voltage / (float)ADC_RESOLUTION) * ADC_REF_VOLTAGE * V5_DIVIDER_RATIO;

    // --- Calculate currents ---
    float current_5V = (v_5V_current / (GAIN_5V * SHUNT_5V));
    float current_motor = (v_motor / (GAIN_MOTOR * SHUNT_MOTOR));

    StaticJsonDocument<200> doc;
    doc["vbat"] = v_vbat;
    doc["v5"] = v_5V;
    doc["current_5v"] = current_5V ;     
    doc["current_motor"] = current_motor ; 

    String output;
    serializeJson(doc, output);
    webSocket.broadcastTXT(output);  // Sends to all connected clients

    // --- Print everything ---
    /*
    Serial.print("VBat Voltage: ");
    Serial.print(v_vbat, 2);
    Serial.println(" V");

    Serial.print("5V Rail Voltage: ");
    Serial.print(v_5V, 2);
    Serial.println(" V");

    Serial.print("5V Supply Current: ");
    Serial.print(current_5V * 1000);
    Serial.println(" mA");

    Serial.print("Motor Current: ");
    Serial.print(current_motor * 1000);
    Serial.println(" mA");

    Serial.println("-----------------------------");
    */
  }

  // Robot speeds converted to motor speeds
  desired_speed = V_Control_Input / WHEEL_RADIUS;
  desired_angular_velocity = omega_Control_Input;

  // If desired speed changes sign, reset the integral term of the outer loop controller
  if (desired_speed * prev_desired_speed < 0){
    speed_integral = 0.0;
  }

  // Run the outer control loop every LOOP_INTERVAL_OUTER ms
  if (millis() > loopTimerOuter) {
    unsigned long now_o = millis();
    float dt_o = (now_o - prev_time_o) / 1000.0;
    if (dt_o <= 0.001) dt_o = 0.01;
    prev_time_o = now_o;
    loopTimerOuter += LOOP_INTERVAL_OUTER;

    float robot_speed = (step1.getSpeedRad() + step2.getSpeedRad()) / 2.0;

    float speed_error = desired_speed - robot_speed;
    speed_integral += speed_error * dt_o;
    desired_tilt = 0.001 * Kp_speed * speed_error + 0.001 * Ki_speed * speed_integral + 0.001 * Kd_speed * (speed_error - prev_speed_error) / dt_o;
    prev_speed_error = speed_error;

    // If the output (desired_tilt) is saturated, stop integrating.
    if (desired_tilt >= MAX_TILT_ANGLE_RAD && speed_error > 0)
      speed_integral -= speed_error * dt_o;
    else if (desired_tilt <= -MAX_TILT_ANGLE_RAD && speed_error < 0)
      speed_integral -= speed_error * dt_o;

    desired_tilt = constrain(desired_tilt, -MAX_TILT_ANGLE_RAD, MAX_TILT_ANGLE_RAD);
  }

  // Run the inner control loop every LOOP_INTERVAL_INNER ms
  if (millis() > loopTimerInner) {
    unsigned long now_i = millis();
    float dt_i = (now_i - prev_time_i) / 1000.0;
    if (dt_i <= 0.001) dt_i = 0.1;
    prev_time_i = now_i;
    loopTimerInner += LOOP_INTERVAL_INNER;

    // Fetch data from MPU6050
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Calculate tilt using accelerometer
    float acc_angle = asin(constrain(a.acceleration.z / 9.67, -1.0, 1.0));

    if (first_run) {
      tilt_angle = acc_angle;
      first_run = false;
    } else {
      //Serial.print("Desired Tilt: "); Serial.println(g.gyro.y, 4);
      g.gyro.y += 0.002796;
      tilt_angle = alpha * (tilt_angle + g.gyro.y * dt_i) + (1.0 - alpha) * (acc_angle + 0.03);
    }

    // After ca;cilating tilt_angle, check for NaN
    if (isnan(tilt_angle)) {
      Serial.println("NaN detected! Resetting...");
      tilt_angle = 0.0;
      first_run = true;
    }

    float tilt_error = desired_tilt - tilt_angle - 0.015;
    tilt_integral += tilt_error * dt_i;
    float tilt_accel = Kp_tilt * tilt_error + Ki_tilt * tilt_integral + Kd_tilt * (tilt_error - prev_tilt_error) / dt_i;
    prev_tilt_error = tilt_error;

    float ang_speed = step1.getSpeedRad() - step2.getSpeedRad();

    float ang_error = desired_angular_velocity - ang_speed;
    ang_integral += ang_error * dt_i;
    float tilt_accel_ang = Kp_ang * ang_error + Ki_ang * ang_integral + Kd_ang * (ang_error - prev_ang_error) / dt_i;
    prev_ang_error = ang_error;

    float step1_accel = tilt_accel + tilt_accel_ang;
    float step2_accel = tilt_accel - tilt_accel_ang;

    step1_accel = constrain(step1_accel, -40.0, 40.0);
    step2_accel = constrain(step2_accel, -40.0, 40.0);

    step1.setAccelerationRad(abs(step1_accel));
    step2.setAccelerationRad(abs(step2_accel));

    step1.setTargetSpeedRad(step1_accel >= 0 ? 20 : -20);
    step2.setTargetSpeedRad(step2_accel >= 0 ? 20 : -20);
  }

  // Calculate odometry and send to Raspberry Pi through serial
  if (millis() > odomTimer) {
    odomTimer += ODOM_INTERVAL;
    int curr_left_steps = step1.getPosition();
    int curr_right_steps = step2.getPosition();
    int dL = curr_left_steps - prev_left_steps;
    int dR = curr_right_steps - prev_right_steps;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    //Serial.print("Desired Tilt: "); Serial.println(g.gyro.x, 4);
    float dTheta = (g.gyro.x + 0.058798) * ODOM_INTERVAL / 1000;
    robot_pose = updateOdometry(robot_pose, dL, dR, WHEEL_RADIUS, WHEEL_BASE, step1.STEP_ANGLE, dTheta);
    prev_left_steps = curr_left_steps;
    prev_right_steps = curr_right_steps;
    char dataBuffer[64];
    snprintf(dataBuffer, sizeof(dataBuffer), "ODOM:%.4f,%.4f,%.4f\n", robot_pose.x, robot_pose.y, robot_pose.theta);
    Serial.print(dataBuffer);
  }
  
  // Print updates every PRINT_INTERVAL ms
  // Format:
  // OUTER LOOP: time, actual linear speed, desired linear speed, desired tilt
  // INNER LOOP: time, actual angular speed, desired angular speed, actual tilt angle
  /*
  if (millis() > printTimer) {
    printTimer += PRINT_INTERVAL;
    Serial.print("OUTER LOOP: ");
    Serial.print(millis()); Serial.print(",");
    Serial.print((step1.getSpeedRad() + step2.getSpeedRad()) / 2, 4); Serial.print(",");
    Serial.print(desired_speed, 4); Serial.print(",");
    Serial.println(desired_tilt, 4);

    Serial.print("INNER LOOP: ");
    Serial.print(millis()); Serial.print(",");
    Serial.print(step1.getSpeedRad() - step2.getSpeedRad()); Serial.print(",");
    Serial.print(desired_angular_velocity, 4); Serial.print(",");
    Serial.println(tilt_angle + 0.015, 4);
  }
    */
  
}