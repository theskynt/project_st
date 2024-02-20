#include <Arduino.h>
#include <math.h>
#include <Adafruit_PWMServoDriver.h>
#define PI 3.14159265358979323846

// สร้าง Object สำหรับโมดูล PCA9685 ให้ชื่อ pwm ค่า Address คือ Default (0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define servoMin  130       // ค่า Pulse ต่ำสุด (องศา 0)
#define servoMax  600    // ค่า Pulse สูงสุด (องศา 180)

#define base 0
#define shoulder 1
#define elbow 2
#define wrist 3
#define gripper 4

bool robotRunning = false;


float phi = 0;  //องศาที่แขนตั้งฉากกับพื้น
float a1 = 10.5; // ความยาวจากlink2 ไป link3
float a2 = 12.5; // ความยาวจากlink3 ไป link4
float a3 = 18.0; // ความยาวจากlink4 ไป มือจับ (gripper)

float px;       // Add missing semicolons
float py;       // Add missing semicolons
float theta1;   // Add missing semicolons
float theta2;   // Add missing semicolons
float theta3;   // Add missing semicolons

float base_angle = 180;
float shoulder_angle = 29.22;
float elbow_angle = 89.67;
float wrist_angle = 119.55;
float gripper_angle = 0;int sensor1 = 6;
int sensor2 = 5;
int sensor3 = 4;
int sensor4 = 3;
int sensor5 = 2;


void setup() {
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);
 
  Serial.begin(9600);
}

void loop() {
  int val1 = digitalRead(sensor1);
  int val2 = digitalRead(sensor2);
  int val3 = digitalRead(sensor3);
  int val4 = digitalRead(sensor4);
  int val5 = digitalRead(sensor5);

  Serial.print ("  ");
  Serial.print (val1);
  Serial.print ("  ");
  Serial.print (val2);
  Serial.print ("  ");
  Serial.print (val3);
  Serial.print ("  ");
  Serial.print (val4);
  Serial.print ("  ");
  Serial.println (val5);
  
  delay(300);
}

String input;

// เซนเซอร์ the ultrasonic sensor
const int pingPin = 13; // Trig
const int inPin = 12;   // Echo

//ค่า PID
float Kp = 10, Ki = 5, Kd = 5;
float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

//เซ็นเซอร์IR
int sensor[5] = { 0, 0, 0, 0, 0 };

//การเดินหุ่น
int track = 0;
int full = 0;
int rail = 2;
int t2 = 0;
int online = 0;
int backhome = 0;
int tree = 0;
int backward_status = 0;

//ความเร็วมอเตอร์
int initial_motor_speed = 80;  //บวกลบแล้วห้ามเกิน255

// ฟังก์ชันสำหรับแปลงค่าองศาเป็น Pulse
int angleToPulse(float angle) {
  return map(angle, 0, 180, servoMin, servoMax);
}

// ฟังก์ชันสำหรับแปลงค่าองศาเป็นเรเดี่ยน
float deg2rad(float degrees) {
  return degrees * (PI / 180.0);
}

// ฟังก์ชันสำหรับแปลงค่าเรเดี่ยนเป็นองศา
float rad2deg(float radians) {
  return radians * (180.0 / PI);
}

// ฟังก์ชันสำหรับการคำนวณ inverse kinematics
void inverse_kinematics(float a1, float a2, float a3, float phi, float px, float py) {
  phi = deg2rad(phi);

  float wx = px - a3 * cos(phi);
  float wy = py - a3 * sin(phi);

  float delta = wx * wx + wy * wy;
  float c2 = (delta - a1 * a1 - a2 * a2) / (2 * a1 * a2);
  float s2 = -sqrt(1 - c2 * c2);
  theta2 = atan2(s2, c2);

  float s1 = ((a1 + a2 * c2) * wy - a2 * s2 * wx) / delta;
  float c1 = ((a1 + a2 * c2) * wx + a2 * s2 * wy) / delta;
  theta1 = atan2(s1, c1);
  theta3 = phi - theta1 - theta2;

  theta1 = 180 - rad2deg(theta1);
  theta2 = 180 + rad2deg(theta2);
  theta3 = 180 + rad2deg(theta3);
}

void controlBase(float targetAngle) {
  if (base_angle < targetAngle) {
    for (float pulse = base_angle; pulse <= targetAngle; pulse++) {
      pwm.setPWM(base, 0, angleToPulse(pulse));
      base_angle = pulse;
      delay(10);
    }
  } else {
    for (float pulse = base_angle; pulse >= targetAngle; pulse--) {
      pwm.setPWM(base, 0, angleToPulse(pulse));
      base_angle = pulse;
      delay(10);
    }
  }
}

void controlShoulder(float targetAngle) {
  if (shoulder_angle < targetAngle) {
    for (float pulse = shoulder_angle; pulse <= targetAngle; pulse++){
      pwm.setPWM(shoulder, 0, angleToPulse(pulse));
      shoulder_angle = pulse;                           
    delay(30);
    }
  } else {
    for (float pulse = shoulder_angle; pulse >= targetAngle; pulse--){
      pwm.setPWM(shoulder, 0, angleToPulse(pulse));
      shoulder_angle = pulse;                           
    delay(30);
    }
  }
}

void controlElbow(float targetAngle) {
  if (elbow_angle < targetAngle) {
    for (float pulse = elbow_angle; pulse <= targetAngle; pulse++){
      pwm.setPWM(elbow, 0, angleToPulse(pulse));
      elbow_angle = pulse;                           
    delay(30);
    }
  } else {
    for (float pulse = elbow_angle; pulse >= targetAngle; pulse--){
      pwm.setPWM(elbow, 0, angleToPulse(pulse));
      elbow_angle = pulse;                           
    delay(30);
    }
  }
}

void controlWrist(float targetAngle) {
  if (wrist_angle < targetAngle) {
    for (float pulse = wrist_angle; pulse <= targetAngle; pulse++) {
      pwm.setPWM(wrist, 0, angleToPulse(pulse));
      wrist_angle = pulse;
      delay(30);
    }
  } else {
    for (float pulse = wrist_angle; pulse >= targetAngle; pulse--) {
      pwm.setPWM(wrist, 0, angleToPulse(pulse));
      wrist_angle = pulse;
      delay(30);
    }
  }
}

void controlGripper(float targetAngle) {
  if (gripper_angle < targetAngle) {
    for (float pulse = gripper_angle; pulse <= targetAngle; pulse++) {
      pwm.setPWM(gripper, 0, angleToPulse(pulse));
      gripper_angle = pulse;
      delay(30);
    }
  } else {
    for (float pulse = gripper_angle; pulse >= targetAngle; pulse--) {
      pwm.setPWM(gripper, 0, angleToPulse(pulse));
      gripper_angle = pulse;
      delay(30);
    }
  }
}


long microsecondsToCentimeters(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

void sendDataToPython(String data) {
  Serial.print(data);
}


long getUltrasonicDistance() {
  long duration, cm;

  pinMode(pingPin, OUTPUT);

  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(inPin, INPUT);
  duration = pulseIn(inPin, HIGH);

  cm = microsecondsToCentimeters(duration);
  
  return cm;
}

void read_sensor_values() {
  //เซ็ทIR
  sensor[0] = digitalRead(6);
  sensor[1] = digitalRead(5);
  sensor[2] = digitalRead(4);
  sensor[3] = digitalRead(3);
  sensor[4] = digitalRead(2);

  //เซนเซอร์ค่าอาร์เรย์  ค่าความผิดพลาด
  if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 0))  //1 1 1 1 0               4
    error = 4;
  if (error == 4) error = 5;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 0))  //1 1 1 0 0               3
    error = 3;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 0) && (sensor[4] == 1))  //1 1 1 0 1               2
    error = 2;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 0) && (sensor[4] == 1))  //1 1 0 0 1               1
    error = 1;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))  //1 1 0 1 1               0
    error = 0;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 0) && (sensor[3] == 1) && (sensor[4] == 1))  //1 0 0 1 1              -1
    error = -1;
  else if ((sensor[0] == 1) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))  //1 0 1 1 1              -2
    error = -2;
  else if ((sensor[0] == 0) && (sensor[1] == 0) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))  //0 0 1 1 1              -3
    error = -3;
  else if ((sensor[0] == 0) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))  //0 1 1 1 1              -4
    error = -4;
  else if ((sensor[0] == 1) && (sensor[1] == 1) && (sensor[2] == 1) && (sensor[3] == 1) && (sensor[4] == 1))  //1 1 1 1 1 -5 หรือ 5 (ขึ้นอยู่กับค่าก่อนหน้านี้)
    if (error == -4) error = -5;
}

void calculate_pid() {
  P = error;
  I = I + previous_I;
  D = error - previous_error;
  PID_value = (Kp * P) + (Ki * I) + (Kd * D);  //float Kp=0,Ki=0,Kd=0;
  previous_I = I;
  previous_error = error;
}

void forward(){
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed + PID_value;
  int right_motor_speed = initial_motor_speed - PID_value;
  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);
  
  analogWrite(10, left_motor_speed);  // Left Motor Speed
  analogWrite(11, right_motor_speed); // Right Motor Speed
  analogWrite(A0, 255);
  analogWrite(A1, 0);
  analogWrite(A2, 255);
  analogWrite(A3, 0);
}


void left(){
  analogWrite(10, 110);  // Left Motor Speed
  analogWrite(11, 110);  // Right Motor Speed
  analogWrite(A0, 0);
  analogWrite(A1, 0);
  analogWrite(A2, 0);
  analogWrite(A3, 0);
  delay(900);
  analogWrite(A0, 255);
  analogWrite(A1, 0);
  analogWrite(A2, 255);
  analogWrite(A3, 0);
  delay(500);
  analogWrite(A0, 255);
  analogWrite(A1, 0);
  analogWrite(A2, 0);
  analogWrite(A3, 255);
  while(digitalRead(2) == 1);
  while(digitalRead(2) == 0);
}

void right(){
  analogWrite(10, 110);  // Left Motor Speed
  analogWrite(11, 110);  // Right Motor Speed
  analogWrite(A0, 0);
  analogWrite(A1, 0);
  analogWrite(A2, 0);
  analogWrite(A3, 0);
  delay(900);
  analogWrite(A0, 255);
  analogWrite(A1, 0);
  analogWrite(A2, 255);
  analogWrite(A3, 0);
  delay(500);
  analogWrite(A0, 0);
  analogWrite(A1, 255);
  analogWrite(A2, 255);
  analogWrite(A3, 0);
  while(digitalRead(6) == 1);
  while(digitalRead(6) == 0);
}

void backward(){
  analogWrite(10, 110);  // Left Motor Speed
  analogWrite(11, 110);  // Right Motor Speed
  analogWrite(A0, 0);
  analogWrite(A1, 0);
  analogWrite(A2, 0);
  analogWrite(A3, 0);
  delay(900);
  analogWrite(10, 110);  // Left Motor Speed
  analogWrite(11, 110);
  analogWrite(A0, 255);
  analogWrite(A1, 0);
  analogWrite(A2, 0);
  analogWrite(A3, 255);
  delay(900);
  analogWrite(A0, 0); //stop switch
  analogWrite(A1, 0);
  analogWrite(A2, 0);
  analogWrite(A3, 0);
  delay(200);
  analogWrite(10, 110);  // Left Motor Speed
  analogWrite(11, 110); 
  analogWrite(A0, 255);
  analogWrite(A1, 0);
  analogWrite(A2, 255);
  analogWrite(A3, 0);
  delay(800);
  analogWrite(A0, 0); //stop switch
  analogWrite(A1, 0);
  analogWrite(A2, 0);
  analogWrite(A3, 0);
  delay(200);
  analogWrite(10, 110);  // Left Motor Speed
  analogWrite(11, 110);
  analogWrite(A0, 0);
  analogWrite(A1, 255);
  analogWrite(A2, 255);
  analogWrite(A3, 0);
  while(digitalRead(6) == 1);
  while(digitalRead(6) == 0);
  analogWrite(10, 110);  // Left Motor Speed
  analogWrite(11, 110);
  analogWrite(A0, 255);
  analogWrite(A1, 0);
  analogWrite(A2, 255);
  analogWrite(A3, 0);
  delay(100);
}

void jump(){
  analogWrite(10, 110);  // Left Motor Speed
  analogWrite(11, 110); 
  analogWrite(A0, 255);
  analogWrite(A1, 0);
  analogWrite(A2, 255);
  analogWrite(A3, 0);
  delay(500);
}

void collect(){
      sendDataToPython("open");
      stop();
      delay(500);
      analogWrite(10, 110);  // Stop Left Motor
      analogWrite(11, 110);  // Stop Right Motor
      analogWrite(A0, 0);  // Set Left Motor Pins to 0
      analogWrite(A1, 255);
      analogWrite(A2, 0);  // Set Right Motor Pins to 0
      analogWrite(A3, 255);
      delay(300);
      stop();
      delay(6000);
      while(Serial.available() > 0){
        input = Serial.readStringUntil('\n');
        py = input.toFloat();
        analogWrite(10, 0);  // Stop Left Motor
        analogWrite(11, 0);  // Stop Right Motor
        analogWrite(A0, 0);  // Set Left Motor Pins to 0
        analogWrite(A1, 0);
        analogWrite(A2, 0);  // Set Right Motor Pins to 0
        analogWrite(A3, 0);

        px = 15;
        
        inverse_kinematics(a1, a2, a3, phi, px, py);

        controlShoulder(theta1);
        controlElbow(theta2);
        controlWrist(theta3);
        delay(3000); 

        long distance = getUltrasonicDistance();
        delay(1000); 

        px = px + (distance - 7);
        inverse_kinematics(a1, a2, a3, phi, px, py);
        controlWrist(theta3);
        controlElbow(theta2);
        controlShoulder(theta1);
        pwm.setPWM(gripper, 0, angleToPulse(55));
        delay(1000); 
        controlShoulder(29.22);
        controlElbow(89.67);
        controlWrist(119.55);
        controlBase(180);
        controlBase(90);
        controlShoulder(90);
        controlElbow(95);
        controlWrist(100);
        pwm.setPWM(gripper, 0, angleToPulse(0));
        delay(1000); 
        controlShoulder(29.22);
        controlElbow(89.67);
        controlWrist(119.55);
        controlBase(180);
        sendDataToPython("success");
        delay(5000);
      }
      sendDataToPython("close");
      jump();
      if (backward_status == 0) {
        tree=tree+1;
      }
      else {
        tree=tree-1;
      }
}
void stop(){
  analogWrite(10, 0);  // Left Motor Speed
  analogWrite(11, 0);  // Right Motor Speed
  analogWrite(A0, 0);
  analogWrite(A1, 0);
  analogWrite(A2, 0);
  analogWrite(A3, 0);
}


void motor_control(){

  if (online == 0){
    if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0){
      if (backhome == 0){
        if (t2 > track){
          jump();
          track = track + 1;
        }
        else{
          left();
          online = 1;
        }
      }
      else{
        if (track > 0){
          jump();
          track = track - 1;
        }
        else if (track == 0){
          stop();
          robotRunning = false;
        }
      }
    }
    else{
      forward();
    }
  }
  else if (online == 1){
    if (sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1){
      backward();
      backward_status=1;
    }
    else if (sensor[0] == 0 && sensor[1] == 0 && sensor[2] == 0 && sensor[3] == 0 && sensor[4] == 0){
      if (backward_status==1){
        if (tree>0){
          collect();
        }
        else{
          backward_status=0;
          if (full == 1){
            right();
            online = 0;
            backhome = 1;
          }
          else if (full == 0){
            online = 0;
            t2 = t2 + 1;
            if (t2 == rail){
              right();
              backhome = 1;

            }
            else{
              track = track + 1;
              left();
            }
          }
        }
      }
      else if(backward_status==0){
        collect();

      }
    }
    else{
      forward();
    }
  }
}


void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);     // กำหนด Frequency = 60 Hz (เซอร์โวส่วนใหญ่จะทำงานที่ 50-60 Hz)
  delay(20);

  pwm.setPWM(base, 0, angleToPulse(base_angle));
  pwm.setPWM(shoulder, 0, angleToPulse(shoulder_angle));
  pwm.setPWM(elbow, 0, angleToPulse(elbow_angle));
  pwm.setPWM(wrist, 0, angleToPulse(wrist_angle));
  pwm.setPWM(gripper, 0, angleToPulse(gripper_angle));

  //เซ็ทมอเตอร์
  pinMode(10, OUTPUT);  //PWM Pin 1 Left  ENA
  pinMode(11, OUTPUT);  //PWM Pin 2 Right ENB
  pinMode(A0, OUTPUT);  //Left Motor Pin 1 in 1
  pinMode(A1, OUTPUT);  //Left Motor Pin 2 in 2
  pinMode(A2, OUTPUT);  //Right Motor Pin 1 in 3
  pinMode(A3, OUTPUT);  //Right Motor Pin 2 in 4

}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "start") {
      // Serial.print("start");
      robotRunning = true;
      jump();
    } 
  }
  if (robotRunning) {
  // Adjust the delay based on your application needs

    read_sensor_values();
    calculate_pid();
    motor_control();

    if (Serial.available() > 0) {
      input = Serial.readStringUntil('\n');
      if (input == "stop"){
        // Serial.print("stop");
        robotRunning = false;
        return;
      }
      // else{
      //   py = input.toFloat();
      //   analogWrite(10, 0);  // Stop Left Motor
      //   analogWrite(11, 0);  // Stop Right Motor
      //   analogWrite(A0, 0);  // Set Left Motor Pins to 0
      //   analogWrite(A1, 0);
      //   analogWrite(A2, 0);  // Set Right Motor Pins to 0
      //   analogWrite(A3, 0);

      //   px = 15;
        
      //   inverse_kinematics(a1, a2, a3, phi, px, py);

      //   controlShoulder(theta1);
      //   controlElbow(theta2);
      //   controlWrist(theta3);
      //   delay(3000); 

      //   long distance = getUltrasonicDistance();
      //   delay(1000); 

      //   px = px + (distance - 7);
      //   inverse_kinematics(a1, a2, a3, phi, px, py);

      //   controlShoulder(theta1);
      //   controlElbow(theta2);
      //   controlWrist(theta3);
      //   controlGripper(70);
      //   delay(1000); 
      //   controlBase(90);
      //   controlShoulder(90);
      //   controlElbow(95);
      //   controlWrist(100);
      //   controlGripper(0);
      //   delay(1000); 
      //   controlShoulder(29.22);
      //   controlElbow(89.67);
      //   controlWrist(119.55);
      //   controlBase(180);
      //   sendDataToPython("success");
      //   delay(1000);
      // }
    }

  }else{
    stop();
  }
}
