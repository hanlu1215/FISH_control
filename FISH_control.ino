#include "my_fun.h"

MCP2515 mcp2515(2); //R 电机CAN通讯模块的连接引脚
const int IN1_PIN = 5; // 控制引脚 IN1
const int IN2_PIN = 6; // 控制引脚 IN2
const int Servo_PIN = 9; // 舵机连接到引脚 9


Servo myServo;         // 创建一个舵机对象
int servoPosition = 90; // 舵机的位置

void setup() {
  setup_fun();
  myServo.attach(Servo_PIN);   // 将舵机连接到引脚 9
  
  pinMode(IN1_PIN, OUTPUT); // 设置 IN1 引脚为输出
  pinMode(IN2_PIN, OUTPUT); // 设置 IN2 引脚为输出
  // 初始设置抽水机为待机状态
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
}

void loop(void) {
  k = k + 1;
  delay(5);
  dt = millis() - control_Time;
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) update_R_measure();
  update_f();
  control_R_motor();
 // myServo.write(servoPosition);     // 设置舵机位置
  
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    handleSerialCommand(command);
  }
  if (k % 200 == 0) {
    print_run_msg();
    k = 0;
  }
}
