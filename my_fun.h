#ifndef MY_FUN_H
#define MY_FUN_H

#include <Arduino.h>
#include <SPI.h>
#include "mcp2515.h"
#include <Servo.h>

// 定义常量、结构体、全局变量等（如果需要）
extern unsigned long control_Time;
extern unsigned long time_start_f_cmd_set;
extern struct can_frame canMsg;
extern float error, integral, derivative;
extern unsigned long dt;
extern MCP2515 mcp2515;
extern int dir;
extern int16_t speed_measure;
extern int16_t speed_target;
extern int16_t angle_measure;
extern int16_t angle_target;
extern int16_t I_mA;
extern int k;
extern float f;
extern float f_cmd;
extern String command;
extern char flag;
extern Servo myServo;         // 创建一个舵机对象
extern int servoPosition; // 舵机的位置
extern const int IN1_PIN; // 控制引脚 IN1
extern const int IN2_PIN; // 控制引脚 IN2


// 函数声明
void setup_fun(void);
int16_t pid_speed_control(int speed_target, int speed_measure);
int16_t pid_angle_control(int angle_target, int angle_measure);
int16_t check_I(int16_t I_mA);
void print_run_msg(void);
void update_R_measure(void);
void update_f(void);
void control_R_motor(void);
// 根据指令内容执行操作
void processPumpAction(String action);
// 设置抽水机正转
void setPumpForward(int pwmValue);
// 设置抽水机反转
void setPumpReverse(int pwmValue);
// 设置抽水机刹车
void brakePump();
// 设置抽水机待机
void stopPump();
void handleSerialCommand(String command);

#endif
