#include <SPI.h>
#include "mcp2515.h"
//#include <IBusBM.h>
//IBusBM IBus;    // IBus object
unsigned long control_Time;
unsigned long time_start_f_cmd_set = 0;
struct can_frame canMsg;
float error, integral , derivative ;
unsigned long dt = 12;
MCP2515 mcp2515(2);
//int k = 0;
int dir = 1;
int16_t speed_measure = 0;
int16_t speed_target = 0;
int16_t angle_measure = 0;
int16_t angle_target = 0;
int16_t I_mA = 0;
//int ibus_val[6];
int k = 0;
float f = 0;
float f_cmd = 0;
String command;
char flag = 's';
void setup() {
  setup_fun();
}

void loop() {
  k = k + 1;
  //  ibus_val[4] = IBus.readChannel(4); // get latest value for servo channel n+1
  delay(5);
  dt = millis() - control_Time;
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    update_R_measure();
  }
  update_f();
  control_R_motor();



  /*-----------------------------------*/
  if (Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    handleSerialCommand(command);
  }
  if (k % 200 == 0) {
    print_run_msg();
    k = 0;
  }
}

/*-----------------------------------*/
void setup_fun(void) {
  canMsg.can_id  = 0x200;
  canMsg.can_dlc = 8;
  canMsg.data[0] = 0x02;
  canMsg.data[1] = 0x00;
  canMsg.data[2] = 0x00;
  canMsg.data[3] = 0x00;
  canMsg.data[4] = 0x00;
  canMsg.data[5] = 0x00;
  canMsg.data[6] = 0x00;
  canMsg.data[7] = 0x00;
  Serial.begin(9600);
  Serial.setTimeout(5);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(20);
  Serial.println("");
  Serial.println("------ Ready to run-------");
  Serial.println("f100 to set f1.0 hz");
  Serial.println("s to restart");
  Serial.println("command end with newline");
  while (flag != 'f') {
    delay(5);
    if (Serial.available() > 0) {
      command = Serial.readStringUntil('\n');
      Serial.print("command:");
      Serial.println(command);
      if (command.startsWith("f")) {
        flag = 'f';
      }
    }
  }
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  //  IBus.begin(Serial);
  Serial.println("running...");
  time_start_f_cmd_set = millis();
  control_Time = millis();
}
int16_t pid_speed_control(int speed_target, int speed_measure) {
  error =  speed_target - speed_measure;
  integral += 0.1 * error ;                 /*积分项：误差项的累计*/
  if (integral > 5000) {
    integral = 5000;
  }
  if (integral < -5000) {
    integral = -5000;
  }
  I_mA = int16_t(2.2L * error) + int16_t(integral); /*三项分别乘以PID系数即为输出*/
  return I_mA;
}
int16_t pid_angle_control(int angle_target, int angle_measure) {
  error =  angle_target - angle_measure;
  integral += 3 * error ;                 /*积分项：误差项的累计*/
  if (integral > 5000) {
    integral = 5000;
  }
  if (integral < -5000) {
    integral = -5000;
  }
  I_mA = int16_t(50.1L * error) + int16_t(integral); /*三项分别乘以PID系数即为输出*/
  return I_mA;
}
int16_t check_I(int16_t I_mA) {
  if (I_mA > 10000) {
    I_mA = 10000L;
    //    Serial.println("big I");
  }
  if (I_mA < -10000) {
    I_mA = -10000L;
    //    Serial.println("-big I");
  }
  return I_mA;
}
void print_run_msg(void) {
  Serial.println("---------------------");
  Serial.println("f: " + String(f));
  Serial.println("angle_measure= " + String(angle_measure / 8191.0 * 360.0));
  Serial.println("speed_measure= " + String(speed_measure / 36));
  Serial.println("speed_target= " + String(speed_target / 36));
  //  Serial.println("ibus_val[1]:" + String(ibus_val[1]));
  Serial.println("I_mA:" + String(I_mA));
  Serial.println("dt:" + String(dt));
}

void update_R_measure(void) {
  if (canMsg.can_id == 0x201) {
    speed_measure = canMsg.data[2];
    speed_measure <<= 8;
    speed_measure |= canMsg.data[3];
    angle_measure = canMsg.data[0];
    angle_measure <<= 8;
    angle_measure |= canMsg.data[1];
  }
}
void update_f(void) {
  if (abs(f - f_cmd) > 0.0001) {
    f = (1 - exp((time_start_f_cmd_set / 50000.0 - millis() / 50000.0))) * (f_cmd - f) + f;
  } else {
    f = f_cmd;
  }
}
void control_R_motor(void) {
  control_Time = millis();
  speed_target = dir * f * 60 * 36 ; // 36为减速比，250最大转速250rpm
  I_mA = pid_speed_control(speed_target, speed_measure);
  //  I_mA = pid_angle_control(angle_target, angle_measure);
  I_mA = check_I(I_mA);
  if (f - 0 < 0.001) {
    I_mA = 0;
  }
  canMsg.data[1] = (I_mA >> (8 * 0)) & 0xff;
  canMsg.data[0] = (I_mA >> (8 * 1)) & 0xff;
  mcp2515.sendMessage(&canMsg);
}
// 处理串口指令的函数
void handleSerialCommand(String command) {
  // 假设命令格式为 "f 150"
  Serial.print("command:");
  Serial.println(command);
  if (command.startsWith("f")) {
    flag = 'f';
    f_cmd = (command.substring(1).toInt()) / 100.0;
    time_start_f_cmd_set = millis();
    Serial.print("New f : ");
    Serial.println(f_cmd);
  }
  else if (command.startsWith("s")) {
    flag = 's';
    // 直接调用汇编指令进行软件复位
    asm volatile ("  jmp 0");
  }
}
