#include <SPI.h>
#include "mcp2515.h"
//#include <IBusBM.h>
//IBusBM IBus;    // IBus object
unsigned long time_start;
unsigned long myTime;
unsigned long time_start_f = 0;
struct can_frame canMsg1;
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
char A[8];  //定义一个无符号数组A
int val;
int j, i;
int k = 0;
float f = 0;
float f_tra = 0;
char flag = '1';
String str = "";
void setup() {
  canMsg1.can_id  = 0x200;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 0x02;
  canMsg1.data[1] = 0x00;
  canMsg1.data[2] = 0x00;
  canMsg1.data[3] = 0x00;
  canMsg1.data[4] = 0x00;
  canMsg1.data[5] = 0x00;
  canMsg1.data[6] = 0x00;
  canMsg1.data[7] = 0x00;
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  while (flag != 'f') {
    flag = Serial.read();
  }
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
  //  IBus.begin(Serial);
  Serial.println("Ready to run");
  time_start = millis();
  time_start_f = millis();
}

void loop() {
  k = k + 1;
  //  ibus_val[4] = IBus.readChannel(4); // get latest value for servo channel n+1
  //  ibus_val[5] = IBus.readChannel(5); // get latest value for servo channel n+1
  switch (flag) {
    case 'f':
      f_tra = float(val) / 100.0;
      time_start_f = millis();
      Serial.println("f_tra:" + String(f_tra));
      flag = 'l';
      break;
    case 'a':
      angle_target = int16_t(val / 360.0L * 8191.0L );
      time_start_f = millis();
      Serial.println("angle_target:" + String(val));
      flag = 'l';
      break;
    default:
      break;
  }
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    myTime = millis() - time_start;
    if (canMsg.can_id == 0x201) {
      speed_measure = canMsg.data[2];
      speed_measure <<= 8;
      speed_measure |= canMsg.data[3];
      angle_measure = canMsg.data[0];
      angle_measure <<= 8;
      angle_measure |= canMsg.data[1];
    }
  }
  if (abs(f - f_tra) > 0.0001) {
    f = (1 - exp((time_start_f / 50000.0 - millis() / 50000.0))) * (f_tra - f) + f;
  } else {
    f = f_tra;
  }


  /*-----------------------------------*/
  speed_target = dir * f * 60 * 36 ; // 36为减速比，250最大转速250rpm
  I_mA = pid_speed_control(speed_target, speed_measure);
  //  I_mA = pid_angle_control(angle_target, angle_measure);
  I_mA = check_I(I_mA);
  if (f - 0 < 0.001) {
    I_mA = 0;
  }
  canMsg1.data[1] = (I_mA >> (8 * 0)) & 0xff;
  canMsg1.data[0] = (I_mA >> (8 * 1)) & 0xff;
  mcp2515.sendMessage(&canMsg1);
  if (k % 200 == 0) {
    j = Serial.available();  // 读取串口寄存器中的信息的帧数
    if (j != 0) { // 如果串口寄存器中有数据，那么依次读取这些数据，并存放到数组A中
      flag = Serial.read();
      for (i = 0; i < 8; i++) {
        A[i] = ' ';
      }
      for (i = 0; i < j - 1; i++) {
        A[i] = Serial.read();
      }
      Serial.println(A);
      Serial.println(j);
      val = strtol(A, NULL, 10); // 将A中的字符转换成十进制数
      j = 0;
    }
    print_run_msg();
    Serial.println("f:" + String(f));
  }
  delay(5);
  dt = millis() - time_start - myTime;
}



/*-----------------------------------*/
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
  Serial.println("k:" + String(k));
  Serial.println(str);
  Serial.println("angle_measure= " + String(angle_measure / 8191.0 * 360.0));
  Serial.println("speed_measure= " + String(speed_measure / 36));
  Serial.println("speed_target= " + String(speed_target / 36));
  //  Serial.println("ibus_val[1]:" + String(ibus_val[1]));
  Serial.println("I_mA:" + String(I_mA));
  Serial.println("dt:" + String(dt));
}
