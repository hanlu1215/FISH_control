#include "my_fun.h"

unsigned long control_Time;
unsigned long time_start_f_cmd_set = 0;
struct can_frame canMsg, canMsg_read;
float error, integral, derivative;
unsigned long dt = 12;

int dir = 1;
int16_t speed_measure = 0;
int16_t speed_target = 0;
int16_t angle_measure = 0;
int16_t angle_target = 0;
int16_t I_mA = 0;
int k = 0;
float f = 0;
float f_cmd = 0;
String command;
char flag = 's';


void setup_fun(void) {
  canMsg.can_id = 0x200;
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
  while (flag != 'f') {
    delay(5);
    if (Serial.available() > 0) {
      command = Serial.readStringUntil('\n');
      Serial.println("");
      Serial.println("------ Ready to run-------");
      Serial.println("f100 to set f1.0 hz");
      Serial.println("s to restart");
      Serial.println("command end with newline");
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
  Serial.println("running...");
  time_start_f_cmd_set = millis();
  control_Time = millis();
}



int16_t pid_speed_control(int speed_target, int speed_measure) {
  error = speed_target - speed_measure;
  integral += 0.1 * error; /*积分项：误差项的累计*/
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
  error = angle_target - angle_measure;
  integral += 3 * error; /*积分项：误差项的累计*/
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
  }
  if (I_mA < -10000) {
    I_mA = -10000L;
  }
  return I_mA;
}

void print_run_msg(void) {
  Serial.println("---------------------");
  Serial.println("f: " + String(f));
  Serial.println("angle_measure= " + String(angle_measure / 8191.0 * 360.0));
  Serial.println("speed_measure= " + String(speed_measure / 36));
  Serial.println("speed_target= " + String(speed_target / 36));
  Serial.println("I_mA:" + String(I_mA));
  Serial.println("dt:" + String(dt));
}

void update_R_measure(void) {
  if (canMsg_read.can_id == 0x201) {
    speed_measure = canMsg_read.data[2];
    speed_measure <<= 8;
    speed_measure |= canMsg_read.data[3];
    angle_measure = canMsg_read.data[0];
    angle_measure <<= 8;
    angle_measure |= canMsg_read.data[1];
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
  speed_target = dir * f * 60 * 36; // 36为减速比，250最大转速250rpm
  I_mA = pid_speed_control(speed_target, speed_measure);
  I_mA = check_I(I_mA);
  if (f - 0 < 0.001) {
    I_mA = 0;
  }

  canMsg.data[1] = (I_mA >> (8 * 0)) & 0xff;
  canMsg.data[0] = (I_mA >> (8 * 1)) & 0xff;
  mcp2515.sendMessage(&canMsg);
}
// 根据指令内容执行操作
void processPumpAction(String action) {
  if (action.equals("d")) {
    stopPump(); // 待机
  } else if (action.equals("s")) {
    brakePump(); // 刹车
  } else if (action.startsWith("+") || action.startsWith("-")) {
    int pwmValue = action.substring(1).toInt(); // 提取 PWM 占空比
    if (action.startsWith("+")) {
      setPumpForward(pwmValue); // 正转
    } else if (action.startsWith("-")) {
      setPumpReverse(pwmValue); // 反转
    }
  } else {
    Serial.println("Invalid command. Use d, s, +<pwm>, or -<pwm>.");
  }
}

// 设置抽水机正转
void setPumpForward(int pwmValue) {
  if (pwmValue < 0 || pwmValue > 255) {
    Serial.println("Invalid PWM value. Must be between 0 and 255.");
    return;
  }
  analogWrite(IN1_PIN, pwmValue); // 设置正转 PWM 占空比
  digitalWrite(IN2_PIN, LOW);     // IN2 设为 LOW
  Serial.print("Pump set to forward with PWM: ");
  Serial.println(pwmValue);
}

// 设置抽水机反转
void setPumpReverse(int pwmValue) {
  if (pwmValue < 0 || pwmValue > 255) {
    Serial.println("Invalid PWM value. Must be between 0 and 255.");
    return;
  }
  digitalWrite(IN1_PIN, LOW);     // IN1 设为 LOW
  analogWrite(IN2_PIN, pwmValue); // 设置反转 PWM 占空比
  Serial.print("Pump set to reverse with PWM: ");
  Serial.println(pwmValue);
}

// 设置抽水机刹车
void brakePump() {
  digitalWrite(IN1_PIN, HIGH);    // 设置 IN1 为 HIGH
  digitalWrite(IN2_PIN, HIGH);    // 设置 IN2 为 HIGH
  Serial.println("Pump set to brake.");
}

// 设置抽水机待机
void stopPump() {
  digitalWrite(IN1_PIN, LOW);     // 设置 IN1 为 LOW
  digitalWrite(IN2_PIN, LOW);     // 设置 IN2 为 LOW
  Serial.println("Pump set to standby.");
}


void handleSerialCommand(String command) {
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
    canMsg.data[1] = (0 >> (8 * 0)) & 0xff;
    canMsg.data[0] = (0 >> (8 * 1)) & 0xff;
    mcp2515.sendMessage(&canMsg);
    delay(100);
    asm volatile ("  jmp 0");
  }
  else if (command.startsWith("t")) {
    int pos = command.substring(1).toInt(); // 提取命令中的数字
    if (pos >= 1000 && pos <= 2000) {   // 确保位置在合理范围内
      servoPosition = map(pos, 1000, 2000, 0, 180); // 将 1000-2000 映射到 0-180 度
      myServo.write(servoPosition);     // 设置舵机位置
      Serial.print("Servo moved to: ");
      Serial.println(servoPosition);
    } else {
      Serial.println("Invalid position. Use t1000 to t2000.");
    }
  }
  else if (command.startsWith("c")) {
    String action = command.substring(1); // 提取指令内容
    processPumpAction(action); // 处理指令
  }
  else {
    Serial.println("Invalid command. Use format: tXXXX");
  }
}
