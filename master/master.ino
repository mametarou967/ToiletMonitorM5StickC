#include <M5StickC.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
esp_now_peer_info_t slave;

#define CMD_TOGGLE_SEISOU_STATUS 101
#define CMD_NOTIFY_STATUS 102

#define VL53L0X_REG_IDENTIFICATION_MODEL_ID         0xc0
#define VL53L0X_REG_IDENTIFICATION_REVISION_ID      0xc2
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD   0x50
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD 0x70
#define VL53L0X_REG_SYSRANGE_START                  0x00
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS         0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS             0x14

#define ToF_ADDR 0x29  // the iic address of tof
#define ExistHumanNum 150

byte gbuf[16];
int distanceNum = 0;
int existHumanCount = 0;
int notExistHumanCount = 0;

enum JudgeStatus
{
  ExistHumanStatus,
  NotExistHumanStatus,
  SeisouChuuStatus
};

JudgeStatus judgeStatus = NotExistHumanStatus;

int count = 0;
// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  if(data[0] == CMD_TOGGLE_SEISOU_STATUS)
  {
    // 画面にも描画
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0, 0);
    Serial.println("toggle rcv.");
    if(judgeStatus != SeisouChuuStatus){
      judgeStatus = SeisouChuuStatus;
    }else{
      judgeStatus = NotExistHumanStatus;
    }
  }
}
void setup() {
  M5.begin();

  // put your setup code here, to run once:
  Wire.begin(0, 26, 400000UL);  // join i2c bus (address optional for master)
  Serial.begin(115200);         // start serial for output
  Serial.println("VLX53LOX test started.");
  
  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
  // マルチキャスト用Slave登録
  memset(&slave, 0, sizeof(slave));
  for (int i = 0; i < 6; ++i) {
    slave.peer_addr[i] = (uint8_t)0xff;
  }
  esp_err_t addStatus = esp_now_add_peer(&slave);
  if (addStatus == ESP_OK) {
    // Pair success
    Serial.println("Pair success");
  }
  // ESP-NOWコールバック登録
  esp_now_register_recv_cb(OnDataRecv);
  M5.begin();
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setTextSize(2);
}

void loop() {
  M5.update();
  // ボタンを押したら送信

  distanceNum = measure_distance();

  if(judgeStatus != SeisouChuuStatus){
    update_status(distanceNum);
  }
  
  uint8_t data[2] = {CMD_NOTIFY_STATUS, judgeStatus};
  esp_err_t result = esp_now_send(slave.peer_addr, data, sizeof(data));

  /*
  if ( M5.BtnA.wasPressed() ) {
    uint8_t data[2] = {CMD_NOTIFY_STATUS, judgeStatus};
    esp_err_t result = esp_now_send(slave.peer_addr, data, sizeof(data));
    Serial.print("Send Status: ");
  }
  */
  delay(1000);
}


int measure_distance() {
    write_byte_data_at(VL53L0X_REG_SYSRANGE_START, 0x01);

    read_block_data_at(VL53L0X_REG_RESULT_RANGE_STATUS,
                       12);  // read 12 bytes once

    uint16_t dist =
        makeuint16(gbuf[11], gbuf[10]);  // split distance data to "dist"
    byte DeviceRangeStatusInternal = ((gbuf[0] & 0x78) >> 3);

    Serial.print("distance ");
    Serial.println(dist);

    M5.Lcd.setCursor(20, 80);
    M5.Lcd.fillRect(20, 80, 60, 25, BLACK);
    if(dist < ExistHumanNum){
      M5.Lcd.setTextColor(RED);
    }else{
      M5.Lcd.setTextColor(WHITE);
    }
    M5.Lcd.print(dist);

    return dist;
}

JudgeStatus update_status(int dist)
{
    if(dist < ExistHumanNum)
    {
      existHumanCount++;
      notExistHumanCount=0;
    }else{
      notExistHumanCount++;
      existHumanCount=0;
    }

    if(judgeStatus == ExistHumanStatus && notExistHumanCount >= 3)
    {
      judgeStatus = NotExistHumanStatus;
    }else if(judgeStatus == NotExistHumanStatus && existHumanCount >= 3){
      judgeStatus = ExistHumanStatus;
    }
    
    Serial.print("status ");
    Serial.println(judgeStatus);

    return judgeStatus;
}

uint16_t bswap(byte b[]) {
    // Big Endian unsigned short to little endian unsigned short
    uint16_t val = ((b[0] << 8) & b[1]);
    return val;
}

uint16_t makeuint16(int lsb, int msb) {
    return ((msb & 0xFF) << 8) | (lsb & 0xFF);
}

uint16_t VL53L0X_decode_vcsel_period(short vcsel_period_reg) {
    // Converts the encoded VCSEL period register value into the real
    // period in PLL clocks
    uint16_t vcsel_period_pclks = (vcsel_period_reg + 1) << 1;
    return vcsel_period_pclks;
}

/*
 * IIC Functions
 */
/* function description: write one byte data */
void write_byte_data(byte data) {
    Wire.beginTransmission(ToF_ADDR);
    Wire.write(data);
    Wire.endTransmission();
}

/* function description: write one byte data to specifical register */
void write_byte_data_at(byte reg, byte data) {
    Wire.beginTransmission(ToF_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

/* function description: read two bytes data to specifical register */
void write_word_data_at(byte reg, uint16_t data) {
    byte b0 = (data & 0xFF);
    byte b1 = ((data >> 8) && 0xFF);

    Wire.beginTransmission(ToF_ADDR);
    Wire.write(reg);
    Wire.write(b0);
    Wire.write(b1);
    Wire.endTransmission();
}

/* function description: read one byte data */
byte read_byte_data() {
    Wire.requestFrom(ToF_ADDR, 1);
    while (Wire.available() < 1) delay(1);
    byte b = Wire.read();
    return b;
}

/* function description: read one byte data from specifical register */
byte read_byte_data_at(byte reg) {
    // write_byte_data((byte)0x00);
    write_byte_data(reg);
    Wire.requestFrom(ToF_ADDR, 1);
    while (Wire.available() < 1) delay(1);
    byte b = Wire.read();
    return b;
}

/* function description: read two bytes data from specifical register */
uint16_t read_word_data_at(byte reg) {
    write_byte_data(reg);
    Wire.requestFrom(ToF_ADDR, 2);
    while (Wire.available() < 2) delay(1);
    gbuf[0] = Wire.read();
    gbuf[1] = Wire.read();
    return bswap(gbuf);
}

/* function description: read multiple bytes data from specifical register */
void read_block_data_at(byte reg, int sz) {
    int i = 0;
    write_byte_data(reg);
    Wire.requestFrom(ToF_ADDR, sz);
    for (i = 0; i < sz; i++) {
        while (Wire.available() < 1) delay(1);
        gbuf[i] = Wire.read();
    }
}
