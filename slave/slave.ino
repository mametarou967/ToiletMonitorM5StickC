#include <M5StickC.h>
#include <esp_now.h>
#include <WiFi.h>
esp_now_peer_info_t slave;

#include "exist.h"
#include "noExist.h"
#include "seisou.h"

#define CMD_TOGGLE_SEISOU_STATUS 101
#define CMD_NOTIFY_STATUS 102

enum JudgeStatus
{
  ExistHumanStatus,
  NotExistHumanStatus,
  SeisouChuuStatus,
  NoStatus = 100
};

JudgeStatus prvDispStatus = NoStatus;

// 受信コールバック
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  if(data[0] == CMD_NOTIFY_STATUS)
  {
    // 画面にも描画
    if(prvDispStatus != data[1]){
      M5.Lcd.fillScreen(WHITE);
      M5.Lcd.startWrite();
      
      if(data[1] == ExistHumanStatus){
        M5.Lcd.pushImage(40, 0, existWidth, existHeight, exist);
      }else if(data[1] == NotExistHumanStatus){
        M5.Lcd.pushImage(40, 0, noExistWidth, noExistHeight, noExist);
      }else{
        M5.Lcd.pushImage(40, 0, seisouWidth, seisouHeight, seisou);
      }
      
      M5.Lcd.endWrite();
      prvDispStatus = (JudgeStatus)data[1];
    }
  }
}
void setup() {
  M5.begin();
  M5.Lcd.fillScreen(WHITE);
  M5.Lcd.setTextColor(BLACK);
  M5.Axp.ScreenBreath(10);  
  M5.Lcd.setRotation(3);
  M5.Lcd.setSwapBytes(false);   
  // ESP-NOW初期化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
    M5.Lcd.print("ESPNow Init Success\n");
  } else {
    Serial.println("ESPNow Init Failed");
    M5.Lcd.print("ESPNow Init Failed\n");
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
}
void loop() {
  M5.update();
  // ボタンを押したら送信
  if ( M5.BtnA.wasPressed() ) {
    uint8_t data[2] = {CMD_TOGGLE_SEISOU_STATUS };
    esp_err_t result = esp_now_send(slave.peer_addr, data, sizeof(data));
  }
  delay(1);
}
