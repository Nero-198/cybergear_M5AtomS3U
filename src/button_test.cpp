#include <Arduino.h>
#include <M5AtomS3.h>

void setup() {
    // M5ATOM S3の初期化
    auto cfg = M5.config();
    AtomS3.begin(cfg);
    
    // シリアル通信の初期化
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("=== M5ATOM S3 ボタンテスト ===");
    Serial.println("ボタンを押すとLEDが点灯します");
    
    // 初期状態：LED消灯
    AtomS3.dis.drawpix(0x000000); // 黒（消灯）
}

void loop() {
    AtomS3.update();
    
    // ボタンの状態をチェック
    if (AtomS3.BtnA.isPressed()) {
        // ボタンが押されている間：赤色LED
        AtomS3.dis.drawpix(0xff0000);
        Serial.println("ボタンが押されています");
        delay(50); // デバウンス
    } else {
        // ボタンが離されている間：青色LED
        AtomS3.dis.drawpix(0x0000ff);
        delay(50);
    }
    
    // ボタンが押された瞬間の検出
    if (AtomS3.BtnA.wasPressed()) {
        Serial.println("ボタンが押されました！");
        // 緑色に一瞬光る
        AtomS3.dis.drawpix(0x00ff00);
        delay(200);
    }
    
    // ボタンが離された瞬間の検出
    if (AtomS3.BtnA.wasReleased()) {
        Serial.println("ボタンが離されました！");
        // 黄色に一瞬光る
        AtomS3.dis.drawpix(0xffff00);
        delay(200);
    }
}