#include <Arduino.h>
#include <M5AtomS3.h>
#include "CyberGear.h"

// CyberGearオブジェクト (Motor ID: 0x7E, Master ID: 0x00)
CyberGear motor(0x7E, 0x00);

// CA-IS3050G CAN通信ピン定義（M5ATOM S3用）
#define CAN_TX_PIN 2
#define CAN_RX_PIN 1

// 制御用変数
float target_position = 0.0;
float target_velocity = 0.0;
float kp = 30.0;  // 位置ゲイン
float kd = 1.0;   // 速度ゲイン
float target_torque = 5.0;

bool control_enabled = false; // 制御ループ ON/OFF フラグ（起動時はOFF）

unsigned long last_control_time = 0;
const unsigned long control_interval = 10; // 10ms間隔で制御

void setup() {
    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 1500) { delay(10); }  // PC接続待ち (最大1.5s)

    // M5 初期化（必要なら構成を調整）
    auto cfg = M5.config();
    // cfg.serial_baudrate = 115200;  // (M5Unifiedの版によって存在) 明示するなら
    AtomS3.begin(cfg, false);
    
    Serial.println("=== CyberGear CA-IS3050G制御システム起動 ===");
    
    // CA-IS3050G CAN通信の初期化 (ESP32内蔵CAN使用)
    if (!motor.init(CAN_TX_PIN, CAN_RX_PIN, 1000000)) {
        Serial.println("CA-IS3050G CAN初期化失敗");
        while (1) {
            AtomS3.update();
            AtomS3.dis.drawpix(0xff0000); // 赤色でエラー表示
            delay(500);
        }
    }
    
    
    // モーターの制限値設定
    motor.setTorqueLimit(5.0);   // トルク制限 5Nm
    motor.setSpeedLimit(10.0);   // 速度制限 10rad/s
    motor.setCurrentLimit(10.0); // 電流制限 10A
    
    // モーターのゼロ位置設定
    Serial.println("ゼロ位置設定中...");
    motor.setZeroPosition();
    delay(1000);
    
    // モーター有効化
    Serial.println("モーター有効化中...");
    motor.enable();
    delay(500);
    
    Serial.println("セットアップ完了");
    Serial.println("ボタンを押して制御開始");
    Serial.println("シリアルコマンド: 'help' でコマンド一覧表示");
    Serial.println("'scan' でCyberGearの自動検出");
}

void loop() {
    AtomS3.update();
    
    // ボタンが押された時の処理
    if (AtomS3.BtnA.wasPressed()) {
        Serial.println("ボタンが押されました");
        
        // 目標位置を変更（90度ずつ回転）
        target_position += PI / 2.0;
        if (target_position > 2 * PI) {
            target_position = 0.0;
        }
        
        Serial.print("目標位置: ");
        Serial.print(target_position * 180.0 / PI);
        Serial.println(" 度");
        
        // LED色を変更（制御中を示す）
        AtomS3.dis.drawpix(0xffff00); // 黄色
    }
    
    // 定期的な制御実行
    if (control_enabled && millis() - last_control_time >= control_interval) {
        last_control_time = millis();
        
        // モーション制御コマンド送信
        motor.setMotionControl(target_position, target_velocity, kp, kd, target_torque);
        
        // モーターステータス取得と表示
        CyberGear::MotorStatus status;
        if (motor.getMotorStatus(status)) {
            // 100回に1回だけ詳細情報を表示（スパム防止）
            static int counter = 0;
            if (counter++ >= 100) {
                counter = 0;
                Serial.println("--- モーター状態 ---");
                Serial.print("位置: ");
                Serial.print(status.position * 180.0 / PI);
                Serial.println(" 度");
                Serial.print("速度: ");
                Serial.print(status.velocity);
                Serial.println(" rad/s");
                Serial.print("トルク: ");
                Serial.print(status.torque);
                Serial.println(" Nm");
                Serial.print("温度: ");
                Serial.print(status.temperature);
                Serial.println(" ℃");
                Serial.println("-------------------");
            }
            
            // 目標位置に近づいたらLEDを緑に
            float position_error = abs(target_position - status.position);
            if (position_error < 0.1) { // 約5.7度以内
                AtomS3.dis.drawpix(0x00ff00); // 緑色
            }
        }
    }
    
    // シリアルコマンド処理
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "stop") {
            Serial.println("モーター停止+制御OFF");
            control_enabled = false;
            motor.disable();
            AtomS3.dis.drawpix(0xff0000); // 赤色
        }
        else if (command == "start") {
            Serial.println("モーター開始+制御ON");
            motor.enable();
            control_enabled = true;
            AtomS3.dis.drawpix(0x0000ff); // 青色
        }
        else if (command == "zero") {
            Serial.println("ゼロ位置設定");
            motor.setZeroPosition();
            target_position = 0.0;
        }
        else if (command.startsWith("pos ")) {
            float pos = command.substring(4).toFloat();
            target_position = pos * PI / 180.0; // 度からラジアンに変換
            Serial.print("目標位置設定: ");
            Serial.print(pos);
            Serial.println(" 度");
        }
        else if (command.startsWith("vel ")) {
            target_velocity = command.substring(4).toFloat();
            Serial.print("目標速度設定: ");
            Serial.print(target_velocity);
            Serial.println(" rad/s");
        }
        else if (command.startsWith("kp ")) {
            kp = command.substring(3).toFloat();
            Serial.print("位置ゲイン設定: ");
            Serial.println(kp);
        }
        else if (command.startsWith("kd ")) {
            kd = command.substring(3).toFloat();
            Serial.print("速度ゲイン設定: ");
            Serial.println(kd);
        }
        else if (command == "scan") {
            Serial.println("CyberGearスキャンを開始します...");
            AtomS3.dis.drawpix(0xffffff); // 白色でスキャン中
            motor.scanForMotors(0x01, 0x7F, 100);
            AtomS3.dis.drawpix(0x0000ff); // 青色で待機状態に戻る
        }
        else if (command.startsWith("scan ")) {
            // 範囲指定スキャン: scan 0x10 0x20
            int space_pos = command.indexOf(' ', 5);
            if (space_pos > 0) {
                uint8_t start_id = (uint8_t)strtol(command.substring(5, space_pos).c_str(), NULL, 0);
                uint8_t end_id = (uint8_t)strtol(command.substring(space_pos + 1).c_str(), NULL, 0);
                Serial.printf("範囲指定スキャン: 0x%02X - 0x%02X\n", start_id, end_id);
                AtomS3.dis.drawpix(0xffffff); // 白色でスキャン中
                motor.scanForMotors(start_id, end_id, 100);
                AtomS3.dis.drawpix(0x0000ff); // 青色で待機状態に戻る
            }
        }
        else if (command.startsWith("ping ")) {
            uint8_t test_id = (uint8_t)strtol(command.substring(5).c_str(), NULL, 0);
            Serial.printf("ID 0x%02X をPingテスト中...\n", test_id);
            if (CyberGear::pingMotor(test_id, 0x00, 200)) {
                Serial.println("応答あり ✓");
            } else {
                Serial.println("応答なし");
            }
        }
        else if (command == "control on") {
            control_enabled = true;
            Serial.println("制御ループ: ON");
        }
        else if (command == "control off") {
            control_enabled = false;
            Serial.println("制御ループ: OFF");
        }
        else if (command == "help") {
            Serial.println("=== コマンド一覧 ===");
            Serial.println("stop           - モーター停止+制御OFF");
            Serial.println("start          - モーター開始+制御ON");
            Serial.println("control on     - 制御ループON");
            Serial.println("control off    - 制御ループOFF");
            Serial.println("zero           - ゼロ位置設定");
            Serial.println("pos <deg>      - 目標位置設定（度）");
            Serial.println("vel <rad/s>    - 目標速度設定");
            Serial.println("kp <value>     - 位置ゲイン設定");
            Serial.println("kd <value>     - 速度ゲイン設定");
            Serial.println("scan           - 全CyberGearスキャン");
            Serial.println("scan <s> <e>   - 範囲指定スキャン (例: scan 0x01 0x7F)");
            Serial.println("ping <id>      - 特定IDをPingテスト (例: ping 0x7E)");
            Serial.println("help           - このヘルプ表示");
            Serial.println("==================");
        }
    }
    
    delay(1);
}
