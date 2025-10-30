#include <Arduino.h>
#include <M5AtomS3.h>
#include "CyberGear.h"

// CyberGearオブジェクト (Motor ID: 0x7F, Host ID: 0x7E 推奨デフォルト)
CyberGear motor(0x7F, 0x7E);

// CA-IS3050G CAN通信ピン定義（M5ATOM S3用）
#define CAN_TX_PIN 2
#define CAN_RX_PIN 1


// 制御用変数
float target_position = 0.0;
float target_velocity = 0.0;
float kp = 10.0;  // 位置ゲイン
float kd = 1.0;   // 速度ゲイン
float target_torque = 1.0;

bool control_enabled = false; // 制御ループ ON/OFF フラグ（起動時はOFF）

// デバッグ: 起動直後にCANフレームを一定時間スパム送信して観測しやすくする
// オシロ/アナライザで拡張ID(29bit)の波形が出るか確認用
#define DEBUG_CAN_SPAM
#define DEBUG_CAN_SPAM_DURATION_MS 10000  // 起動後10秒間、100ms周期で送信

unsigned long last_control_time = 0;
const unsigned long control_interval = 10; // 10ms間隔で制御


static inline float adc_to_pm100_fastf(uint16_t raw);

void setup() {
    Serial.begin(115200);
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 1500) { delay(10); }  // PC接続待ち (最大1.5s)

    // M5 初期化（必要なら構成を調整）
    auto cfg = M5.config();
    // cfg.serial_baudrate = 115200;  // (M5Unifiedの版によって存在) 明示するなら
    AtomS3.begin(cfg, true);
    pinMode(14, INPUT); //ADC3
    
    Serial.println("=== CyberGear CA-IS3050G制御システム起動 ===");
    
    // CA-IS3050G CAN通信の初期化 (ESP32内蔵CAN使用)
    if (!motor.init(CAN_TX_PIN, CAN_RX_PIN, 1000000)) {
        Serial.println("CA-IS3050G CAN初期化失敗");
        while (1) {
            AtomS3.update();
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
}

float limit_spd = 1.0f; // 引数がなければデフォルト1rad/s

void loop() {
    AtomS3.update();
    uint16_t motor_angle = analogRead(14); //ADCの解像度は12ビット(0-4095)
    float pos = adc_to_pm100_fastf(motor_angle) * (PI / 180.0f); // -100.0 to +100.0 -> rad
    //Serial.println(adc_to_pm100_fastf(motor_angle));

    // 定期的な制御実行
    if (control_enabled && millis() - last_control_time >= control_interval) {
            last_control_time = millis();
            if (motor.setPositionMode(pos, limit_spd)) {
                Serial.printf("RunMode: Position loc=%.3f deg, limit_spd=%.3f rad/s\n", pos, limit_spd);
            } else {
                Serial.println("位置モード設定失敗");
            }
        }
        //Serial.println((motor_angle * 360.0f / 4096.0f) * PI / 180.0f);

    // シリアルコマンド処理
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n'); // 改行まで読み取り
        command.trim(); // 前後の空白削除
        
        if (command == "stop") {
            Serial.println("モーター停止+制御OFF");
            control_enabled = false;
            motor.disable();
        }
        else if (command == "start") {
            Serial.println("モーター開始+制御ON");
            motor.enable();
            control_enabled = true;
        }
        else if (command == "zero") {
            Serial.println("ゼロ位置設定");
            control_enabled = false; // 制御OFF
            motor.disable();
            motor.setZeroPosition();
        }else if(command.startsWith("spd ")) {
            limit_spd = command.substring(4).toFloat();
            Serial.print("速度制限設定: ");
            Serial.print(limit_spd);
            Serial.println(" rad/s");
        }
    }
delay(1);
}

static inline float adc_to_pm100_fastf(uint16_t raw) {
    int32_t s = (int32_t)raw - 2048;            // -2048..+2047
    float v = (float)s * (100.0f / 2047.0f);    // 係数は定数（割り算なし）
    if (v > 100.0f)  v = 100.0f;                // 端での丸め誤差を抑える
    if (v < -100.0f) v = -100.0f;
    return v;                                    // -100.0..+100.0
}
/*
void loop() {
    AtomS3.update();

#ifdef DEBUG_CAN_SPAM
    // 起動直後の一定時間、100msごとにGet Device IDを送ってバスに波形を出す
    static unsigned long spam_start = millis();
    static unsigned long spam_last = 0;
    if (millis() - spam_start < DEBUG_CAN_SPAM_DURATION_MS) {
        if (millis() - spam_last >= 100) {
            spam_last = millis();
            // motor_id=0x7F, host_id=0x7E宛てに1フレーム送信（受信待ちは最短に）
            CyberGear::pingMotor(0x7F, 0x7E, 1);
        }
    }
#endif
    
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
        }
        else if (command == "start") {
            Serial.println("モーター開始+制御ON");
            motor.enable();
            control_enabled = true;
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
        else if (command.startsWith("mode ")) {
            String arg = command.substring(5);
            arg.trim();
            arg.toLowerCase();
            if (arg == "op" || arg.startsWith("op")) {
                if (motor.setRunMode(MODE_MOTION)) {
                    control_enabled = true; // MIT制御を使うので制御ループON
                    Serial.println("RunMode: Operation (MIT) に切替");
                } else {
                    Serial.println("RunMode切替(Operation)失敗");
                }
            } else if (arg.startsWith("pos")) {
                // 使用法: mode pos <deg> [limit_spd]
                String rest = command.substring(9); // "mode pos "の後ろ
                rest.trim();
                float limit_spd = 5.0f;
                float deg = 0.0f;
                int sp = rest.indexOf(' ');
                if (sp >= 0) {
                    deg = rest.substring(0, sp).toFloat();
                    limit_spd = rest.substring(sp + 1).toFloat();
                } else {
                    deg = rest.toFloat();
                }
                float loc_ref = deg * PI / 180.0f;
                if (motor.setPositionMode(loc_ref, limit_spd)) {
                    control_enabled = false; // 位置モードではMIT制御を停止
                    Serial.printf("RunMode: Position loc=%.3f deg, limit_spd=%.3f rad/s\n", deg, limit_spd);
                } else {
                    Serial.println("位置モード設定失敗");
                }
            } else if (arg.startsWith("spd")) {
                // 使用法: mode spd <rad/s> [limit_cur]
                String rest = command.substring(9); // "mode spd "の後ろ
                rest.trim();
                float limit_cur = 2.0f;
                float spd = 0.0f;
                int sp = rest.indexOf(' ');
                if (sp >= 0) {
                    spd = rest.substring(0, sp).toFloat();
                    limit_cur = rest.substring(sp + 1).toFloat();
                } else {
                    spd = rest.toFloat();
                }
                if (motor.setSpeedMode(spd, limit_cur)) {
                    control_enabled = false; // 速度モードではMIT制御を停止
                    Serial.printf("RunMode: Speed spd=%.3f rad/s, limit_cur=%.3f A\n", spd, limit_cur);
                } else {
                    Serial.println("速度モード設定失敗");
                }
            } else if (arg.startsWith("cur")) {
                // 使用法: mode cur <A>
                String rest = command.substring(9); // "mode cur "の後ろ
                rest.trim();
                float iq = rest.toFloat();
                if (motor.setCurrentMode(iq)) {
                    control_enabled = false; // 電流モードではMIT制御を停止
                    Serial.printf("RunMode: Current iq_ref=%.3f A\n", iq);
                } else {
                    Serial.println("電流モード設定失敗");
                }
            } else {
                Serial.println("使用法: mode op | mode pos <deg> [limit_spd] | mode spd <rad/s> [limit_cur] | mode cur <A>");
            }
        }
        else if (command == "mode?") {
            uint8_t rm = 0xFF;
            if (motor.readParam8(ADDR_RUN_MODE, rm)) {
                const char* name = "Unknown";
                if (rm == MODE_MOTION)      name = "Operation";
                else if (rm == MODE_POSITION) name = "Position";
                else if (rm == MODE_SPEED)    name = "Speed";
                else if (rm == MODE_CURRENT)  name = "Current";
                Serial.printf("run_mode = %u (%s)\n", rm, name);
            } else {
                Serial.println("run_mode読み取り失敗");
            }
        }
        else if (command == "scan") {
            Serial.println("CyberGearスキャンを開始します...");
            motor.scanForMotors(0x01, 0x7F, 100);
        }
        else if (command.startsWith("torque ")) {
            target_torque = command.substring(7).toFloat();
            Serial.print("目標トルク設定: ");
            Serial.println(target_torque);
        }
        else if (command.startsWith("scan ")) {
            // 範囲指定スキャン: scan 0x10 0x20
            int space_pos = command.indexOf(' ', 5);
            if (space_pos > 0) {
                uint8_t start_id = (uint8_t)strtol(command.substring(5, space_pos).c_str(), NULL, 0);
                uint8_t end_id = (uint8_t)strtol(command.substring(space_pos + 1).c_str(), NULL, 0);
                Serial.printf("範囲指定スキャン: 0x%02X - 0x%02X\n", start_id, end_id);
                motor.scanForMotors(start_id, end_id, 100);
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
            Serial.println("stop             - モーター停止+制御OFF");
            Serial.println("start            - モーター開始+制御ON");
            Serial.println("control on       - 制御ループON（MIT制御送信）");
            Serial.println("control off      - 制御ループOFF");
            Serial.println("zero             - ゼロ位置設定");
            Serial.println("pos <deg>        - MIT制御: 目標位置設定（度）");
            Serial.println("vel <rad/s>      - MIT制御: 目標速度設定");
            Serial.println("kp <value>       - MIT制御: 位置ゲイン設定");
            Serial.println("kd <value>       - MIT制御: 速度ゲイン設定");
            Serial.println("torque <Nm>      - MIT制御: 目標トルク設定（現行実装では未送信項目）");
            Serial.println("mode op          - RunMode: Operation(MIT) に切替（制御ループON推奨）");
            Serial.println("mode pos <deg> [limit_spd] - RunMode: Position に切替してloc_ref/limit_spdを設定");
            Serial.println("mode spd <rad/s> [limit_cur] - RunMode: Speed に切替してspd_ref/limit_curを設定");
            Serial.println("mode cur <A>     - RunMode: Current に切替してiq_refを設定");
            Serial.println("mode?            - 現在のrun_modeを表示");
            Serial.println("scan             - 全CyberGearスキャン");
            Serial.println("scan <s> <e>     - 範囲指定スキャン (例: scan 0x01 0x7F)");
            Serial.println("ping <id>        - 特定IDをPingテスト (例: ping 0x7E)");
            Serial.println("help             - このヘルプ表示");
            Serial.println("==================");
        }
    }
    
    delay(1);
}

*/