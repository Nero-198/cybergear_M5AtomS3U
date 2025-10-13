#include "CyberGear.h"

// デバッグ用: ACK不要モードで送信継続（単体ノード/ACKなしでも波形が出るように）
#ifndef TWAI_MODE_FOR_DEBUG
#define TWAI_MODE_FOR_DEBUG TWAI_MODE_NO_ACK  // 必要に応じて TWAI_MODE_NORMAL に戻してください
#endif

// 静的変数の初期化
uint8_t CyberGear::found_motors[127] = {0};
uint8_t CyberGear::found_motor_count = 0;

static inline uint32_t make_id(uint8_t comm_type, uint16_t data16, uint8_t motor_id) {
    return ((uint32_t)comm_type << 24) | ((uint32_t)data16 << 8) | motor_id;
}

CyberGear::CyberGear(uint8_t motor_id, uint8_t master_id) {
    this->motor_id = motor_id;
    this->master_id = master_id;
}

bool CyberGear::init(int can_tx_pin, int can_rx_pin, long baudrate) {
    // TWAI設定（デフォルトマクロ使用で未初期化項目のリスクを避ける）
twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)can_tx_pin, (gpio_num_t)can_rx_pin, TWAI_MODE_FOR_DEBUG);
    g_config.tx_queue_len = 10;
    g_config.rx_queue_len = 10;
    g_config.alerts_enabled = TWAI_ALERT_TX_FAILED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_TX_IDLE;
    g_config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    
    // タイミング設定（C++で三項演算子に指定初期化子は不可のためifで分岐）
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    if (baudrate != 1000000) {
        t_config = TWAI_TIMING_CONFIG_500KBITS();
    }
    
    // フィルタ設定（全て受信）
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // TWAI初期化
    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial.println("TWAI driver install failed");
        return false;
    }
    
    if (twai_start() != ESP_OK) {
        Serial.println("TWAI start failed");
        return false;
    }
    
    twai_status_info_t info;
    if (twai_get_status_info(&info) == ESP_OK) {
        Serial.printf("TWAI started: state=%d, tx_err=%d, rx_err=%d\n",
                      info.state, info.tx_error_counter, info.rx_error_counter);
    Serial.println("CyberGear initialized with CA-IS3050G (ESP32-TWAI)");
    Serial.printf("Motor ID: 0x%02X, Master ID: 0x%02X\n", motor_id, master_id);
    Serial.printf("CAN TX Pin: %d, RX Pin: %d, Baudrate: %ld\n", can_tx_pin, can_rx_pin, baudrate);
    
    return true;
}
}

void CyberGear::sendCanFrame(uint32_t can_id, const uint8_t* data, uint8_t len) {
    twai_message_t tx_msg = {};
    tx_msg.identifier = can_id;
    tx_msg.data_length_code = len;
    tx_msg.flags = TWAI_MSG_FLAG_EXTD; // 29-bit extended ID
    
    for (int i = 0; i < len && i < 8; i++) {
        tx_msg.data[i] = data[i];
    }
    
    esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(20));
    
    // 簡易デバッグ出力（MIT制御はスパム防止で間引き）
    uint8_t comm_type = (uint8_t)((can_id >> 24) & 0x1F);
    static uint32_t oc_count = 0;
    bool should_print = (comm_type != CMD_MOTION_CONTROL) || ((++oc_count % 50) == 0);
    if (should_print) {
        Serial.printf("[CAN TX] ID=0x%08lX (type=%u,motor=0x%02X) len=%u err=%d data=", (unsigned long)can_id, comm_type, (uint8_t)(can_id & 0xFF), len, (int)err);
        for (int i = 0; i < len && i < 8; ++i) {
            Serial.printf("%02X ", data[i]);
        }
        Serial.println();
    }
}

bool CyberGear::receiveCanFrame(uint32_t& can_id, uint8_t* data, uint8_t& len, uint32_t timeout_ms) {
    twai_message_t rx_msg = {};
    
    if (twai_receive(&rx_msg, pdMS_TO_TICKS(timeout_ms)) == ESP_OK) {
        if (!(rx_msg.flags & TWAI_MSG_FLAG_EXTD)) {
            return false; // 標準IDは無視
        }
        can_id = rx_msg.identifier;
        len = rx_msg.data_length_code;
        for (int i = 0; i < len && i < 8; i++) {
            data[i] = rx_msg.data[i];
        }
        return true;
    }
    
    return false;
}

float CyberGear::uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

uint16_t CyberGear::float_to_uint(float x, float x_min, float x_max, uint8_t bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

bool CyberGear::enable() {
    uint8_t data[8] = {0};
    uint32_t can_id = make_id(CMD_MOTOR_ENABLE, master_id, motor_id);
    sendCanFrame(can_id, data, 8);
    
    Serial.println("モーター有効化コマンド送信");
    return true;
}

bool CyberGear::disable() {
    uint8_t data[8] = {0};
    uint32_t can_id = make_id(CMD_MOTOR_STOP, master_id, motor_id);
    sendCanFrame(can_id, data, 8);
    
    Serial.println("モーター停止コマンド送信");
    return true;
}

bool CyberGear::setZeroPosition() {
    uint8_t data[8] = {0};
    data[0] = 1; // 実行フラグ
    uint32_t can_id = make_id(CMD_SET_MECH_POS_TO_ZERO, master_id, motor_id);
    sendCanFrame(can_id, data, 8);
    
    Serial.println("ゼロ位置設定コマンド送信");
    return true;
}

bool CyberGear::setMotionControl(float position, float velocity, float kp, float kd, float torque) {
    // 16bitスケーリング（LEで送信）
    uint16_t pos_uint = float_to_uint(constrain(position, -4 * PI, 4 * PI), -4 * PI, 4 * PI, 16);
    uint16_t vel_uint = float_to_uint(constrain(velocity, -30.0, 30.0), -30.0, 30.0, 16);
    uint16_t kp_uint  = float_to_uint(constrain(kp, 0.0, 500.0), 0.0, 500.0, 16);
    uint16_t kd_uint  = float_to_uint(constrain(kd, 0.0, 5.0), 0.0, 5.0, 16);
    
    uint8_t data[8] = {
        (uint8_t)(pos_uint & 0xFF), (uint8_t)(pos_uint >> 8),
        (uint8_t)(vel_uint & 0xFF), (uint8_t)(vel_uint >> 8),
        (uint8_t)(kp_uint  & 0xFF), (uint8_t)(kp_uint  >> 8),
        (uint8_t)(kd_uint  & 0xFF), (uint8_t)(kd_uint  >> 8)
    };
    
    uint32_t can_id = make_id(CMD_MOTION_CONTROL, master_id, motor_id);
    sendCanFrame(can_id, data, 8);
    
    return true;
}

bool CyberGear::writeParam(uint16_t addr, float value) {
    uint8_t data[8];
    data[0] = addr & 0xFF;
    data[1] = (addr >> 8) & 0xFF;
    data[2] = 0x00;
    data[3] = 0x00;
    
    // floatをバイト配列に変換
    memcpy(&data[4], &value, 4);
    
    uint32_t can_id = make_id(CMD_RAM_WRITE, master_id, motor_id);
    sendCanFrame(can_id, data, 8);
    
    return true;
}

bool CyberGear::writeParam8(uint16_t addr, uint8_t value) {
    uint8_t data[8];
    data[0] = addr & 0xFF;       // index LSB
    data[1] = (addr >> 8) & 0xFF; // index MSB
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = value;             // u8 value in Byte4
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    
    uint32_t can_id = make_id(CMD_RAM_WRITE, master_id, motor_id);
    sendCanFrame(can_id, data, 8);
    return true;
}

bool CyberGear::readParam(uint16_t addr, float& value) {
    uint8_t data[8];
    data[0] = addr & 0xFF;
    data[1] = (addr >> 8) & 0xFF;
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = 0x00;
    data[5] = 0x00;
    data[6] = 0x00;
    data[7] = 0x00;
    
    uint32_t can_id = make_id(CMD_RAM_READ, master_id, motor_id);
    sendCanFrame(can_id, data, 8);
    
    // 応答を待機
    uint32_t response_id;
    uint8_t response_data[8];
    uint8_t response_len;
    
    if (receiveCanFrame(response_id, response_data, response_len, 500)) {
        if (response_len >= 8 && 
            response_data[0] == (addr & 0xFF) && 
            response_data[1] == ((addr >> 8) & 0xFF)) {
            memcpy(&value, &response_data[4], 4);
            return true;
        }
    }
    
    return false;
}

bool CyberGear::setTorqueLimit(float torque_limit) {
    // トルク制限は電流制限にマップ（仕様によりTorque indexが未定義の場合）
    return writeParam(ADDR_LIMIT_TORQUE, torque_limit);
}

bool CyberGear::setSpeedLimit(float speed_limit) {
    return writeParam(ADDR_LIMIT_SPEED, speed_limit);
}

bool CyberGear::setCurrentLimit(float current_limit) {
    return writeParam(ADDR_LIMIT_CURRENT, current_limit);
}

bool CyberGear::getMotorStatus(MotorStatus& status) {
    // フィードバック（通信タイプ2）の応答を取得（LE）
    uint32_t can_id;
    uint8_t data[8];
    uint8_t len;
    
    if (receiveCanFrame(can_id, data, len, 100)) {
        // 29bit ID: Bit28..24=comm_type, Bit7..0=motor_id
        uint8_t comm_type = (uint8_t)((can_id >> 24) & 0x1F);
        uint8_t mid = (uint8_t)(can_id & 0xFF);
        if (comm_type != 2 || mid != motor_id) {
            return false;
        }
        
        if (len >= 6) {
            // 受信データをLEでデコード
            uint16_t pos_uint = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
            uint16_t vel_uint = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
            uint16_t tor_uint = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
            
            status.position = uint_to_float(pos_uint, -4 * PI, 4 * PI, 16);
            status.velocity = uint_to_float(vel_uint, -30.0, 30.0, 16);
            status.torque = uint_to_float(tor_uint, -12.0, 12.0, 16);
            
            if (len >= 8) {
                uint16_t t_raw = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
                status.temperature = (float)t_raw * 0.1f;
            }
            return true;
        }
    }
    return false;
}

bool CyberGear::changeCanId(uint8_t new_id) {
    // 特例: IDのBit23..16に new_id、Bit15..8に host_id(master_id)
    uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
    uint32_t can_id = ((uint32_t)CMD_CHANGE_CAN_ID << 24) |
                      (((uint32_t)new_id << 16) | ((uint32_t)master_id << 8)) |
                      motor_id;
    sendCanFrame(can_id, data, 8);
    
    // 成功した場合、内部のIDを更新
    motor_id = new_id;
    
    Serial.print("CAN ID変更: ");
    Serial.println(new_id);
    return true;
}

// CAN IDスキャン機能
bool CyberGear::scanForMotors(uint8_t start_id, uint8_t end_id, uint32_t timeout_ms) {
    Serial.println("=== CyberGear CAN IDスキャン開始 ===");
    Serial.printf("スキャン範囲: 0x%02X - 0x%02X\n", start_id, end_id);
    
    // 検出リストをクリア
    found_motor_count = 0;
    memset(found_motors, 0, sizeof(found_motors));
    
    for (uint8_t test_id = start_id; test_id <= end_id; test_id++) {
        Serial.printf("ID 0x%02X をテスト中...", test_id);
        
        if (pingMotor(test_id, master_id, timeout_ms)) {
            found_motors[found_motor_count] = test_id;
            found_motor_count++;
            Serial.printf(" → 検出! ✓\n");
        } else {
            Serial.printf(" → 応答なし\n");
        }
        
        // 進捗表示（10個ごと）
        if ((test_id - start_id + 1) % 10 == 0) {
            Serial.printf("進捗: %d/%d\n", test_id - start_id + 1, end_id - start_id + 1);
        }
        
        delay(10); // 少し間隔を空ける
    }
    
    Serial.println("=== スキャン完了 ===");
    Serial.printf("検出されたCyberGear: %d台\n", found_motor_count);
    
    if (found_motor_count > 0) {
        printFoundMotors();
        return true;
    } else {
        Serial.println("CyberGearが見つかりませんでした。");
        Serial.println("・CAN接続を確認してください");
        Serial.println("・CyberGearの電源を確認してください");
        Serial.println("・CANボーレートを確認してください");
        return false;
    }
}

bool CyberGear::pingMotor(uint8_t motor_id, uint8_t master_id, uint32_t timeout_ms) {
    // デバイス情報取得（通信タイプ0）
    uint8_t data[8] = {0};
    uint32_t can_id = ((uint32_t)CMD_GET_DEVICE_ID << 24) | ((uint32_t)master_id << 8) | motor_id;
    
    // 既存のメッセージをクリア
    twai_message_t dummy_msg;
    while (twai_receive(&dummy_msg, pdMS_TO_TICKS(5)) == ESP_OK) {
        // drain
    }
    
    // Pingコマンド送信（拡張ID）
    twai_message_t tx_msg = {};
    tx_msg.identifier = can_id;
    tx_msg.data_length_code = 8;
    tx_msg.flags = TWAI_MSG_FLAG_EXTD;
    memcpy(tx_msg.data, data, 8);
    twai_transmit(&tx_msg, pdMS_TO_TICKS(20));
    
    // 応答を待機
    unsigned long start_time = millis();
    while (millis() - start_time < timeout_ms) {
        twai_message_t rx_msg = {};
        if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
            if (!(rx_msg.flags & TWAI_MSG_FLAG_EXTD)) continue;
            uint32_t rid = rx_msg.identifier;
            uint8_t response_motor_id = (uint8_t)(rid & 0xFF);
            if (response_motor_id == motor_id) {
                return true;
            }
        }
    }
    return false;
}

void CyberGear::printFoundMotors() {
    Serial.println("--- 検出されたCyberGear ---");
    for (uint8_t i = 0; i < found_motor_count; i++) {
        Serial.printf("CyberGear #%d: ID = 0x%02X (%d)\n", 
                     i + 1, found_motors[i], found_motors[i]);
    }
    Serial.println("-------------------------");
    
    if (found_motor_count == 1) {
        Serial.printf("推奨: motor_id = 0x%02X を使用してください\n", found_motors[0]);
    } else if (found_motor_count > 1) {
        Serial.println("複数のCyberGearが検出されました。");
        Serial.println("使用したいモーターのIDを選択してください。");
    }
}
