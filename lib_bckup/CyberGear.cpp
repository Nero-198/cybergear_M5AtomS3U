// This duplicate implementation in lib/ has been intentionally disabled
// to avoid ODR (multiple definition) conflicts with src/CyberGear.cpp.
// The active and up-to-date implementation is in: src/CyberGear.cpp
// If you want to restore this library version, remove the #if 0 guard
// and ensure only one implementation is compiled (adjust src_filter/lib_ignore).

#if 0
#include "CyberGear.h"

CyberGear::CyberGear(uint8_t motor_id, uint8_t master_id) {
    this->motor_id = motor_id;
    this->master_id = master_id;
}

bool CyberGear::init(int can_tx_pin, int can_rx_pin, long baudrate) {
    // TWAI設定
    twai_general_config_t g_config;
    g_config.mode = TWAI_MODE_NORMAL;
    g_config.tx_io = (gpio_num_t)can_tx_pin;
    g_config.rx_io = (gpio_num_t)can_rx_pin;
    g_config.clkout_io = TWAI_IO_UNUSED;
    g_config.bus_off_io = TWAI_IO_UNUSED;
    g_config.tx_queue_len = 5;
    g_config.rx_queue_len = 5;
    g_config.alerts_enabled = TWAI_ALERT_NONE;
    g_config.clkout_divider = 0;
    g_config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    
    // タイミング設定（1Mbps）
    twai_timing_config_t t_config;
    if (baudrate == 1000000) {
        t_config = TWAI_TIMING_CONFIG_1MBITS();
    } else {
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
    
    Serial.println("CyberGear initialized with CA-IS3050G (ESP32-TWAI)");
    Serial.printf("Motor ID: 0x%02X, Master ID: 0x%02X\n", motor_id, master_id);
    Serial.printf("CAN TX Pin: %d, RX Pin: %d, Baudrate: %ld\n", can_tx_pin, can_rx_pin, baudrate);
    
    return true;
}

void CyberGear::sendCanFrame(uint16_t can_id, uint8_t* data, uint8_t len) {
    twai_message_t tx_msg;
    tx_msg.identifier = can_id;
    tx_msg.data_length_code = len;
    tx_msg.flags = TWAI_MSG_FLAG_NONE;
    
    for (int i = 0; i < len; i++) {
        tx_msg.data[i] = data[i];
    }
    
    twai_transmit(&tx_msg, portMAX_DELAY);
}

bool CyberGear::receiveCanFrame(uint16_t& can_id, uint8_t* data, uint8_t& len, uint32_t timeout_ms) {
    twai_message_t rx_msg;
    
    if (twai_receive(&rx_msg, pdMS_TO_TICKS(timeout_ms)) == ESP_OK) {
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
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint16_t can_id = (CMD_MOTOR_ENABLE << 5) | master_id;
    sendCanFrame(can_id, data, 8);
    
    Serial.println("モーター有効化コマンド送信");
    return true;
}

bool CyberGear::disable() {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    uint16_t can_id = (CMD_MOTOR_STOP << 5) | master_id;
    sendCanFrame(can_id, data, 8);
    
    Serial.println("モーター停止コマンド送信");
    return true;
}

bool CyberGear::setZeroPosition() {
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    uint16_t can_id = (CMD_SET_MECH_POS_TO_ZERO << 5) | master_id;
    sendCanFrame(can_id, data, 8);
    
    Serial.println("ゼロ位置設定コマンド送信");
    return true;
}

bool CyberGear::setMotionControl(float position, float velocity, float kp, float kd, float torque) {
    // パラメータの範囲制限とエンコード
    uint16_t pos_uint = float_to_uint(constrain(position, -4 * PI, 4 * PI), -4 * PI, 4 * PI, 16);
    uint16_t vel_uint = float_to_uint(constrain(velocity, -30.0, 30.0), -30.0, 30.0, 16);
    uint16_t kp_uint = float_to_uint(constrain(kp, 0.0, 500.0), 0.0, 500.0, 16);
    uint16_t kd_uint = float_to_uint(constrain(kd, 0.0, 5.0), 0.0, 5.0, 16);
    uint16_t tor_uint = float_to_uint(constrain(torque, -12.0, 12.0), -12.0, 12.0, 16);
    
    // CANフレームデータの作成
    uint8_t data[8];
    data[0] = pos_uint >> 8;
    data[1] = pos_uint & 0xFF;
    data[2] = vel_uint >> 8;
    data[3] = vel_uint & 0xFF;
    data[4] = kp_uint >> 8;
    data[5] = kp_uint & 0xFF;
    data[6] = kd_uint >> 8;
    data[7] = kd_uint & 0xFF;
    
    uint16_t can_id = (CMD_MOTION_CONTROL << 5) | motor_id;
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
    
    uint16_t can_id = (CMD_RAM_WRITE << 5) | motor_id;
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
    
    uint16_t can_id = (CMD_RAM_READ << 5) | motor_id;
    sendCanFrame(can_id, data, 8);
    
    // 応答を待機
    uint16_t response_id;
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
    return writeParam(ADDR_LIMIT_TORQUE, torque_limit);
}

bool CyberGear::setSpeedLimit(float speed_limit) {
    return writeParam(ADDR_LIMIT_SPEED, speed_limit);
}

bool CyberGear::setCurrentLimit(float current_limit) {
    return writeParam(ADDR_LIMIT_CURRENT, current_limit);
}

bool CyberGear::getMotorStatus(MotorStatus& status) {
    // モーション制御コマンドの応答でステータスを取得
    uint16_t can_id;
    uint8_t data[8];
    uint8_t len;
    
    if (receiveCanFrame(can_id, data, len, 100)) {
        if (len >= 6) {
            // 受信データをデコード
            uint16_t pos_uint = (data[0] << 8) | data[1];
            uint16_t vel_uint = (data[2] << 8) | data[3];
            uint16_t tor_uint = (data[4] << 8) | data[5];
            
            status.position = uint_to_float(pos_uint, -4 * PI, 4 * PI, 16);
            status.velocity = uint_to_float(vel_uint, -30.0, 30.0, 16);
            status.torque = uint_to_float(tor_uint, -12.0, 12.0, 16);
            
            if (len >= 8) {
                status.temperature = ((float)data[6]) * 0.1; // 温度データがある場合
            }
            
            return true;
        }
    }
    
    return false;
}

bool CyberGear::changeCanId(uint8_t new_id) {
    uint8_t data[8] = {0x00, 0x00, 0x00, 0x00, new_id, 0x00, 0x00, 0x00};
    uint16_t can_id = (CMD_CHANGE_CAN_ID << 5) | motor_id;
    sendCanFrame(can_id, data, 8);
    
    // 成功した場合、内部のIDを更新
    motor_id = new_id;
    
    Serial.print("CAN ID変更: ");
    Serial.println(new_id);
    return true;
}