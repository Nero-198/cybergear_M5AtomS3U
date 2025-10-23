#ifndef CYBERGEAR_H
#define CYBERGEAR_H

#include <Arduino.h>
#include "driver/twai.h"

// CyberGear CAN通信コマンド定義
#define CMD_MOTOR_ENABLE    0x03
#define CMD_MOTOR_STOP      0x04
#define CMD_SET_MECH_POS_TO_ZERO 0x06
#define CMD_CHANGE_CAN_ID   0x07
#define CMD_RAM_READ        0x11
#define CMD_RAM_WRITE       0x12
#define CMD_MOTION_CONTROL  0x01

// スキャン用コマンド
#define CMD_GET_DEVICE_ID   0x00  // デバイス情報取得

// CyberGearパラメータアドレス
#define ADDR_MOTOR_STATUS   0x30A
#define ADDR_MOTOR_CONTROL  0x300
// Updated parameter indices per CyberGear manual
#define ADDR_LIMIT_SPEED    0x7017   // limit_spd [rad/s] (float)
#define ADDR_LIMIT_CURRENT  0x7018   // limit_cur [A] (float)
// No explicit torque limit index in doc; map to current limit for compatibility
#define ADDR_LIMIT_TORQUE   0x7018

// Run mode and reference indices
#define ADDR_RUN_MODE       0x7005   // run_mode (u8) 0:Operation 1:Position 2:Speed 3:Current
#define ADDR_IQ_REF         0x7006   // iq_ref (float) [A]
#define ADDR_SPD_REF        0x700A   // spd_ref (float) [rad/s]
#define ADDR_LOC_REF        0x7016   // loc_ref (float) [rad]

// モーター制御モード
#define MODE_MOTION         0x00
#define MODE_POSITION       0x01
#define MODE_SPEED          0x02
#define MODE_CURRENT        0x03

class CyberGear {
private:
    uint8_t motor_id;
    uint8_t master_id;
    
    // 検出されたモーターのリスト
    static uint8_t found_motors[127];
    static uint8_t found_motor_count;
    
    // CAN通信用の内部関数
    void sendCanFrame(uint32_t can_id, const uint8_t* data, uint8_t len = 8);
    bool receiveCanFrame(uint32_t& can_id, uint8_t* data, uint8_t& len, uint32_t timeout_ms = 100);
    
    // データ変換用の内部関数
    float uint_to_float(uint16_t x_int, float x_min, float x_max, uint8_t bits);
    uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits);

public:
    // コンストラクタ (Motor ID: 7E)
    CyberGear(uint8_t motor_id = 0x7E, uint8_t master_id = 0x00);
    
    // 初期化 (CA-IS3050G用 - ESP32内蔵CAN使用)
    bool init(int can_tx_pin = 2, int can_rx_pin = 1, long baudrate = 1000000);
    
    // モーター制御
    bool enable();
    bool disable();
    bool setZeroPosition();

    typedef enum {
        CONTROL_MOTION = 0,
        CONTROL_POSITION = 1,
        CONTROL_SPEED = 2,
        CONTROL_CURRENT = 3
    } ControlMode;
    
    // モーション制御
    bool setMotionControl(float position, float velocity, float kp, float kd, float torque);

    // ランモード/各モード設定
    bool setRunMode(uint8_t mode);                 // 0:Operation 1:Position 2:Speed 3:Current
    bool setPositionMode(float loc_ref, float limit_spd);
    bool setSpeedMode(float spd_ref, float limit_cur);
    bool setCurrentMode(float iq_ref);
    
    // パラメータ読み書き
    bool writeParam(uint16_t addr, float value);
    bool writeParam8(uint16_t addr, uint8_t value);
    bool readParam(uint16_t addr, float& value);
    bool readParam8(uint16_t addr, uint8_t& value);
    
    // 制限値設定
    bool setTorqueLimit(float torque_limit);
    bool setSpeedLimit(float speed_limit);
    bool setCurrentLimit(float current_limit);
    
    // モーター状態取得
    struct MotorStatus {
        float position;
        float velocity;
        float torque;
        float temperature;
    };
    
    bool getMotorStatus(MotorStatus& status);
    
    // CAN ID変更
    bool changeCanId(uint8_t new_id);
    
    // CAN IDスキャン機能
    bool scanForMotors(uint8_t start_id = 0x01, uint8_t end_id = 0x7F, uint32_t timeout_ms = 100);
    static bool pingMotor(uint8_t motor_id, uint8_t master_id = 0x00, uint32_t timeout_ms = 100);
    void printFoundMotors();
};

#endif
