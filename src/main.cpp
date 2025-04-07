/**
 * For Flexible_jaws controller
 * by X.Chang@Version1.0
 * LAB_121
 * 参数笔记
 * 最大张角对应传感器机械角度 0rad
 * 最小张角对应传感器机械角度 90rad
 * @version: v2.1.2
 */

#include <SimpleFOC.h>
#include <iostream>
#include <vector>
#include <stdexcept>
#include <cstdint>
#include <cstring>

// I2C 磁编码器实例初始化
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// 电机实例初始化
BLDCMotor motor = BLDCMotor(7);                         // 初始化一个motor实例，极对数为7
BLDCDriver3PWM driver = BLDCDriver3PWM(25, 26, 27, 33); // 绑定三相电机驱动的 PWM 引脚

// 引脚定义
// const int TX1 = 18;
// const int RX1 = 17;

// dirction
const int left_hand = -1;

// 角度设置变量
float target = 0; // 目标
float angle_limit = 100;
int counter = 0;

// 命令解释器的初始化
Commander command = Commander(Serial);
void doTarget(char *cmd) { command.scalar(&target, cmd); }
void onMotor(char *cmd) { command.motor(&motor, cmd); }
// 自定义函数
int32_t decodeToTwosComplement(const std::vector<uint8_t> &byteList) // decode
{
    if (byteList.size() < 4)
    {
        std::cerr << "Error: Byte list must contain at least 4 bytes." << std::endl;
        return 0;
    }

    // 将字节序列直接解释为一个32位有符号整数
    int32_t number;
    memcpy(&number, byteList.data(), sizeof(number));

    return number;
}

std::vector<uint8_t> getSubVector(const std::vector<uint8_t> &originalVector, int start, int end)
{ // 用来分割vector
    // 请注意，end 是不包含在内的，所以实际上是到 end - 1 的元素
    return std::vector<uint8_t>(originalVector.begin() + start, originalVector.begin() + end);
}

std::vector<int32_t> getModeSpeed(const std::vector<uint8_t> &originalVector)
{
    std::vector<int32_t> res = {};
    std::vector<uint8_t> originalVector1 = getSubVector(originalVector, 8, 12);
    std::vector<uint8_t> originalVector2 = getSubVector(originalVector, 12, 16);
    int32_t mode = decodeToTwosComplement(originalVector1);
    int32_t speed = decodeToTwosComplement(originalVector2);
    res.push_back(mode);
    res.push_back(speed);

    return res;
}

bool checkId(const std::vector<uint8_t> &originalVector)
{
    if (originalVector[7] == 0x14)
    {
        return true;
    }
    else
        return false;
}
void setup()
{

    // 传感器
    sensor.init();             // 初始化磁传感器
    motor.linkSensor(&sensor); // 将电机与传感器连接

    // 电机驱动配置
    driver.voltage_power_supply = 12; // 供电电压
    driver.init();                    // 驱动初始化
    motor.linkDriver(&driver);        // 绑定驱动与电机

    // FOC控制模式选择
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // 运动控制环的配置
    // motor.controller = MotionControlType::velocity; // 速度环
    motor.controller = MotionControlType::torque; // 力矩环

    // 控制部分的配置
    // default parameters in defaults.h

    // velocity PI control
    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 2;
    // max velocity
    motor.voltage_limit = 6;
    motor.voltage_sensor_align = 1;

    // velocity low pass filtering time constant
    // the lower the less filtered
    motor.LPF_velocity.Tf = 0.05;

    // angle P controller
    motor.P_angle.P = 20;
    // maximal velocity of the position control
    motor.velocity_limit = 12;

    // motor.sensor_offset = 1.78;              // Set the zero electrical angle
    motor.sensor_direction = Direction::CCW; // Set the sensor direction to counter-clockwise

    // use monitoring with serial
    Serial.begin(115200);
    // Serial1.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 18, 17);
    // comment out if not needed
    motor.useMonitoring(Serial);

    // initialize motor
    motor.init();
    // align sensor and start FOC
    // motor.();
    motor.initFOC();

    _delay(100); // 等待主控制程序启动

    Serial.println(F("Motor ready"));
}

void loop()
{
    motor.loopFOC();
    if(counter < 5000){
        counter = counter + 1;
        motor.controller = MotionControlType::torque;
        motor.move(1.5);
        if(counter == 5000) motor.sensor_offset = -sensor.getAngle();
    }
    else {
        if (left_hand * motor.shaft_angle > angle_limit) {
            // 超过限制时，切换到力矩控制并停止电机
            motor.controller = MotionControlType::torque;
            target = 0;
        } else {
            // 未超过限制时，继续正常运动
            motor.move(target);
        }
        // Serial.println(motor.shaft_angle);
        if (Serial1.available() >= 16) {        
            std::vector<uint8_t> data(16);
            Serial1.readBytes(data.data(), 16);
            
            // if (data[7] == 0x14) {
            uint8_t mode = data[8];
            uint16_t value = (data[10] << 8) | data[11];
            switch (mode) {
                case 0: // 初始化
                    break;
                case 1: // 力矩控制
                    motor.controller = MotionControlType::torque;
                    target = left_hand * static_cast<float>(value) / 100.0;
                    break;
                case 2: // 角度控制
                    motor.controller = MotionControlType::angle;
                    target = left_hand * static_cast<float>(value) / 100.0;
                    break;
                case 3: // 角度控制带力矩限制
                    float angle_target = static_cast<float>(value) / 100.0;
                    uint16_t voltage_limit_value = (data[12] << 8) | data[13];
                    float voltage_limit = static_cast<float>(voltage_limit_value) / 100.0;
                    motor.controller = MotionControlType::angle;
                    target = left_hand * angle_target;
                    motor.voltage_limit = voltage_limit;
                    break;
                // }
            }
        }
    }
}