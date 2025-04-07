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

// direction
const int left_hand = -1;

// 角度设置变量
float target = 0; // 目标
float angle_limit = 100;
int counter = 0;

// 速度允差
const float velocity_threshold = 1;
const float angle_threshold = 200;

// Control
float error = 0;
float torque_limit = 0;
float angle_target = 0;

// FOC 任务函数
void focTask(void *pvParameters) {
    while (1) {
        motor.loopFOC();  // 在独立线程中运行 motor.loopFOC
    }
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
    motor.controller = MotionControlType::torque; // 力矩环

    // 控制部分的配置
    motor.PID_velocity.P = 0.2;
    motor.PID_velocity.I = 2;
    motor.voltage_limit = 3;
    motor.voltage_sensor_align = 1;

    motor.LPF_velocity.Tf = 0.05;
    motor.P_angle.P = 20;
    motor.velocity_limit = 12;

    motor.sensor_offset = 1.67;
    motor.sensor_direction = Direction::CCW; // 设置传感器方向为逆时针

    // 使用串口监控
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, 18, 17);
    motor.useMonitoring(Serial);

    // 初始化电机
    motor.init();
    motor.initFOC();

    // 创建 FOC 任务，固定到核心 0
    xTaskCreatePinnedToCore(
        focTask,
        "FOC Task",
        40960,
        NULL,
        2,
        NULL,
        0
    );

    Serial.println(F("Motor ready"));
}

void loop()
{
    if(counter < 1000){
        counter = counter + 1;
        motor.controller = MotionControlType::torque;
        motor.move(1.5);
        if(counter == 1000) motor.sensor_offset = -sensor.getAngle();
    }
    else {
        if (left_hand * motor.shaft_angle > angle_limit) {
            motor.controller = MotionControlType::torque;
            target = 0;
        } else {
            error = target - motor.shaft_angle;
            // Serial.println(motor.shaft_velocity);
            if (abs(motor.shaft_velocity) < velocity_threshold && abs(error) > angle_threshold) {
                motor.controller = MotionControlType::torque;
                target = torque_limit;
            }
            else {
                motor.controller = MotionControlType::angle;
                target = left_hand * angle_target;
            }

            motor.move(target);
        }

        if (Serial1.available() >= 16) {        
            std::vector<uint8_t> data(16);
            Serial1.readBytes(data.data(), 16);
            
            uint16_t value = (data[10] << 8) | data[11];
            angle_target = static_cast<float>(value) / 100.0;

            uint16_t torque_limit_value = (data[12] << 8) | data[13];
            torque_limit = static_cast<float>(torque_limit_value) / 100.0;          
        }
    }
    delay(2);
}
