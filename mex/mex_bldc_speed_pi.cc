/**
 * @file mex_ipark.cc
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2026-05-25
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#define _USE_MATH_DEFINES

#include <cstdint>
#include <time.h>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <chrono>
#include <string>

#include "mex.hpp"
#include "mexAdapter.hpp"

#define ENABLE_PICA_DRIVE_DEBUG 1

#include "drive_conf.h"
#include "bldc/bldc.hpp"
#include "bldc/config_manager.hpp"

#include <spdlog/spdlog.h>
#include "spdlog/sinks/basic_file_sink.h"

using namespace matlab::data;
using matlab::mex::ArgumentList;

using ofstream = std::ofstream;

using namespace pica::motor::bldc;

class MexFunction : public matlab::mex::Function {
public:
    MexFunction()
    {
        {
            // 日志初始化
            std::stringstream ss;
            ss << time(nullptr);
            auto logger = spdlog::basic_logger_mt("speed", "logs/" + ss.str() + ".log");
            
            logger->set_level(spdlog::level::info);
            logger->flush_on(spdlog::level::err);

            spdlog::set_default_logger(logger);
        }

        spdlog::info("Mex start.");

        init();

        ConfigMgr::GetInstance()->init(&m_flash);

        spdlog::info("Mex: ConfigMgr addr at {}, motor at {}", (uint64_t)ConfigMgr().GetInstance(), uint64_t(ConfigMgr().GetInstance()->motor()));

        m_bldc.init();
    }
    ~MexFunction()
    {
        spdlog::info("Mex exit.");

        spdlog::shutdown();
    }

    void operator()(ArgumentList outputs, ArgumentList inputs) {
        // update input args
        // checkArguments(outputs, inputs);

        try {
            input(inputs);

            IRQHandler();

            run();

            output(&outputs);

        } catch(const std::exception& e) {
            spdlog::error(e.what());
        }
        

    }

    void checkArguments(ArgumentList outputs, ArgumentList inputs) {
        // Get pointer to engine
        std::shared_ptr<matlab::engine::MATLABEngine> matlabPtr = getEngine();

        // Get array factory
        ArrayFactory factory;

        // Check offset argument: First input must be scalar double
        if (inputs[0].getType() != ArrayType::DOUBLE ||
            inputs[0].getType() == ArrayType::COMPLEX_DOUBLE ||
            inputs[0].getNumberOfElements() != 1)
        {
            matlabPtr->feval(u"error",
                0,
                std::vector<Array>({ factory.createScalar("First input must be scalar double") }));
        }

        // Check array argument: Second input must be double array
        if (inputs[1].getType() != ArrayType::DOUBLE ||
            inputs[1].getType() == ArrayType::COMPLEX_DOUBLE)
        {
            matlabPtr->feval(u"error",
                0,
                std::vector<Array>({ factory.createScalar("Input must be double array") }));
        }
        // Check number of outputs
        if (outputs.size() > 1) {
            matlabPtr->feval(u"error",
                0,
                std::vector<Array>({ factory.createScalar("Only one output is returned") }));
        }
    }

private:
    void init();

    /**
     * @brief 处理输入数据，如参数变换等
     *
     */
    void input(ArgumentList& inputs);

    void IRQHandler();

    void run();

    void output(ArgumentList *outputs);

private:
    float m_voltage_shunt[3], m_ibus, m_voltage_shunt_calibrator[3];
    float m_voltage_phase[3], m_vbus;
    float m_duty_cycles[3];

    float m_theta_mach; /*!< 电机机械角度, rad */
    float m_omega_mach; /*!< 电机机械角速度, rad/s */

    float m_torque{0};
    float m_speed{0};
    float m_position{0};

    double m_clock{0};
    hrt_absnano m_tick{0};

    BLDC m_bldc;
    ConfigManager::Flash m_flash;

    bool m_counting_up{true}; /*!< 模拟当前 PWM 计数方向: true - 向上, false-向下*/

    ofstream m_csv;
};

void MexFunction::run()
{
    m_bldc.do_checks();
    
    // 更新电机控制环参数
    m_bldc.setTorqueSetpoint(m_torque);
    m_bldc.setVelocitySetpoint(m_speed);
    m_bldc.setPositionSetpoint(m_position);

    m_bldc.update(m_tick + 45);

    // 更新校正电流
    m_bldc.sampleCurrentCalibratorHandler(NULL, PICA_DRIVE_CURRENT_MEASURE_PERIOD);
    
    // 运行电流环
    if (!m_bldc.run(m_tick + 120)) {
        spdlog::info("run bldc failed");
    }
}

void MexFunction::IRQHandler()
{
    // 更新传感器参数
    m_bldc.sampleBusVoltageHandler(m_vbus);
    m_bldc.sampleEncoderHandler(m_theta_mach, m_omega_mach);

    // 更新采样电流
    m_bldc.sampleCurrentHandler(m_voltage_shunt, m_tick);
}

void MexFunction::init()
{
    auto& motor = m_flash.motor;
    auto& speed = m_flash.speed.pi;
    auto& current = m_flash.current.foc;

    motor.motor_type = BLDC::kHighCurrent,
    motor.current_controller_type = BLDC::CurrentControllerType::kFOC,
    motor.speed_controller_type = speed::kPI;
    motor.motor_control_mode = int8_t(pica::Motor::ControlMode::kVelocity);
    // motor.motor_control_mode = int8_t(pica::Motor::ControlMode::kPosition);

    motor.pole_pairs        = 5;
    motor.phase_inductance  = 0.5 * 0.64e-3,
    motor.phase_resistance  = 0.5 * 0.57,
    motor.torque_constant   = 0.0591758042f;
    motor.shunt_conductance = 1/50e-3; // 50mR

    // 时间常数
    float Tq = motor.phase_inductance / motor.phase_resistance;
    float Td = motor.phase_inductance / motor.phase_resistance;

    motor.current_controller_bandwidth = 2 * M_PI / fminf(Tq, Td);

    motor.R_wL_FF_enabled = true;
    motor.b_EMF_FF_enabled = true;
}

void MexFunction::input(ArgumentList& inputs)
{
    const TypedArray<double>& clock = inputs[0];
    const matlab::data::TypedArray<double>& angle_mach = inputs[1];
    const matlab::data::TypedArray<double>& omega_mach = inputs[2];
    const matlab::data::TypedArray<double>& voltage_shunt = inputs[3];
    const matlab::data::TypedArray<double>& target = inputs[4];

    auto& motor = *ConfigMgr::GetInstance()->motor();

    // 更新时间
    m_clock = clock[0];
    m_tick = hrt_absnano(m_clock * 1e9);

    // 电流电压
    // m_voltage_shunt[0] = voltage_shunt[0];
    // m_voltage_shunt[1] = voltage_shunt[1];
    // m_voltage_shunt[2] = voltage_shunt[2];
    float scale = 1/motor.shunt_conductance / m_bldc.phaseCurrentRevGain();
    m_voltage_shunt[0] = voltage_shunt[0] * scale;
    m_voltage_shunt[1] = voltage_shunt[1] * scale;
    m_voltage_shunt[2] = voltage_shunt[2] * scale;

    m_voltage_shunt_calibrator[0] = 0.f;
    m_voltage_shunt_calibrator[1] = 0.f;
    m_voltage_shunt_calibrator[2] = 0.f;

    m_vbus = 24;
    m_theta_mach = angle_mach[0];
    m_omega_mach = omega_mach[0];

    // 输入目标值
    m_position = target[0];
    m_speed    = target[1];
    m_torque   = target[2];
}

void MexFunction::output(ArgumentList *op)
{
    ArrayFactory factoryObject;
    ArgumentList& outputs = *op;

    //   [iabc, ialpha_beta, idq, vdq, v_alpha_beta_final, duty_cycle] = mex_picadrive_foc_current_loop(ts, theta_elec, omega_elec, iabc, target);

    float iabc[3];

    m_bldc.currentMeasured().copyTo(iabc);
    auto duty_cycle = m_bldc.dutyCycle();

    auto foc = m_bldc.currentController<FOC>();

    auto& i_alpha_beta_meas  = foc->getCurrentAlphaBetaMeasured();
    auto& idq_meas           = foc->getCurrentDQMeasured();
    auto& vdq                = foc->getVoltageDQFinal();
    auto& vdq_sp             = foc->getVoltageDQSetpoint();
    auto& v_alpha_beta_final = m_bldc.voltageAlphaBetaFinal();

    outputs[0] = factoryObject.createArray(ArrayDimensions{3},
        {
            (double)iabc[0],
            (double)iabc[1],
            (double)iabc[2]
        }
    );
    outputs[1] = factoryObject.createArray(ArrayDimensions{2},
        {
            (double)i_alpha_beta_meas(0),
            (double)i_alpha_beta_meas(1),
        }
    );
    outputs[2] = factoryObject.createArray(ArrayDimensions{2},
        {
            (double)idq_meas(0),
            (double)idq_meas(1),
        }
    );
    outputs[3] = factoryObject.createArray(ArrayDimensions{2},
        {
            (double)foc->getControllerParam().err_prev(0),
            (double)foc->getControllerParam().err_prev(1),
        }
    );
    outputs[4] = factoryObject.createArray(ArrayDimensions{2},
        {
            (double)vdq(0),
            (double)vdq(1),
        }
    );
    outputs[5] = factoryObject.createArray(ArrayDimensions{2},
        {
            (double)v_alpha_beta_final(0),
            (double)v_alpha_beta_final(1),
        }
    );
    outputs[6] = factoryObject.createArray(ArrayDimensions{3},
        {
            (double)duty_cycle[0],
            (double)duty_cycle[1],
            (double)duty_cycle[2],
        }
    );
}
