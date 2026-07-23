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

#include "mex.hpp"
#include "mexAdapter.hpp"

#define ENABLE_PICA_DRIVE_DEBUG 1

#include "drive_conf.h"
#include "motor/bldc.hpp"

using namespace matlab::data;
using matlab::mex::ArgumentList;

using ofstream = std::ofstream;

class MexFunction : public matlab::mex::Function {
public:
    using BLDC = pica::motor::BLDC;

    MexFunction()
    {
        init();
    }
    ~MexFunction()
    {
        
    }

    void operator()(ArgumentList outputs, ArgumentList inputs) {
        // update input args
        // checkArguments(outputs, inputs);

        sample_input(inputs);

        if (m_counting_up) {
            m_counting_up = false;
            IRQHandler();
        
        } else {
            m_counting_up = true;
            run();
        }

        fresh_output(&outputs);
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
    void sample_input(ArgumentList& inputs);

    void IRQHandler();

    void run();

    void fresh_output(ArgumentList *outputs);

    void log();

    double diff_timestamp(const struct timespec *start, const struct timespec *end)
    {
        return (end->tv_sec - start->tv_sec) + (end->tv_nsec - start->tv_nsec) * 1e-9;
    }

    int getTimestamp(struct timespec *ts)
    {
        struct timeval tv;

        if (gettimeofday(&tv, NULL) < 0) {
            return -1;
        }

        ts->tv_sec = tv.tv_sec;
        ts->tv_nsec = tv.tv_usec * 1000;

        return 0;
    }

private:
    float m_voltage_shunt[3], m_ibus, m_voltage_shunt_calibrator[3];
    float m_voltage_phase[3], m_vbus;
    float m_duty_cycles[3];

    float m_theta_mach; /*!< 电机机械角度, rad */
    float m_omega_mach; /*!< 电机机械角速度, rad/s */

    float m_torque{0};
    float m_speed{0};
    float m_position{0};

    double m_ts_now{0};
    double m_ts_sample{0};

    pica::motor::BLDC m_bldc;
    pica::motor::Config m_cfg;

    bool m_counting_up{true}; /*!< 模拟当前 PWM 计数方向: true - 向上, false-向下*/

    ofstream m_csv;
};

void MexFunction::run()
{
    double ts_diff = m_ts_now - m_ts_sample;

    m_bldc.do_checks();
    
    // 更新电机控制环参数
    m_bldc.setTorque(m_torque);
    m_bldc.setPosition(m_position);
    m_bldc.setVelocity(m_speed);

    m_bldc.update((hrt_absnano(m_ts_now * 1e9)) + 45);

    // 更新校正电流
    m_bldc.sampleCurrentCalibratorHandler(NULL, PICA_DRIVE_CURRENT_MEASURE_PERIOD);
    
    // 运行电流环
    m_bldc.run((hrt_absnano(m_ts_now * 1e9))+120);
}

void MexFunction::IRQHandler()
{
    // 更新传感器参数
    m_bldc.sampleBusVoltageHandler(m_vbus);
    m_bldc.sampleEncoderHandler(m_theta_mach, m_omega_mach);

    // 更新采样电流
    m_bldc.sampleCurrentHandler(m_voltage_shunt, (hrt_absnano(m_ts_now * 1e9)));
    
    m_ts_sample = m_ts_now;
}

void MexFunction::init()
{
    m_cfg.initDefaultValue();
    
    m_cfg.motor_type = BLDC::HIGH_CURRENT,
    m_cfg.current_controller_type = BLDC::CurrentControllerType::FieldOrientedControl,

    m_cfg.pole_pairs        = 5;
    m_cfg.phase_inductance  = 0.5 * 0.64e-3,
    m_cfg.phase_resistance  = 0.5 * 0.57,
    m_cfg.torque_constant   = 0.0591758042f;
    m_cfg.shunt_conductance = 1/50e-3; // 50mR
    m_cfg.current_limit     = 7.81;
    m_cfg.inertia = 0.0000177245;

    // 时间常数
    float Tq = m_cfg.phase_inductance / m_cfg.phase_resistance;
    float Td = m_cfg.phase_inductance / m_cfg.phase_resistance;

    m_cfg.current_controller_bandwidth = 2 * M_PI / fminf(Tq, Td);

    m_cfg.R_wL_FF_enabled = m_cfg.b_EMF_FF_enabled = true;

    auto& speed_cfg = m_cfg.speed_controller_cfg;
    float speed_bw = 3871/60.f * 2*M_PI; // 速度环带宽：4000 rpm

    speed_cfg.control_mode = pica::motor::SpeedController::kVelocity;
    // speed_cfg.control_mode = pica::motor::SpeedController::kPosition;
    speed_cfg.pi.pos_gain = 60.f;
    speed_cfg.pi.vel_gain = (speed_bw * m_cfg.inertia) / m_cfg.torque_constant;
    speed_cfg.pi.vel_integrator_gain = speed_bw * speed_cfg.pi.vel_gain;

    m_bldc.init(&m_cfg);
}

void MexFunction::sample_input(ArgumentList& inputs)
{
    const TypedArray<double>& clock = inputs[0];
    const matlab::data::TypedArray<double>& angle_mach = inputs[1];
    const matlab::data::TypedArray<double>& omega_mach = inputs[2];
    const matlab::data::TypedArray<double>& voltage_shunt = inputs[3];
    const matlab::data::TypedArray<double>& target = inputs[4];


    // 更新时间
    m_ts_now = clock[0];

    // 电流电压
    // m_voltage_shunt[0] = voltage_shunt[0];
    // m_voltage_shunt[1] = voltage_shunt[1];
    // m_voltage_shunt[2] = voltage_shunt[2];
    m_voltage_shunt[0] = (voltage_shunt[0] / m_cfg.shunt_conductance) / m_bldc.getPhaseCurrentRevGain();
    m_voltage_shunt[1] = (voltage_shunt[1] / m_cfg.shunt_conductance) / m_bldc.getPhaseCurrentRevGain();
    m_voltage_shunt[2] = (voltage_shunt[2] / m_cfg.shunt_conductance) / m_bldc.getPhaseCurrentRevGain();

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

void MexFunction::fresh_output(ArgumentList *op)
{
    ArrayFactory factoryObject;
    ArgumentList& outputs = *op;

    //   [iabc, ialpha_beta, idq, vdq, v_alpha_beta_final, duty_cycle] = mex_picadrive_foc_current_loop(ts, theta_elec, omega_elec, iabc, target);

    const float *iabc = m_bldc.getPhaseCurrentMeasured();
    const float *duty_cycle = m_bldc.getDutyCycles();

    auto foc = m_bldc.getCurrentController<pica::motor::FOC>();

    auto& i_alpha_beta_meas  = foc->getCurrentAlphaBetaMeasured();
    auto& idq_meas           = foc->getCurrentDQMeasured();
    auto& vdq                = foc->getVoltageDQFinal();
    auto& vdq_sp             = foc->getVoltageDQSetpoint();
    auto& v_alpha_beta_final = foc->getVoltageAlphaBetaFinal();

    outputs[0] = factoryObject.createArray(ArrayDimensions{3},
        {
            (double)iabc[0],
            (double)iabc[1],
            (double)iabc[2]
        }
    );
    outputs[1] = factoryObject.createArray(ArrayDimensions{2},
        {
            (double)i_alpha_beta_meas.alpha,
            (double)i_alpha_beta_meas.beta,
        }
    );
    outputs[2] = factoryObject.createArray(ArrayDimensions{2},
        {
            (double)idq_meas.d,
            (double)idq_meas.q,
        }
    );
    outputs[3] = factoryObject.createArray(ArrayDimensions{2},
        {
            (double)foc->getCurrentControllerD().err_prev,
            (double)foc->getCurrentControllerQ().err_prev,
        }
    );
    outputs[4] = factoryObject.createArray(ArrayDimensions{2},
        {
            (double)vdq.d,
            (double)vdq.q,
        }
    );
    outputs[5] = factoryObject.createArray(ArrayDimensions{2},
        {
            (double)v_alpha_beta_final.alpha,
            (double)v_alpha_beta_final.beta,
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
