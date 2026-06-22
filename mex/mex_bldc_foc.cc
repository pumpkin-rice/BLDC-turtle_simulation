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

#include "bldc.h"

using namespace matlab::data;
using matlab::mex::ArgumentList;

using ofstream = std::ofstream;

class MexFunction : public matlab::mex::Function {
public:
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

		run();

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
	double m_ts_last{0};

	struct bldc m_bldc;
	struct motor_parameters m_motor_param;

	ofstream m_csv;
};

void MexFunction::run()
{
	double ts_diff = m_ts_now - m_ts_last;

	// 更新传感器参数
    bldc_sample_voltage_handler(&m_bldc, NULL, m_vbus);
    bldc_sample_encoder_handler(&m_bldc, m_theta_mach, m_omega_mach);

    // 更新采样电流
    bldc_sample_current_handler(&m_bldc, m_voltage_shunt);

    bldc_do_checks(&m_bldc);

    // 更新电机控制环参数
    bldc_set_torque(&m_bldc, m_torque);

    bldc_update(&m_bldc);

    // 更新校正电流
    bldc_sample_current_calibrator_handler(&m_bldc, NULL, ts_diff);
    
    // 运行电流环
    bldc_run(&m_bldc, ts_diff, ts_diff);

	m_ts_last = m_ts_now;
}

void MexFunction::init()
{
	motor_init_param_by_default(&m_motor_param);
	
	m_motor_param.type = MOTOR_TYPE_HIGH_CURRENT,
	m_motor_param.ctrl_type = MOTOR_CTRL_TYPE_FOC,

	m_motor_param.pole_pairs        = 5;
	m_motor_param.phase_inductance  = 0.5 * 0.64e-3,
	m_motor_param.phase_resistance  = 0.5 * 0.57,
	m_motor_param.torque_constant   = 0.0591758042f;
	m_motor_param.shunt_conductance = 1/50e-3; // 50mR

	struct motor_parameters *params = &m_motor_param;

	// 时间常数
	float Tq = params->phase_inductance / params->phase_resistance;
	float Td = params->phase_inductance / params->phase_resistance;

	params->current_controller_bandwidth = 2 * M_PI / fminf(Tq, Td);

	bldc_init(&m_bldc, &m_motor_param);
}

void MexFunction::sample_input(ArgumentList& inputs)
{
	const TypedArray<double>& clock = inputs[0];
	const matlab::data::TypedArray<double>& angle_mach = inputs[1];
	const matlab::data::TypedArray<double>& omega_mach = inputs[2];
	const matlab::data::TypedArray<double>& voltage_shunt = inputs[3];
	const matlab::data::TypedArray<double>& target = inputs[4];


	// 更新时间
	m_ts_last = m_ts_now;
	m_ts_now = clock[0];

	// 电流电压
	// m_voltage_shunt[0] = voltage_shunt[0];
	// m_voltage_shunt[1] = voltage_shunt[1];
	// m_voltage_shunt[2] = voltage_shunt[2];
	m_voltage_shunt[0] = (voltage_shunt[0] / m_motor_param.shunt_conductance) / m_bldc.phase_current_rev_gain;
	m_voltage_shunt[1] = (voltage_shunt[1] / m_motor_param.shunt_conductance) / m_bldc.phase_current_rev_gain;
	m_voltage_shunt[2] = (voltage_shunt[2] / m_motor_param.shunt_conductance) / m_bldc.phase_current_rev_gain;

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

	struct foc *foc = bldc_get_current_controller_foc(&m_bldc);
    
    const float *iabc = bldc_get_current_phase_meas(&m_bldc);
    const float *duty_cycle = bldc_get_duty_cycle(&m_bldc);

    const float *i_alpha_beta_meas = foc_dbg_i_alpha_beta_measured(foc);
    const float *idq_meas = foc_dbg_idq_meas(foc);
    const float *vdq = foc_dbg_vdq(foc);
    const float *v_alpha_beta_final = foc_dbg_v_alpha_beta_final(foc);

	outputs[0] = factoryObject.createArray(ArrayDimensions{3},
		{
			(double)iabc[0],
			(double)iabc[1],
			(double)iabc[2]
		}
	);
	outputs[1] = factoryObject.createArray(ArrayDimensions{2},
		{
			(double)i_alpha_beta_meas[0],
			(double)i_alpha_beta_meas[1]
		}
	);
	outputs[2] = factoryObject.createArray(ArrayDimensions{2},
		{
			(double)idq_meas[0],
			(double)idq_meas[1],
		}
	);
	outputs[3] = factoryObject.createArray(ArrayDimensions{2},
		{
			(double)vdq[0],
			(double)vdq[1],
		}
	);
	outputs[4] = factoryObject.createArray(ArrayDimensions{2},
		{
			(double)v_alpha_beta_final[0],
			(double)v_alpha_beta_final[1],
		}
	);
	outputs[5] = factoryObject.createArray(ArrayDimensions{3},
		{
			(double)duty_cycle[0],
			(double)duty_cycle[1],
			(double)duty_cycle[2],
		}
	);

}
