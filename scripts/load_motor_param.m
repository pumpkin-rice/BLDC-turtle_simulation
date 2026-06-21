%% 电机参数
% 电机参数来源于 MicroChip 文档 [Hurst DMA0204024B101 BLDC Motor DataSheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Hurst%20DMA0204024B101%20BLDC%20Motor%20DataSheet.pdf)
motor.voltage          = 24; % 工作电压: Vpeak
motor.voltage_constant = 6.2; % 1/Kv = V_{peak}/K_{RPM}, V/krpm
motor.Ke = 0.371; % Ke = V_{peak}/K_{rad/s}, 1/Kv * 6 /1000

motor.pole_pairs       = 5; % 电机极对数
motor.phase_resistance = 0.57/2; % 相电阻, ohm
motor.phase_inductance = 0.64/2 * 1e-3; % 相电感, H@1kHz

motor.psi_f            = 0.00788; % 永磁体磁链(Wb), 这里由 Kt/(1.5*p_n*\psi_f) 得到

motor.torque_constant  = 0.0591758042; % K_t
motor.inertia          = 0.0000177245; % 转动惯量(kg·m^2)
motor.friction_damping_factor = 0.0003; % 摩擦阻尼系数(N·m·s/rad)
% motor.friction_damping_factor = 0.000; % 摩擦阻尼系数(N·m·s/rad)

motor.electrical_time_constant = motor.phase_inductance / motor.phase_resistance; % 电机时间常数
motor.mechanical_time_constant = 2.882e-3; % 机械时间常数，s

%% PWM 参数
%% Set PWM Switching frequency
PWM.frequency 	= 48e3;              % converter s/w freq, Hz
PWM.period      = 1/PWM.frequency;   % PWM switching time period, s
PWM.voltage_ref = 3.3; % PWM 占空比参考电压

%% 电流控制器运行参数
current.ctrl_frequency  = 16e3 % 电流控制器运行频率, Hz
current.ctrl_period     = fix((1/current.ctrl_frequency) * 1e6) / 1e6; % 电流控制器运行周期, s

%% 电流环控制器参数
current.frequency  = 1/motor.electrical_time_constant;
current.band_width = 2 * pi * current.frequency;    % 电流环带宽, rad/s
current.kp         = current.band_width * motor.phase_inductance;
current.ki         = motor.phase_resistance * current.band_width;

current.limit = 7.81;
current.limit_margin = 1.1;

current.torque_constant = 0.04; % Kt: [Nm/A] for PM motors, [Nm/A^2] for induction motors. Equal to 8.27/Kv of the motor

motor.torque_max   = 5; % 电机力矩限幅, Nm
