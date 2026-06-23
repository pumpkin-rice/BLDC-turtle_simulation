# picadrive-simulink

基于 Matlab 和 Simulink 的 BLDC(PMSM) 仿真入门工程集，用于记录 PMSM 驱动仿真学习过程。

此项目同时是 [pica-drive](https://github.com/pumpkin-rice/pica-drive) 仿真测试用例。

# slx 文件命名说明
从 FOC 闭环仿真开始，工程内各个文件按照 `mcb_<motor>_<controller>_<sensor>_[other]` 方式命名，各字段说明如下：
- mcb: 模型控制块
- motor: 电机类型，如 pmsm、bldc、acim 等，专有名词可大写
- controller: 控制器类型，如 FOC
- sensor: 角度传感器类型，如 abs、abz、hall、sensorless 等
- other: 其他说明性文本，如引入 mex 或 slrt 时，应分别备注 mex/slrt

# 版本说明

版本命名：
版本发布时，以 `<motor>_<controller>_[other]-<ver>` 进行命名，各字段说明如下：
- motor: 电机类型，如 pmsm、bldc、acim 等，专有名词可大写
- controller: 控制器类型，如 FOC
- other: 其他说明性文本，如引入 mex 时应标注 mex；模型变更为 slrt 时应后缀 slrt
- ver: 版本号，每个 `<motor>_<controller>_[other]` 均对应一个版本号，其编号应为 `v<maj>.<min>` 格式：
	1. maj: 标识大版本，如仿真模型完成后，标记为 `v1.0`，适用于大版本标记的内容如下：
	    - 更改模型中环路控制器应按大版本标记
	2. min: patch，表示对模型进行小幅修改。适用范围如下：
	    - 模型 bug 修复（bugfix）
        - 模块参数更新：如更改 PID 参数等

## pmsm_foc_current-loop-PI_v1.0

1. 完成基于 PI 控制器的电流环仿真，模型文件为 `mcb_pmsm_foc_abs_current_loop.slx`

- 参考资料：
1. 袁雷等老师编写的[现代永磁同步电机控制原理及MATLAB仿真](https://baike.baidu.com/item/%E7%8E%B0%E4%BB%A3%E6%B0%B8%E7%A3%81%E5%90%8C%E6%AD%A5%E7%94%B5%E6%9C%BA%E6%8E%A7%E5%88%B6%E5%8E%9F%E7%90%86%E5%8F%8AMATLAB%E4%BB%BF%E7%9C%9F/57866982)：基于 PI 调节器的 PMSM 矢量控制章节
2. matlab 示例工程 [使用霍尔传感器的 PMSM 的磁场定向控制](https://ww2.mathworks.cn/help/mcb/gs/foc-pmsm-using-hall-sensor-example.html)
3. 电机参数来源于 MicroChip 文档 [Hurst DMA0204024B101 BLDC Motor DataSheet](https://ww1.microchip.com/downloads/en/DeviceDoc/Hurst%20DMA0204024B101%20BLDC%20Motor%20DataSheet.pdf)

## pmsm_foc_speed-loop-PI_v1.0

1. 基于 pmsm_foc_current-loop-PI_v1.0 模型，完成基于 PI 控制器的速度环仿真，模型文件为 `mcb_pmsm_foc_abs_speed_loop.slx`

- 参考资料：
同 pmsm_foc_current-loop-PI_v1.0。

## pmsm_foc_speed-loop-PI_pica-drive_v1.0

1. 基于 mex 的 pica-drive 仿真模型，模型文件为 `mcb_pmsm_foc_abs_current_loop_mex.slx`

- 参考资料：
Ti InstaSPIN-FOC 和 InstaSPIN-MOTION 用户手册: https://www.ti.com.cn/cn/lit/ug/zhcu083i/zhcu083i.pdf
