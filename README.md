# picadrive-simulink

基于 Matlab 和 Simulink 的 BLDC(PMSM) 仿真入门工程集。

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
