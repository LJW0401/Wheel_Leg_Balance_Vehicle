# 腿部模型
这部分代码直接搬运了@遥想星空 的开源项目[foc-wheel-legged-robot](https://github.com/Skythinker616/foc-wheel-legged-robot/tree/master/esp32-controller/software/src)中的代码。

### 文件说明

- `main.cpp`：（本项目不使用，但进行参考）主要程序文件，包含所有任务模块的逻辑代码
- `PID.c/h`：PID控制器的实现，包含单级和串级PID控制器
- `debug.c/h`：（本项目不使用）用于[Linkscope](https://gitee.com/skythinker/link-scope)软件实现蓝牙无线调试，不参与正常运行，读者可忽略
- 由MATLAB直接生成的C代码文件，没有可读性，读者可参阅@遥想星空 MATLAB程序说明：
	- `leg_pos.c/h`：腿部位置解算函数
	- `leg_spd.c/h`：腿部运动速度解算函数
	- `leg_conv.c/h`：腿部输出换算函数
	- `lqr_k.c/h`：LQR反馈矩阵计算函数

@遥想星空 MATLAB中的介绍如下

- `leg_func_calc.m`：进行腿部解算和 VMC 映射计算<!--，包含上述步骤1-2-->，导出以下三个M函数：
	- `leg_pos.m`：可由关节电机角度求得腿部姿态
	- `leg_spd.m`：可由关节电机角度速度求得腿部运动速度
	- `leg_conv.m`：可由虚拟腿目标扭矩和推力求得电机所需输出的力矩
- `sys_calc.m`：求得系统状态方程，求得反馈矩阵<!--，包含上述步骤3-4-->，导出下述M函数：
	- `lqr_k.m`：代入腿长 L0 后返回该腿长对应的反馈矩阵 K