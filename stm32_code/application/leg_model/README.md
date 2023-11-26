# 腿部模型
这部分代码借鉴了[@遥想星空](https://github.com/Skythinker616) 的开源项目[foc-wheel-legged-robot](https://github.com/Skythinker616/foc-wheel-legged-robot/tree/master/esp32-controller/software/src)中的代码。

### 文件说明

- `PID.c/h`：PID控制器的实现，包含单级和串级PID控制器
- 由MATLAB直接生成的C代码文件，没有可读性，参阅[MATLAB程序说明](../../../matlab/README.md)：
	- `LegPos.c/h`：腿部位置解算函数
	- `JointPos.c/h`：关节电机位置解算函数。
	- `LegSpd.c/h`：腿部运动速度解算函数
	- `LegConv.c/h`：腿部输出换算函数
	- `LQR_K.c/h`：LQR反馈矩阵计算函数
