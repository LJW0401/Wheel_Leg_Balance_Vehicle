# 参考资料
## 控制系统
控制系统设计参考了[RoboMaster平衡步兵机器人控制系统设计](https://zhuanlan.zhihu.com/p/563048952)

## 工程代码
工程代码则参考了[无刷轮腿平衡机器人开源GitHub仓库](https://github.com/Skythinker616/foc-wheel-legged-robot/tree/master)，并借鉴其中的仿真功能在上车之前先测试效果。

## 其他参考
在CAN线控制方面参考了如下文章：
- [关于RoboMaster 电机CAN通信](https://sourcelizi.github.io/201912/robomaster-motor-can/)
- [关于STM32 CAN 发送失败或丢数据问题](https://juejin.cn/post/7207973807776350263)


为了实现对小米电机的控制，我们参考了2个开源的驱动程序：
- [小米微电机STM32 HAL库驱动教程](https://blog.csdn.net/m0_53802226/article/details/132941275)
- [小米电机CyberGear STM32HAL 使用指南](https://blog.csdn.net/zdYukino/article/details/133505453)

查阅西工大参考文献：
- [Ascento: A Two-Wheeled Jumping Robot](https://ieeexplore.ieee.org/document/8793792)
- [LQR-Assisted Whole-Body Control of a Wheeled Bipedal Robot With Kinematic Loops](https://ieeexplore.ieee.org/document/9028180)