# -R2


<<<<<<< HEAD
=======
store the history code for R2


>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
## v0.1

  保存了pid调整后的版本1代码

## v0.11

  修正了角度计算的逻辑错误与机器人坐标系解算的问题
  注：action归零命令为 *ACT0*

## v0.12

  尝试了自带的角度纠正pid，效果一般

## v1.1

  开始用cpp重构H743IIt6，并完成task和namespace的框架

## v1.2

  加入了Base_Calculation，部分全局变量需要等待相应的文件完成再改（目前注释掉了相关代码）

## v1.3

加入了Motor和回调函数与外设类管理，全局变量进行了部分补充（待完成通讯类和加入雷达监测后的跟随底盘解算）

- [X] Motor

## v2.0

解决了基本系统框架初始化问题与一些变量运算问题，完整功能实现，但是串口回调函数无法使用，仍在改进

- [X] Base_Calculation
- [X] variableInitalization

## v2.1

解决了串口接收和手柄问题，但是action数据有点奇怪，can还没解决

- [X] Callback_Function
- [X] Xbox_Map

## v2.2

- [X] CAN
- [X] Action

## v2.3

<<<<<<< HEAD
- [ ] pid_adjust
- [ ] Auto_trace
=======
- [X] Auto_trace

## v3.0

- [X] Migrate to Jerry'sm architecture and add filters
>>>>>>> 944f7e49b9ca7249e370900b25af451d08e604c0
