# -R2

store the history code for R2

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

 - [ ] Motor
 - [x] CallbackFunction