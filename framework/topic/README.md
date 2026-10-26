# Dust_Zephyr_Architecture_Topic

线程间数据通道。定义消息结构和 zbus/消息队列通道，用于模块间解耦数据交换。

- **imu_to/** — IMU 姿态数据通道
- **remote_to/** — 遥控器数据通道
- **to_can_tx/** — CAN 发送数据通道（基于 k_msgq）

详见 [ARCHITECTURE.md](ARCHITECTURE.md)。
