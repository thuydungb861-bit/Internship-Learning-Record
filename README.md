# Internship-Learning-Record
Internship Learning Record

IMU：使用HTW906P jetson orin NX 驱动
维特 IMU 的发包是有固定顺序的：通常是先发加速度，再发角速度，最后发角度/四元数。
编写逻辑：(1)HWT906P 的 Python 协议解析器：输入一串串口字节，按 11 字节帧切包、校验、识别帧类型，再把加速度、角速度、欧拉角/四元数、磁场等内容组装成 ImuSample 输出
(2)HWT906P ROS 2 Python 驱动节点:串口读取 HWT906P 数据(一次性4096个字节)，自动尝试波特率，解析出 IMU/磁力计数据，然后发布到 ROS 2 话题。
