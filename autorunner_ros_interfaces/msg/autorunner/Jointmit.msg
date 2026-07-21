#关节MIT控制 (CAN 0x15A~0x15F) 本期仅定义 编码实现待Piper SDK确认量化范围
uint8 joint_num  #关节序号1~6
float32 pos_ref  #位置参考 rad
float32 vel_ref  #速度参考 rad/s
float32 kp       #Kp给定参考值10
float32 kd       #Kd给定参考值0.8
float32 t_ref    #力矩参考 N·m
