#关节设置 (CAN 0x475)
uint8 joint_num  #关节序号1~6 7代表全部关节电机
bool set_zero    #置当前位置为零点
bool clear_err   #清除关节错误代码
float32 max_acc  #最大关节加速度 rad/s^2 NaN表示不设置
