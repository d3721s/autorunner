#反馈当前电机限制角度/最大速度 (CAN 0x473)
uint8 joint_num    #关节序号1~6
float32 max_angle  #最大角度限制 rad
float32 min_angle  #最小角度限制 rad
float32 max_speed  #最大关节速度 rad/s
