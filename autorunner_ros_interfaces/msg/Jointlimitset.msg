#电机角度限制/最大速度设置 (CAN 0x474)
uint8 joint_num    #关节序号1~6
float32 max_angle  #最大角度限制 rad NaN不修改
float32 min_angle  #最小角度限制 rad NaN不修改
float32 max_speed  #最大关节速度 rad/s NaN不修改
