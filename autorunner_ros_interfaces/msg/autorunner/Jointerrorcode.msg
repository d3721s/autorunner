#关节驱动器低速反馈·驱动器状态位 (CAN 0x261~0x266 byte5)
uint8[6] joint_error  #驱动器状态字节 bit0电压过低 bit1电机过温 bit2驱动器过流 bit3驱动器过温 bit4碰撞保护 bit5驱动器错误 bit6使能状态 bit7堵转保护
