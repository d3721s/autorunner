#关节驱动器低速反馈·电压/母线电流 (CAN 0x261~0x266)
float32[6] voltage      #驱动器电压 V
float32[6] bus_current  #母线电流 A (1.5kg关节电机无母线电流采样 恒为0)
