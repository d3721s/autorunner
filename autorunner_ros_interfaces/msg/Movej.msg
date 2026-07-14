#关节运动命令 (CAN 0x155/0x156/0x157 + 0x151 MOVE J)
float32[6] joint  #目标关节角 rad 顺序joint1~joint6
uint8 speed       #运动速度百分比1~100
