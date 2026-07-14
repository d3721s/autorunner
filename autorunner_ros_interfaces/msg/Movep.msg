#末端位姿运动命令 movep与movel共用 (CAN 0x152/0x153/0x154 + 0x151 MOVE P/L)
Armpose pose  #目标末端位姿 欧拉表示
uint8 speed   #运动速度百分比1~100
