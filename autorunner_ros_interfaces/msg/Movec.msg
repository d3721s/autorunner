#圆弧运动命令 (CAN (0x152~0x154+0x158)x3 + 0x151 MOVE C) 起点为当前末端位姿
Armpose pose_mid  #圆弧中点
Armpose pose_end  #圆弧终点
uint8 speed       #运动速度百分比1~100
