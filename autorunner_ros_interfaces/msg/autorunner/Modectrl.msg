#模式控制直发 (CAN 0x151) 高级用途 日常由驱动在Movex命令内自动发送
uint8 ctrl_mode    #控制模式 0待机 1CAN指令控制 7离线轨迹
uint8 move_mode    #MOVE模式 0:MOVE P 1:MOVE J 2:MOVE L 3:MOVE C 4:MOVE M
uint8 speed        #运动速度百分比0~100
uint8 mit_mode     #0x00位置速度模式(默认) 0xAD:MIT模式(用于主从模式)
uint8 install_pos  #安装位置 0无效 1水平正装 2侧装左 3侧装右
