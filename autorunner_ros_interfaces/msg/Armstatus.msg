#机械臂状态反馈 (CAN 0x2A1)
uint8 ctrl_mode        #控制模式 0待机 1CAN指令控制 2示教 3以太网 4wifi 5遥控器 6联动示教输入 7离线轨迹
uint8 arm_status       #机械臂状态 0正常 1急停 2无解 3奇异点 4目标角度超限 5关节通信异常 6抱闸未打开 7碰撞 8示教超速 9关节状态异常 10其它异常 11示教记录 12示教执行 13示教暂停 14主控NTC过温 15释放电阻NTC过温
uint8 move_mode        #模式反馈 0:MOVE P 1:MOVE J 2:MOVE L 3:MOVE C 4:MOVE M
uint8 teach_status     #示教状态 0关闭 1开始记录 2结束记录 3执行 4暂停 5继续 6终止 7运动到起点
uint8 motion_status    #运动状态 0到达指定点位 1未到达
uint8 trajectory_num   #当前运行轨迹点序号0~255(离线轨迹模式下反馈)
uint8 joint_comm_err   #byte7 bit0~5: J1~J6通信异常位 (0正常 1异常)
uint8 joint_angle_limit_err #byte6 bit0~5: J1~J6角度超限位 (0正常 1异常)
