#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
u1_arm 达妙电机总线仿真器 —— 在 vcan0 上模拟 5 个达妙电机.

协议(见《达妙电机上手流程(一)》):
  - 控制帧 ID = CAN_ID + 模式偏移 (MIT +0x000 / 位置速度 +0x100 / 速度 +0x200)
  - 使能 FF..FC / 失能 FF..FD / 保存零点 FF..FE / 清错 FF..FB
  - 位置速度帧: float32 pos + float32 vel (小端)
  - 寄存器: 0x7FF 广播, 0x33 读 / 0x55 写 / 0xAA 存 / 0xCC 读反馈
  - 反馈帧(ID=master_id): D0=ID|(ERR<<4), POS16, VEL12, T12 线性映射, D6/D7 温度

行为模型: 一阶速度限幅趋近目标位置; 收到任何控制/使能/失能/0xCC 帧后回一帧反馈
(与真机"命令触发式反馈"一致)。

用法:
  sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
  python3 fake_motors.py --iface vcan0
"""
import argparse
import socket
import struct
import sys
import threading
import time

# 与 config/u1_arm_driver.yaml 默认一致
MOTORS = [
    # joint, can_id, master_id, p_max, v_max, t_max
    ("j1", 0x01, 0x11, 12.5, 30.0, 10.0),   # DM-J4310
    ("j2", 0x02, 0x12, 12.5, 10.0, 28.0),   # DM-J4340
    ("j3", 0x03, 0x13, 12.5, 30.0, 10.0),   # DM-J4310
    ("j4", 0x04, 0x14, 12.5, 50.0, 5.0),    # DM3507
    ("j5", 0x05, 0x15, 12.5, 50.0, 5.0),    # DM3507
]

CAN_FMT = "<IB3x8s"  # struct can_frame


def float_to_uint(x, vmin, vmax, bits):
    x = min(max(x, vmin), vmax)
    return round((x - vmin) / (vmax - vmin) * ((1 << bits) - 1))


class FakeMotor:
    """单个达妙电机的最小动力学 + 协议应答模型."""

    def __init__(self, name, can_id, master_id, p_max, v_max, t_max):
        self.name = name
        self.can_id = can_id
        self.master_id = master_id
        self.p_max, self.v_max, self.t_max = p_max, v_max, t_max
        self.enabled = False
        self.pos = 0.0          # rad
        self.vel = 0.0          # rad/s
        self.target_pos = 0.0
        self.target_vel_limit = 1.0
        self.err = 0x0          # 高4位状态: 0失能 1使能
        # 忠实复现真机: 电机出厂默认 MIT 模式(1), 只有切到位置速度模式(2)后
        # 才会响应位置速度偏移(0x100)的使能帧。ctrl_mode: 1 MIT/2 位置速度/3 速度/4 力位
        self.ctrl_mode = 1
        self.lock = threading.Lock()

    def step(self, dt):
        """一阶模型: 以 target_vel_limit 限幅逼近 target_pos."""
        with self.lock:
            if not self.enabled:
                self.vel = 0.0
                return
            diff = self.target_pos - self.pos
            vmax = min(abs(self.target_vel_limit), self.v_max) or 0.5
            step = max(-vmax * dt, min(vmax * dt, diff))
            self.pos += step
            self.vel = step / dt if dt > 0 else 0.0

    def feedback_frame(self):
        with self.lock:
            status = 0x1 if self.enabled else 0x0
            p = float_to_uint(self.pos, -self.p_max, self.p_max, 16)
            v = float_to_uint(self.vel, -self.v_max, self.v_max, 12)
            t = float_to_uint(0.0, -self.t_max, self.t_max, 12)
            d = bytes([
                (status << 4) | (self.can_id & 0x0F),
                (p >> 8) & 0xFF, p & 0xFF,
                (v >> 4) & 0xFF,
                ((v & 0x0F) << 4) | ((t >> 8) & 0x0F),
                t & 0xFF,
                40,   # T_MOS ℃
                45,   # T_Rotor ℃
            ])
        return self.master_id, d


class FakeBus:
    def __init__(self, iface, period):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((iface,))
        self.sock.settimeout(0.1)
        self.period = period
        self.motors = {m[1]: FakeMotor(*m) for m in MOTORS}
        self.registers = {}   # (can_id, rid) -> value; 仅回显用

    def send(self, can_id, data):
        frame = struct.pack(CAN_FMT, can_id, len(data), data.ljust(8, b"\x00"))
        try:
            self.sock.send(frame)
        except OSError:
            pass

    def send_feedback(self, motor):
        fid, d = motor.feedback_frame()
        self.send(fid, d)

    def handle(self, can_id, data):
        # 寄存器广播帧
        if can_id == 0x7FF and len(data) >= 4:
            target = data[0] | (data[1] << 8)
            m = self.motors.get(target)
            if m is None:
                return
            cmd, rid = data[2], data[3]
            if cmd == 0xCC:                       # 读电机反馈
                self.send_feedback(m)
            elif cmd == 0x33:                     # 读寄存器 -> 回显
                val = self.registers.get((target, rid), 0)
                self.send(m.master_id, bytes([data[0], data[1], 0x33, rid])
                          + struct.pack("<I", val))
            elif cmd == 0x55:                     # 写寄存器 -> 存并回显
                val = struct.unpack("<I", data[4:8].ljust(4, b"\x00"))[0]
                self.registers[(target, rid)] = val
                if rid == 0x0A:                   # CTRL_MODE
                    with m.lock:
                        m.ctrl_mode = val
                self.send(m.master_id, bytes([data[0], data[1], 0x55, rid])
                          + struct.pack("<I", val))
            elif cmd == 0xAA:                     # 保存 (仅失能有效, 这里直接应答)
                self.send(m.master_id, bytes([data[0], data[1], 0xAA, 0x00]))
            return

        # 控制/特殊命令帧: ID = 模式偏移 + can_id
        base = can_id & 0x0FF
        mode = can_id & 0xF00           # 0x000 MIT / 0x100 位置速度 / 0x200 速度
        m = self.motors.get(base)
        if m is None or mode > 0x300:
            return

        # 模式偏移 -> 对应 ctrl_mode 编码 (0x000->1 MIT, 0x100->2 位置速度, ...)
        mode_to_ctrl = {0x000: 1, 0x100: 2, 0x200: 3, 0x300: 4}

        if len(data) == 8 and data[:7] == b"\xFF" * 7:
            tail = data[7]
            with m.lock:
                if tail == 0xFC:
                    # 忠实复现: 仅当使能帧的模式偏移与电机当前 ctrl_mode 匹配时才使能
                    if mode_to_ctrl.get(mode) == m.ctrl_mode:
                        m.enabled = True
                        m.target_pos = m.pos   # 锁存当前位置, 防跳变
                    # 否则(模式不符)静默忽略, 电机保持失能 —— 即真机"使能不了"的现象
                elif tail == 0xFD:
                    m.enabled = False
                elif tail == 0xFE:      # 保存位置零点
                    m.pos = 0.0
                    m.target_pos = 0.0
                elif tail == 0xFB:      # 清错
                    m.err = 0x1 if m.enabled else 0x0
            self.send_feedback(m)
            return

        if mode == 0x100 and len(data) == 8:      # 位置速度帧
            pos, vel = struct.unpack("<ff", data)
            with m.lock:
                if m.enabled:
                    m.target_pos = min(max(pos, -m.p_max), m.p_max)
                    m.target_vel_limit = vel
            self.send_feedback(m)
        elif mode == 0x000 and len(data) == 8:    # MIT 帧: 解位置(16b)+速度前馈(12b)
            p = (data[0] << 8) | data[1]
            v = (data[2] << 4) | (data[3] >> 4)
            pos = p / 65535.0 * 2 * m.p_max - m.p_max
            vel = v / 4095.0 * 2 * m.v_max - m.v_max
            with m.lock:
                if m.enabled:
                    m.target_pos = pos
                    # 速度上限取前馈幅值(留最小值防止停滞), 使跟踪速度更接近真实
                    m.target_vel_limit = max(abs(vel), 0.5)
            self.send_feedback(m)
        elif mode == 0x200 and len(data) >= 4:    # 速度帧
            (vel,) = struct.unpack("<f", data[:4])
            with m.lock:
                if m.enabled:
                    m.target_pos = m.p_max if vel >= 0 else -m.p_max
                    m.target_vel_limit = vel
            self.send_feedback(m)

    def run(self):
        print(f"[fake_motors] {len(self.motors)} 个达妙电机仿真已启动: "
              + ", ".join(f"{m.name}(id=0x{m.can_id:02X},mst=0x{m.master_id:02X})"
                          for m in self.motors.values()))
        last = time.monotonic()
        while True:
            now = time.monotonic()
            if now - last >= self.period:
                for m in self.motors.values():
                    m.step(now - last)
                last = now
            try:
                frame = self.sock.recv(16)
            except socket.timeout:
                continue
            can_id, dlc, data = struct.unpack(CAN_FMT, frame)
            can_id &= socket.CAN_EFF_MASK
            self.handle(can_id, data[:dlc])


def main():
    ap = argparse.ArgumentParser(description="u1_arm 达妙电机 CAN 总线仿真器")
    ap.add_argument("--iface", default="vcan0", help="CAN 接口 (默认 vcan0)")
    ap.add_argument("--period", type=float, default=0.002, help="动力学步长 s")
    args = ap.parse_args()
    try:
        FakeBus(args.iface, args.period).run()
    except KeyboardInterrupt:
        pass
    except OSError as e:
        print(f"[fake_motors] 打开 {args.iface} 失败: {e}\n"
              f"  sudo modprobe vcan && sudo ip link add dev {args.iface} type vcan "
              f"&& sudo ip link set up {args.iface}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
