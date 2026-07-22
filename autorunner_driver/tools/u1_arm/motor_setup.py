#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
u1_arm 达妙电机标定/配置 CLI —— 通过 0x7FF 广播寄存器帧与设零帧操作电机.

支持:
  zero    <can_id>            保存当前位置为零点 (发 模式偏移+ID 的 FF..FE, 仅失能下生效)
  read    <can_id> <rid>      读寄存器 (0x33)
  write   <can_id> <rid> <v>  写寄存器 (0x55, 立即生效, 掉电丢失)
  save    <can_id> <rid>      保存寄存器到 flash (0xAA, 仅失能下生效)
  refresh <can_id>            读电机反馈 (0xCC), 打印 位置/速度/温度/状态
  enable  <can_id>            使能 (位置速度模式)
  disable <can_id>            失能

示例 (先失能再设零):
  python3 motor_setup.py --iface can0 disable 1
  python3 motor_setup.py --iface can0 zero 1
常用寄存器: 0x0A=CTRL_MODE(1 MIT/2 位置速度/3 速度/4 力位), 0x23=CAN 波特率码
"""
import argparse
import socket
import struct
import sys

CAN_FMT = "<IB3x8s"
MODE_POSVEL = 0x100   # 设零/使能默认走位置速度模式偏移


def open_bus(iface):
    s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    s.settimeout(1.0)
    return s


def send(s, can_id, data):
    s.send(struct.pack(CAN_FMT, can_id, len(data), bytes(data).ljust(8, b"\x00")))


def recv(s):
    try:
        f = s.recv(16)
    except socket.timeout:
        return None, None
    cid, dlc, d = struct.unpack(CAN_FMT, f)
    return cid & socket.CAN_EFF_MASK, d[:dlc]


def u2f(u, mn, mx, bits):
    return u / ((1 << bits) - 1) * (mx - mn) + mn


def cmd_refresh(s, can_id, p_max, v_max, t_max):
    send(s, 0x7FF, [can_id & 0xFF, can_id >> 8, 0xCC, 0x00])
    cid, d = recv(s)
    if d is None or len(d) < 8:
        print("无反馈 (检查 CAN ID / 波特率 / 接线)")
        return
    st, mid = d[0] >> 4, d[0] & 0xF
    p = (d[1] << 8) | d[2]
    v = (d[3] << 4) | (d[4] >> 4)
    t = ((d[4] & 0xF) << 8) | d[5]
    print(f"反馈: master=0x{cid:X} id={mid} 状态={st} "
          f"pos={u2f(p,-p_max,p_max,16):.4f}rad vel={u2f(v,-v_max,v_max,12):.3f}rad/s "
          f"tau={u2f(t,-t_max,t_max,12):.3f}Nm MOS={d[6]}C 线圈={d[7]}C")


def main():
    ap = argparse.ArgumentParser(description="u1_arm 达妙电机标定/配置 CLI")
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--p-max", type=float, default=12.5)
    ap.add_argument("--v-max", type=float, default=30.0)
    ap.add_argument("--t-max", type=float, default=10.0)
    ap.add_argument("op", choices=["zero", "read", "write", "save", "refresh",
                                   "enable", "disable"])
    ap.add_argument("args", nargs="*", type=lambda x: int(x, 0))
    a = ap.parse_args()

    try:
        s = open_bus(a.iface)
    except OSError as e:
        print(f"打开 {a.iface} 失败: {e}", file=sys.stderr)
        return 1

    if a.op == "zero":
        cid = a.args[0]
        send(s, MODE_POSVEL + cid, [0xFF] * 7 + [0xFE])
        print(f"已发送设零帧 (电机 {cid}); 请确认电机处于失能状态")
    elif a.op == "enable":
        cid = a.args[0]
        send(s, MODE_POSVEL + cid, [0xFF] * 7 + [0xFC])
        print(f"已发送使能帧 (电机 {cid})")
    elif a.op == "disable":
        cid = a.args[0]
        send(s, MODE_POSVEL + cid, [0xFF] * 7 + [0xFD])
        print(f"已发送失能帧 (电机 {cid})")
    elif a.op == "read":
        cid, rid = a.args[0], a.args[1]
        send(s, 0x7FF, [cid & 0xFF, cid >> 8, 0x33, rid])
        c, d = recv(s)
        if d and len(d) >= 8:
            val = struct.unpack("<I", bytes(d[4:8]))[0]
            print(f"寄存器 0x{rid:02X} = {val} (0x{val:X})")
        else:
            print("读取无应答")
    elif a.op == "write":
        cid, rid, val = a.args[0], a.args[1], a.args[2]
        send(s, 0x7FF, [cid & 0xFF, cid >> 8, 0x55, rid] + list(struct.pack("<I", val)))
        print(f"已写寄存器 0x{rid:02X} = {val} (掉电丢失, 需 save 持久化)")
    elif a.op == "save":
        cid, rid = a.args[0], a.args[1]
        send(s, 0x7FF, [cid & 0xFF, cid >> 8, 0xAA, rid])
        print(f"已发送保存指令 (寄存器 0x{rid:02X}); 仅失能下生效")
    elif a.op == "refresh":
        cmd_refresh(s, a.args[0], a.p_max, a.v_max, a.t_max)
    return 0


if __name__ == "__main__":
    sys.exit(main())
