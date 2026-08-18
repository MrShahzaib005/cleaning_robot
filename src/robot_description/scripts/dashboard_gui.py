#!/usr/bin/env python3
"""
Cleaning Robot Dashboard GUI — 5 Patterns
1.Wall Follow  2.Spiral  3.Zigzag  4.Random  5.Targeted
Auto/Manual floor detection + Vacuum ON/OFF
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
import time, sys

try:
    import tkinter as tk
    from tkinter import font as tkfont
except ImportError:
    print("ERROR: tkinter not installed! Run: sudo apt install python3-tk -y")
    sys.exit(1)


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_gui')
        self.create_subscription(String, '/floor_type', self.floor_cb, 10)
        self.create_subscription(String, '/vacuum/speed', self.vac_speed_cb, 10)
        self.create_subscription(String, '/vacuum/status', self.vac_status_cb, 10)
        self.create_subscription(String, '/cleaning/status', self.clean_cb, 10)
        self.create_subscription(String, '/floor_type/confidence', self.conf_cb, 10)
        self.create_subscription(String, '/floor_type/current_mode', self.mode_cb, 10)

        self.pattern_pub = self.create_publisher(String, '/cleaning/select', 10)
        self.cancel_pub = self.create_publisher(String, '/cleaning/cancel', 10)
        self.vacuum_ctrl_pub = self.create_publisher(String, '/vacuum/control', 10)
        self.floor_mode_pub = self.create_publisher(String, '/floor_type/mode', 10)
        self.floor_manual_pub = self.create_publisher(String, '/floor_type/manual_set', 10)

        self.floor_type = 'waiting...'
        self.vac_speed = '0%'
        self.vac_status = '---'
        self.clean_status = 'IDLE'
        self.confidence = '---'
        self.floor_mode = 'AUTO'
        self.logs = []

    def floor_cb(self, msg): self.floor_type = msg.data
    def vac_speed_cb(self, msg): self.vac_speed = msg.data
    def vac_status_cb(self, msg): self.vac_status = msg.data
    def clean_cb(self, msg):
        self.clean_status = msg.data
        self.logs.append(f'[{time.strftime("%H:%M:%S")}] {msg.data}')
        if len(self.logs) > 30: self.logs.pop(0)
    def conf_cb(self, msg): self.confidence = msg.data
    def mode_cb(self, msg): self.floor_mode = msg.data

    def pub_pattern(self, p):
        m = String(); m.data = p; self.pattern_pub.publish(m)
        self.logs.append(f'[{time.strftime("%H:%M:%S")}] Pattern: {p}')
    def pub_cancel(self):
        m = String(); m.data = 'stop'; self.cancel_pub.publish(m)
        self.logs.append(f'[{time.strftime("%H:%M:%S")}] CANCELLED')
    def pub_vacuum(self, c):
        m = String(); m.data = c; self.vacuum_ctrl_pub.publish(m)
    def pub_floor_mode(self, mode):
        m = String(); m.data = mode; self.floor_mode_pub.publish(m)
    def pub_manual_floor(self, floor):
        m = String(); m.data = floor; self.floor_manual_pub.publish(m)


BG='#0d1117'; CARD='#161b22'; BORDER='#30363d'; TEXT='#e6edf3'
DIM='#7d8590'; ACCENT='#58a6ff'; GREEN='#3fb950'; ORANGE='#d29922'
RED='#f85149'; PURPLE='#bc8cff'; CYAN='#39d2c0'; TEAL='#2dd4bf'

def floor_color(f):
    return {'tiles':'#607D8B','wood':'#8D6E63','carpet':RED}.get(f.lower(), DIM)
def floor_icon(f):
    return {'tiles':'⬜','wood':'🟫','carpet':'🟥'}.get(f.lower(), '❓')
def speed_int(s):
    try: return int(s.replace('%',''))
    except: return 0
def speed_color(s):
    v = speed_int(s)
    if v <= 35: return GREEN
    elif v <= 70: return ORANGE
    return RED


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    root = tk.Tk()
    root.title('Cleaning Robot — Control Panel')
    root.geometry('420x780')
    root.configure(bg=BG)
    root.resizable(False, False)

    fn_lbl = tkfont.Font(family='Helvetica', size=9)
    fn_val = tkfont.Font(family='Courier', size=18, weight='bold')
    fn_btn = tkfont.Font(family='Helvetica', size=9, weight='bold')
    fn_sm = tkfont.Font(family='Courier', size=8)
    fn_hd = tkfont.Font(family='Helvetica', size=10, weight='bold')

    def card(parent):
        return tk.Frame(parent, bg=CARD, highlightbackground=BORDER,
                        highlightthickness=1, padx=10, pady=8)

    # HEADER
    hdr = tk.Frame(root, bg='#1a2233', pady=6)
    hdr.pack(fill='x')
    tk.Label(hdr, text='  CLEANING ROBOT', font=fn_hd, fg=ACCENT, bg='#1a2233').pack(side='left', padx=8)

    # FLOOR + VACUUM ROW
    row1 = tk.Frame(root, bg=BG)
    row1.pack(fill='x', padx=6, pady=(6,3))

    cf = card(row1)
    cf.pack(side='left', fill='both', expand=True, padx=(0,3))
    tk.Label(cf, text='FLOOR TYPE', font=fn_lbl, fg=DIM, bg=CARD).pack(anchor='w')
    fl_icon = tk.Label(cf, text='❓', font=tkfont.Font(size=22), bg=CARD)
    fl_icon.pack(anchor='w')
    fl_val = tk.Label(cf, text='---', font=fn_val, fg=TEXT, bg=CARD)
    fl_val.pack(anchor='w')
    fl_conf = tk.Label(cf, text='', font=fn_sm, fg=DIM, bg=CARD)
    fl_conf.pack(anchor='w')
    fl_bar = tk.Frame(cf, bg=DIM, height=3)
    fl_bar.pack(fill='x', pady=(4,0))

    cv = card(row1)
    cv.pack(side='left', fill='both', expand=True, padx=(3,0))
    tk.Label(cv, text='VACUUM SPEED', font=fn_lbl, fg=DIM, bg=CARD).pack(anchor='w')
    vc_val = tk.Label(cv, text='0%', font=fn_val, fg=GREEN, bg=CARD)
    vc_val.pack(anchor='w', pady=(8,0))
    bar_bg = tk.Frame(cv, bg='#21262d', height=12)
    bar_bg.pack(fill='x', pady=(6,0))
    bar_bg.pack_propagate(False)
    bar_fl = tk.Frame(bar_bg, bg=GREEN, height=12)
    bar_fl.place(x=0, y=0, relheight=1.0, relwidth=0.0)
    vr = tk.Frame(cv, bg=CARD)
    vr.pack(fill='x', pady=(6,0))
    tk.Button(vr, text='ON', font=fn_btn, fg='white', bg=GREEN, relief='flat',
              padx=12, pady=2, command=lambda: node.pub_vacuum('on')).pack(side='left', padx=(0,4))
    tk.Button(vr, text='OFF', font=fn_btn, fg='white', bg='#6e7681', relief='flat',
              padx=12, pady=2, command=lambda: node.pub_vacuum('off')).pack(side='left')

    # FLOOR DETECTION MODE
    cm = card(root)
    cm.pack(fill='x', padx=6, pady=3)
    mode_header = tk.Frame(cm, bg=CARD)
    mode_header.pack(fill='x')
    tk.Label(mode_header, text='FLOOR DETECTION', font=fn_lbl, fg=DIM, bg=CARD).pack(side='left')
    mode_indicator = tk.Label(mode_header, text='AUTO', font=fn_btn, fg=GREEN, bg=CARD)
    mode_indicator.pack(side='right')

    mode_btns = tk.Frame(cm, bg=CARD)
    mode_btns.pack(fill='x', pady=(4,0))
    mode_btns.columnconfigure(0, weight=1); mode_btns.columnconfigure(1, weight=1)
    tk.Button(mode_btns, text='AUTO', font=fn_btn, fg='white', bg=GREEN,
              relief='flat', pady=3, command=lambda: node.pub_floor_mode('auto')
              ).grid(row=0, column=0, padx=(0,3), sticky='ew')
    tk.Button(mode_btns, text='MANUAL', font=fn_btn, fg='white', bg=ORANGE,
              relief='flat', pady=3, command=lambda: node.pub_floor_mode('manual')
              ).grid(row=0, column=1, padx=(3,0), sticky='ew')

    manual_frame = tk.Frame(cm, bg=CARD)
    manual_frame.pack(fill='x', pady=(4,0))
    manual_frame.columnconfigure(0, weight=1)
    manual_frame.columnconfigure(1, weight=1)
    manual_frame.columnconfigure(2, weight=1)
    tk.Button(manual_frame, text='Tiles', font=fn_btn, fg='white', bg='#607D8B',
              relief='flat', pady=4, command=lambda: node.pub_manual_floor('tiles')
              ).grid(row=0, column=0, padx=2, sticky='ew')
    tk.Button(manual_frame, text='Wood', font=fn_btn, fg='white', bg='#8D6E63',
              relief='flat', pady=4, command=lambda: node.pub_manual_floor('wood')
              ).grid(row=0, column=1, padx=2, sticky='ew')
    tk.Button(manual_frame, text='Carpet', font=fn_btn, fg='white', bg=RED,
              relief='flat', pady=4, command=lambda: node.pub_manual_floor('carpet')
              ).grid(row=0, column=2, padx=2, sticky='ew')

    # CLEANING STATUS
    cp = card(root)
    cp.pack(fill='x', padx=6, pady=3)
    tk.Label(cp, text='STATUS', font=fn_lbl, fg=DIM, bg=CARD).pack(anchor='w')
    cl_val = tk.Label(cp, text='IDLE', font=fn_hd, fg=CYAN, bg=CARD)
    cl_val.pack(anchor='w', pady=2)

    # PATTERN BUTTONS — 5 PATTERNS
    cb = card(root)
    cb.pack(fill='x', padx=6, pady=3)
    tk.Label(cb, text='CLEANING PATTERNS', font=fn_lbl, fg=DIM, bg=CARD).pack(anchor='w')

    bg = tk.Frame(cb, bg=CARD)
    bg.pack(fill='x', pady=4)
    bg.columnconfigure(0, weight=1); bg.columnconfigure(1, weight=1); bg.columnconfigure(2, weight=1)

    # Row 1: Wall Follow, Spiral, Zigzag
    tk.Button(bg, text='Wall Follow\n0-30%', font=fn_btn, fg='white', bg=GREEN,
              relief='flat', pady=5, command=lambda: node.pub_pattern('wall_follow')
              ).grid(row=0, column=0, padx=2, pady=2, sticky='ew')
    tk.Button(bg, text='Spiral\n30-60%', font=fn_btn, fg='white', bg=PURPLE,
              relief='flat', pady=5, command=lambda: node.pub_pattern('spiral')
              ).grid(row=0, column=1, padx=2, pady=2, sticky='ew')
    tk.Button(bg, text='Zigzag\n30-60%', font=fn_btn, fg='white', bg=ACCENT,
              relief='flat', pady=5, command=lambda: node.pub_pattern('zigzag')
              ).grid(row=0, column=2, padx=2, pady=2, sticky='ew')

    # Row 2: Random, Targeted
    tk.Button(bg, text='Random\n60-85%', font=fn_btn, fg='white', bg=ORANGE,
              relief='flat', pady=5, command=lambda: node.pub_pattern('random')
              ).grid(row=1, column=0, padx=2, pady=2, sticky='ew')
    tk.Button(bg, text='Targeted\n85-100%', font=fn_btn, fg='white', bg=TEAL,
              relief='flat', pady=5, command=lambda: node.pub_pattern('targeted')
              ).grid(row=1, column=1, padx=2, pady=2, sticky='ew')

    # STOP button
    tk.Button(cb, text='STOP / CANCEL', font=fn_btn, fg='white', bg=RED,
              relief='flat', pady=6, command=node.pub_cancel).pack(fill='x', padx=3, pady=(4,3))

    # LOG
    cl = card(root)
    cl.pack(fill='both', expand=True, padx=6, pady=(3,6))
    tk.Label(cl, text='LOG', font=fn_lbl, fg=DIM, bg=CARD).pack(anchor='w')
    log_txt = tk.Text(cl, height=5, bg='#0d1117', fg=DIM, font=fn_sm,
                      relief='flat', borderwidth=0, wrap='word')
    log_txt.pack(fill='both', expand=True, pady=(3,0))
    log_txt.config(state='disabled')

    prev_log = [0]

    def update():
        executor.spin_once(timeout_sec=0.01)

        ft = node.floor_type
        fl_val.config(text=ft.upper(), fg=floor_color(ft))
        fl_icon.config(text=floor_icon(ft))
        fl_bar.config(bg=floor_color(ft))
        fl_conf.config(text=node.confidence)

        vs = node.vac_speed
        vc_val.config(text=vs, fg=speed_color(vs))
        bar_fl.config(bg=speed_color(vs))
        bar_fl.place(x=0, y=0, relheight=1.0, relwidth=speed_int(vs)/100.0)

        fm = node.floor_mode
        mode_indicator.config(text='AUTO' if 'AUTO' in fm else 'MANUAL',
                              fg=GREEN if 'AUTO' in fm else ORANGE)

        cs = node.clean_status
        color_map = {'FOLLOW':GREEN,'ZIGZAG':ACCENT,'SPIRAL':PURPLE,
                     'RANDOM':ORANGE,'TARGET':TEAL,'SEARCH':ORANGE,
                     'CORNER':ORANGE,'CANCEL':RED,'COMPLET':GREEN,
                     'EXPAND':PURPLE,'CONTRACT':PURPLE}
        cl_color = CYAN
        for k, v in color_map.items():
            if k in cs: cl_color = v; break
        cl_val.config(text=cs, fg=cl_color)

        if len(node.logs) != prev_log[0]:
            prev_log[0] = len(node.logs)
            log_txt.config(state='normal')
            log_txt.delete('1.0', 'end')
            log_txt.insert('end', '\n'.join(node.logs[-12:]))
            log_txt.see('end')
            log_txt.config(state='disabled')

        root.after(50, update)

    update()

    def on_close():
        root.destroy(); node.destroy_node(); rclpy.shutdown()
    root.protocol("WM_DELETE_WINDOW", on_close)
    root.mainloop()

if __name__ == '__main__':
    main()