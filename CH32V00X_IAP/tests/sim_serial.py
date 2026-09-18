"""把仿真器包装成 pyserial 的 Serial 样子，让测试脚本能直接驱动仓库根目录的 iap_upgrade.py。

仿真器里跑的是 Boot 和 APP 的真实协议代码，Flash、串口、计时是模型：
  - 波特率对不上时，两个方向的字节都变成乱码(真实现象)
  - 擦除模式1 擦掉参数页后，舵机的 ID 和波特率回到出厂值
"""
import pathlib
import subprocess
import sys
import tempfile

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent                      # CH32V00X_IAP
REPO = ROOT.parent
APP = REPO / 'CH32V00X_APP_ADC'
OUT = pathlib.Path(tempfile.gettempdir()) / 'servo_sim'
SIM = OUT / 'servo_sim.exe'
CC = ['gcc', '-std=c99', '-O1', '-Wall', '-Wextra', '-Wno-unused-parameter']

sys.path.insert(0, str(REPO))           # 仓库根目录的 iap_upgrade.py
PARAM_PATTERN = bytes([0x22]) * 512     # 假装参数页 A/B 里存着参数


def build_sim():
    """两个工程都有 System/iap.c，只能分开编译再链接"""
    OUT.mkdir(parents=True, exist_ok=True)
    units = [
        ('sim_main', ['-I' + str(HERE / 'stubs'), '-I' + str(HERE / 'sim')]),
        ('sim_boot', ['-I' + str(HERE / 'stubs'), '-I' + str(HERE / 'sim'),
                      '-I' + str(ROOT / 'System')]),
        ('sim_app', ['-I' + str(HERE / 'stubs'), '-I' + str(HERE / 'sim'),
                     '-I' + str(APP / 'System'), '-I' + str(APP / 'Application'),
                     '-I' + str(APP / 'Common'), '-I' + str(APP / 'Drivers')]),
    ]
    objs = []
    for name, inc in units:
        obj = OUT / (name + '.o')
        subprocess.run(CC + inc + ['-c', str(HERE / 'sim' / (name + '.c')), '-o', str(obj)], check=True)
        objs.append(str(obj))
    subprocess.run(['gcc'] + objs + ['-o', str(SIM)], check=True)


class SimServo:
    """一台仿真舵机，对外长得像 pyserial 的 Serial"""

    def __init__(self):
        self.proc = subprocess.Popen([str(SIM)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     text=True, bufsize=1)
        self.inbox = bytearray()
        self._baud = 115200

    # ---- 仿真器命令 ----
    def cmd(self, line):
        self.proc.stdin.write(line + '\n')
        self.proc.stdin.flush()
        out = []
        while True:
            reply = self.proc.stdout.readline()
            if not reply or reply.strip() == 'END':
                break
            tag, _, body = reply.strip().partition(' ')
            if tag == 'RX':
                self.inbox += bytes.fromhex(body)
            else:
                out.append((tag, body))
        return out

    # ---- pyserial 接口 ----
    @property
    def baudrate(self):
        return self._baud

    @baudrate.setter
    def baudrate(self, value):
        self._baud = value
        self.cmd('BAUD %d' % value)

    def write(self, data):
        self.cmd('TX %s 5' % data.hex().upper())

    def flush(self):
        pass

    def reset_input_buffer(self):
        self.inbox.clear()

    def read(self, size):
        if not self.inbox:
            self.cmd('TICK 2')
        out = bytes(self.inbox[:size])
        del self.inbox[:size]
        return out

    def close(self):
        try:
            self.cmd('QUIT')
            self.proc.stdin.close()
            self.proc.wait(timeout=5)
        except Exception:
            self.proc.kill()

    # ---- 场景搭建 ----
    def power_cycle(self):
        self.inbox.clear()
        self.cmd('POWER')

    def tick(self, ms):
        self.cmd('TICK %d' % ms)

    def erase_all(self):
        self.cmd('ERASEALL')

    def set_flash(self, offset, data):
        self.cmd('SETFLASH %X %s' % (offset, data.hex().upper()))

    def read_flash(self, offset, length):
        for tag, body in self.cmd('FLASH %X %d' % (offset, length)):
            if tag == 'DATA':
                return bytes.fromhex(body)
        return b''

    def set_id(self, servo_id):
        self.cmd('ID %d' % servo_id)

    def set_app_baud(self, baud):
        self.cmd('APPBAUD %d' % baud)

    def state(self):
        for tag, body in self.cmd('STATE'):
            if tag == 'STATE':
                mode, flag = body.split()
                return mode, flag
        return None, None


class SimBus:
    """一根总线挂多台舵机：主机发的字节每台都收到，回复按顺序拼起来"""

    def __init__(self, servos):
        self.servos = servos
        self._baud = 115200

    @property
    def baudrate(self):
        return self._baud

    @baudrate.setter
    def baudrate(self, value):
        self._baud = value
        for servo in self.servos:
            servo.baudrate = value

    def write(self, data):
        for servo in self.servos:
            servo.write(data)

    def flush(self):
        pass

    def reset_input_buffer(self):
        for servo in self.servos:
            servo.reset_input_buffer()

    def read(self, size):
        chunks = [servo.read(size) for servo in self.servos]
        live = [c for c in chunks if c]
        if len(live) <= 1:
            return live[0] if live else b''
        # 两台同时应答：各自起始时刻和时钟都不一样，线上的信号叠在一起，收到的是乱码
        out = bytearray(b'\xFF' * (max(len(c) for c in live) + len(live)))
        for skew, chunk in enumerate(live):
            for i, value in enumerate(chunk):
                out[skew + i] &= value
        return bytes(out)

    def close(self):
        for servo in self.servos:
            servo.close()


class NoClose:
    """挡住 close()：脚本跑完会关串口，仿真舵机还要留着检查 Flash"""

    def __init__(self, inner):
        self.inner = inner

    @property
    def baudrate(self):
        return self.inner.baudrate

    @baudrate.setter
    def baudrate(self, value):
        self.inner.baudrate = value

    def close(self):
        pass

    def __getattr__(self, name):
        return getattr(self.inner, name)


def link_for(serial_like):
    """造一个 iap_upgrade.Link，底层换成仿真器"""
    import iap_upgrade as U

    link = U.Link.__new__(U.Link)       # 不走 __init__，直接塞进假串口
    link.ser = serial_like
    link.buf = bytearray()
    return link


def new_servo(image=None, servo_id=0, app_baud=115200, params=True):
    """新舵机：WCH-Link 全片擦除 -> 烧 APP -> 上电(Boot 等 200ms 窗口后进 APP)"""
    servo = SimServo()
    servo.erase_all()
    if image is not None:
        servo.set_flash(0, image)
        if params:
            servo.set_flash(0xF400, PARAM_PATTERN)
    servo.set_id(servo_id)
    servo.set_app_baud(app_baud)
    servo.baudrate = app_baud
    servo.power_cycle()
    servo.tick(300)
    return servo
