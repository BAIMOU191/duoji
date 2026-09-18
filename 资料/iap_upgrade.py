"""舵机 IAP 升级脚本（单文件，改完开头的配置直接运行）。

    python iap_upgrade.py                 按下面的配置升级
    python iap_upgrade.py --query         只读舵机信息，不升级
    python iap_upgrade.py --rescue        救砖：提示断电重上电后再升级
    python iap_upgrade.py --port COM5 --id 3 --fw 新固件.hex    临时覆盖配置

依赖：pyserial（pip install pyserial）
协议见《舵机 IAP 升级协议》。
"""

# ============================== 配置 ==============================
PORT      = 'COM141'                                        # 串口号
SERVO_ID  = 0                                             # 舵机 ID，0~254
APP_BAUD  = 115200                                        # 舵机当前的波特率
HEX_FILE  = r'CH32V00X_APP_ADC\obj\CH32V00X_APP.hex'      # 要烧录的固件
ERASE_PARAMS = False                                      # True = 连参数页(ID/波特率/校准)一起擦
# ==================================================================

import argparse
import re
import struct
import sys
import time
import zlib

HEAD, TAIL = b'\xAA\x55', b'\x55\xAA'
CMD_HELLO, CMD_START, CMD_WRITE, CMD_FINISH = 0x01, 0x02, 0x03, 0x04
CMD_QUERY, CMD_ENTER = 0x20, 0x21
BOOT_BAUD, PAGE, APP_MAX, FW_INFO_OFFSET = 115200, 256, 0xF400, 0x100

STATUS_TEXT = {
    0: '成功', 1: '帧无效(CRC 或长度不对)', 2: '硬件信息或固件类型不符',
    3: '地址或长度越界', 4: '写入后读回不一致', 5: '整包 CRC32 不符',
    6: '顺序错误(尚未成功开始)', 7: '硬件信息已写入',
}


class Fail(Exception):
    """升级失败，消息直接给人看"""


# ------------------------------------------------------------------ 帧

def crc32(data):
    return zlib.crc32(data) & 0xFFFFFFFF


def build(cmd, data=b''):
    body = bytes([cmd]) + struct.pack('<H', len(data)) + data
    return HEAD + body + struct.pack('<I', crc32(body)) + TAIL


class Link:
    """串口收发一层薄封装：发一帧、等对应的应答帧"""

    noise = 0                                  # 收到却读不成帧的字节数：总线上有人抢着应答时会涨

    def __init__(self, port, baud):
        try:
            import serial
        except ImportError:
            raise Fail('没装 pyserial，先执行：pip install pyserial')
        try:
            self.ser = serial.Serial(port, baud, timeout=0)
        except Exception as exc:
            raise Fail('打开 %s 失败：%s\n可用串口：%s' % (port, exc, self.list_ports()))
        self.buf = bytearray()

    @staticmethod
    def list_ports():
        try:
            from serial.tools import list_ports
            names = [p.device for p in list_ports.comports()]
            return '、'.join(names) if names else '(没找到串口)'
        except Exception:
            return '(无法枚举)'

    def set_baud(self, baud):
        if self.ser.baudrate != baud:
            self.ser.baudrate = baud

    def send(self, cmd, data=b''):
        self.buf.clear()
        self.ser.reset_input_buffer()
        self.ser.write(build(cmd, data))
        self.ser.flush()

    def _take_frame(self):
        """从缓冲里取出一整帧，取不出返回 None"""
        while True:
            start = self.buf.find(HEAD)
            if start < 0:
                self.noise += max(0, len(self.buf) - 1)
                del self.buf[:max(0, len(self.buf) - 1)]
                return None
            if start:
                self.noise += start
                del self.buf[:start]
            if len(self.buf) < 5:
                return None
            length = self.buf[3] | self.buf[4] << 8
            if length > 260:
                self.noise += 2
                del self.buf[:2]
                continue
            if len(self.buf) < 11 + length:
                return None
            frame = bytes(self.buf[:11 + length])
            del self.buf[:11 + length]
            return frame

    def wait(self, cmd, timeout):
        """等 cmd 的应答，返回 (状态码, 状态码之后的数据)；超时返回 None"""
        end = time.monotonic() + timeout
        while True:
            frame = self._take_frame()
            if frame is not None:
                length = frame[3] | frame[4] << 8
                body = frame[2:5 + length]
                if frame[-2:] != TAIL or struct.unpack_from('<I', frame, 5 + length)[0] != crc32(body):
                    self.noise += len(frame)   # 坏帧：多半是两台一起应答；自己的回环不算
                elif frame[2] == (cmd | 0x80) and length >= 1:
                    return frame[5], frame[6:5 + length]
                continue                       # 自己的回环、别人的应答、坏帧：接着等
            remain = end - time.monotonic()
            if remain <= 0:
                return None
            chunk = self.ser.read(512)
            if chunk:
                self.buf += chunk
            else:
                time.sleep(0.001)

    def request(self, cmd, data=b'', timeout=0.2, retries=3, allow=(0,)):
        """发命令并等应答；状态码不在 allow 里就报错"""
        for _ in range(retries):
            self.send(cmd, data)
            reply = self.wait(cmd, timeout)
            if reply is None:
                continue
            status, payload = reply
            if status not in allow:
                raise Fail('命令 0x%02X 返回状态 %d：%s' % (cmd, status, STATUS_TEXT.get(status, '未知')))
            return status, payload
        raise Fail('命令 0x%02X 没有应答' % cmd)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


# ------------------------------------------------------------------ 固件

def load_hex(path):
    """解析 hex，返回 (镜像, 适用硬件4项, 版本3项)"""
    cells, base = {}, 0
    try:
        text = open(path, 'r', encoding='ascii', errors='ignore').read()
    except OSError as exc:
        raise Fail('打不开固件文件：%s' % exc)

    for line in text.splitlines():
        line = line.strip()
        if not line.startswith(':'):
            continue
        try:
            raw = bytes.fromhex(line[1:])
        except ValueError:
            raise Fail('hex 里有非法字符：%s' % line[:16])
        if sum(raw) & 0xFF:
            raise Fail('hex 行校验和不对：%s' % line[:16])
        count, addr, rtype, payload = raw[0], raw[1] << 8 | raw[2], raw[3], raw[4:4 + raw[0]]
        if rtype == 0x00:
            for i in range(count):
                cells[base + addr + i] = payload[i]
        elif rtype == 0x01:
            break
        elif rtype == 0x02:
            base = (payload[0] << 8 | payload[1]) * 16
        elif rtype == 0x04:
            base = (payload[0] << 8 | payload[1]) << 16

    if not cells:
        raise Fail('hex 里没有数据')
    cells = {(a - 0x08000000 if a >= 0x08000000 else a): v for a, v in cells.items()}
    top = max(cells)
    if top >= APP_MAX:
        raise Fail('固件数据到了 0x%X，超出 APP 区(0x%X)' % (top, APP_MAX))

    image = bytearray(b'\xFF' * (((top + 1 + PAGE - 1) // PAGE) * PAGE))
    for addr, value in cells.items():
        image[addr] = value

    info = image[FW_INFO_OFFSET:FW_INFO_OFFSET + 16]
    if info[:4] != b'SVFW':
        raise Fail('固件里没有信息块(偏移 0x100 不是 SVFW)，可能不是本产品的固件')
    return bytes(image), tuple(info[4:8]), tuple(info[8:11])


def hw_text(hw):
    return '/'.join('不限' if v == 0xFF else str(v) for v in hw)


def matches(fw_hw, dev_hw):
    if tuple(dev_hw) == (0xFF, 0xFF, 0xFF, 0xFF):
        return None                            # 舵机还没写硬件信息，没法自动判断
    return all(f == 0xFF or f == d for f, d in zip(fw_hw, dev_hw))


# ------------------------------------------------------------------ 流程

def no_reply(link, servo_id, baud):
    """查不到舵机时给一句尽量具体的提示：收到乱码和什么都没收到，原因完全不同"""
    if link.noise:
        return ('收到了 %d 字节但读不成帧。总线上可能挂了多台同 ID 的舵机在一起应答，'
                '或者还有别的设备在发数据；也确认一下波特率(脚本用的 %d)' % (link.noise, baud))
    return ('查询不到舵机。检查串口号、接线、ID(%d)和波特率(%d)；'
            '如果固件已经刷坏，用 --rescue' % (servo_id, baud))


def query(link, servo_id, timeout=0.2, retries=3):
    _, payload = link.request(CMD_QUERY, bytes([servo_id]), timeout, retries)
    return {'id': payload[0], 'hw': tuple(payload[1:5]), 'version': tuple(payload[5:8])}


def handshake(link, limit):
    """反复发握手直到 Boot 应答；limit 是最长等待秒数"""
    end = time.monotonic() + limit
    while time.monotonic() < end:
        link.send(CMD_HELLO)
        reply = link.wait(CMD_HELLO, 0.02)
        if reply and reply[0] == 0:
            payload = reply[1]
            return {'boot': payload[0], 'flag': payload[1], 'hw': tuple(payload[2:6])}
    return None


def upgrade(link, servo_id, image, fw_hw, fw_ver, app_baud, rescue, erase_params):
    info = None                                # 查询到的舵机信息；救砖或舵机已在 Boot 时为 None

    if rescue:
        print('救砖模式：请现在给舵机断电，再重新上电…')
        link.set_baud(BOOT_BAUD)
        boot = handshake(link, 30.0)
        if boot is None:
            raise Fail('30 秒内没有握手上。确认只接了这一台舵机、接线和串口号是否正确'
                       + ('（收到过 %d 字节乱码，注意 Boot 段固定 115200）' % link.noise
                          if link.noise else ''))
    else:
        link.set_baud(app_baud)
        try:
            info = query(link, servo_id)
        except Fail:
            # 查询不到：舵机可能已经停在 Boot 里(上次升级中断)，先按这个可能性试一下
            link.set_baud(BOOT_BAUD)
            boot = handshake(link, 1.0)
            if boot is None:
                raise Fail(no_reply(link, servo_id, app_baud))
            print('舵机已经停在 Boot 里，直接继续升级')
            info = None
        else:
            print('舵机：ID %d，硬件信息 %s，当前固件 %d.%d.%d'
                  % (info['id'], hw_text(info['hw']), *info['version']))
        if info is not None:
            ok = matches(fw_hw, info['hw'])
            if ok is False:
                raise Fail('固件适用硬件 %s 与舵机 %s 不符，已停止' % (hw_text(fw_hw), hw_text(info['hw'])))
            if ok is None:
                print('注意：舵机的硬件信息还没写入，无法自动核对型号')
            if tuple(info['version']) == tuple(fw_ver):
                print('注意：舵机当前版本与待升级固件相同')

            print('进入升级…')
            try:
                link.request(CMD_ENTER, bytes([servo_id]) + bytes(info['hw']) + bytes(info['version']))
            except Fail as exc:
                if '没有应答' not in str(exc):
                    raise
                print('  进入升级没有应答，可能已经进了 Boot，继续握手')

            link.set_baud(BOOT_BAUD)
            boot = handshake(link, 3.0)
            if boot is None:
                raise Fail('切到 115200 后没有握手上，舵机可能没进 Boot')

    print('Boot：版本 %d，标志页状态 %d，硬件信息 %s' % (boot['boot'], boot['flag'], hw_text(boot['hw'])))
    if info is None:                           # 没查询过(救砖或舵机本来就在 Boot)：用握手应答核对
        ok = matches(fw_hw, boot['hw'])
        if ok is False:
            raise Fail('固件适用硬件 %s 与舵机 %s 不符，已停止' % (hw_text(fw_hw), hw_text(boot['hw'])))

    length, image_crc = len(image), crc32(image)
    total = length // PAGE
    print('开始擦除：长度 %d 字节，CRC32 %08X，共 %d 页%s'
          % (length, image_crc, total, '（连参数页一起擦）' if erase_params else ''))
    link.request(CMD_START, struct.pack('<IIB', length, image_crc, 1 if erase_params else 0),
                 timeout=5.0, retries=3)

    start_time = time.monotonic()
    for index in range(total):
        offset = index * PAGE
        data = struct.pack('<I', offset) + image[offset:offset + PAGE]
        for attempt in range(5):
            link.send(CMD_WRITE, data)
            reply = link.wait(CMD_WRITE, 0.3)
            if reply and reply[0] == 0 and struct.unpack_from('<I', reply[1])[0] == offset:
                break
            if reply and reply[0] in (3, 6):
                raise Fail('写页返回状态 %d：%s' % (reply[0], STATUS_TEXT[reply[0]]))
        else:
            raise Fail('偏移 0x%X 的页写了 5 次都没成功' % offset)
        done = index + 1
        if done % 10 == 0 or done == total:
            print('\r  写入 %3d/%d 页 (%3d%%)' % (done, total, done * 100 // total), end='', flush=True)
    print('   用时 %.1f 秒' % (time.monotonic() - start_time))

    print('整包校验…')
    try:
        link.request(CMD_FINISH, b'', timeout=2.0, retries=3)
    except Fail as exc:
        if '没有应答' not in str(exc):
            raise
        print('  完成命令没有应答，去查版本确认')

    print('等待新固件启动…')
    time.sleep(1.5)
    verify_id, verify_baud = servo_id, app_baud
    if erase_params:                           # 参数被擦掉了，舵机回到出厂的 ID 和波特率
        verify_id, verify_baud = 0, 115200
        print('参数页已擦除：ID 回到 0，波特率回到 115200')
    link.set_baud(verify_baud)

    candidates = [verify_id]
    if rescue or erase_params:
        candidates.append(255)                 # ID 不确定时用广播兜底(只能单独接线)
    info, last = None, None
    for cand in candidates:
        try:
            info = query(link, cand, timeout=0.3, retries=8)
            break
        except Fail as exc:
            last = exc
    if info is None:
        raise last
    if tuple(info['version']) != tuple(fw_ver):
        raise Fail('升级后读到的版本是 %d.%d.%d，期望 %d.%d.%d' % (*info['version'], *fw_ver))
    print('升级完成：固件 %d.%d.%d，ID %d，硬件信息 %s'
          % (*info['version'], info['id'], hw_text(info['hw'])))


def main():
    ap = argparse.ArgumentParser(description='舵机 IAP 升级')
    ap.add_argument('--port', default=PORT)
    ap.add_argument('--id', type=int, default=SERVO_ID)
    ap.add_argument('--baud', type=int, default=APP_BAUD, help='舵机当前波特率')
    ap.add_argument('--fw', default=HEX_FILE)
    ap.add_argument('--rescue', action='store_true', help='救砖：跳过查询，等断电重上电')
    ap.add_argument('--erase-params', action='store_true',
                    help='连参数页一起擦，ID 和波特率会恢复默认')
    ap.add_argument('--query', action='store_true', help='只读舵机信息')
    args = ap.parse_args()

    link = None
    try:
        if args.query:
            link = Link(args.port, args.baud)
            try:
                info = query(link, args.id)
                print('ID %d，硬件信息 %s，固件版本 %d.%d.%d'
                      % (info['id'], hw_text(info['hw']), *info['version']))
            except Fail:
                link.set_baud(BOOT_BAUD)       # 查不到就看看是不是停在 Boot 里
                boot = handshake(link, 1.0)
                if boot is None:
                    raise Fail(no_reply(link, args.id, args.baud))
                print('舵机停在 Boot 里：版本 %d，标志页状态 %d，硬件信息 %s'
                      % (boot['boot'], boot['flag'], hw_text(boot['hw'])))
            return 0

        image, fw_hw, fw_ver = load_hex(args.fw)
        print('固件：%s' % args.fw)
        print('      版本 %d.%d.%d，适用硬件 %s，%d 字节'
              % (*fw_ver, hw_text(fw_hw), len(image)))
        risky = len(re.findall(rb'#\d{3}P[^!]{1,26}!', image, re.S))
        if risky:
            print('提醒：固件里有 %d 处字节序列长得像舵机 ASCII 指令，'
                  '升级时总线上的其他舵机可能误动作，建议只接这一台' % risky)
        link = Link(args.port, args.baud)
        upgrade(link, args.id, image, fw_hw, fw_ver, args.baud, args.rescue,
                args.erase_params or ERASE_PARAMS)
        return 0
    except Fail as exc:
        print('\n失败：%s' % exc, file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print('\n已中断。舵机可能停在 Boot 里，重新运行本脚本即可继续升级。', file=sys.stderr)
        return 1
    finally:
        if link:
            link.close()


if __name__ == '__main__':
    sys.exit(main())
