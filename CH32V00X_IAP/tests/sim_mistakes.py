"""用户用错时的仿真：波特率不对、ID 不对、固件选错、总线上挂了两台……

用法：python tests/sim_mistakes.py [固件hex路径]
每个场景都按命令行跑真正的 iap_upgrade.py（串口换成仿真器），关心两件事：
  1. 脚本给出的提示能不能让人看懂错在哪；
  2. 出错以后舵机还能不能正常用（Flash 有没有被写坏）。
"""
import contextlib
import io
import struct
import sys

from sim_serial import OUT, SimBus, NoClose, build_sim, link_for, new_servo  # noqa: E402
from sim_upgrade import DEFAULT_HEX, enter_boot, load_firmware, run  # noqa: E402

import iap_upgrade as U  # noqa: E402

HEX = str(DEFAULT_HEX)          # main() 里按命令行参数改写


def cli(servo, *args):
    """按命令行跑一遍真正的 main()，串口换成仿真舵机，返回 (退出码, 屏幕输出)"""
    saved_link, saved_argv = U.Link, sys.argv

    class FakeLink(saved_link):
        def __init__(self, port, baud):          # noqa: D107  不开真串口，直接接到仿真舵机
            assert servo is not None, '这一步本来不该打开串口'
            servo.baudrate = baud
            self.ser = NoClose(servo)
            self.buf = bytearray()

    U.Link = FakeLink
    sys.argv = ['iap_upgrade.py'] + [str(a) for a in args]
    buf = io.StringIO()
    try:
        with contextlib.redirect_stdout(buf), contextlib.redirect_stderr(buf):
            code = U.main()
    finally:
        U.Link, sys.argv = saved_link, saved_argv
    return code, buf.getvalue()


def write_hex(name, data, base=0x08000000):
    """把一段数据写成 Intel hex，用来造各种"选错文件"的场景"""
    def record(addr, rtype, payload):
        raw = bytes([len(payload), (addr >> 8) & 0xFF, addr & 0xFF, rtype]) + payload
        return ':' + (raw + bytes([(-sum(raw)) & 0xFF])).hex().upper()

    lines, high = [], None
    for off in range(0, len(data), 16):
        addr = base + off
        if (addr >> 16) != high:
            high = addr >> 16
            lines.append(record(0, 0x04, bytes([high >> 8, high & 0xFF])))
        lines.append(record(addr & 0xFFFF, 0x00, data[off:off + 16]))
    lines.append(':00000001FF')
    path = OUT / name
    path.write_text('\n'.join(lines) + '\n')
    return str(path)


class Echo:
    """半双工转换板把主机自己发出去的字节又送回 RX（很多单线模块都这样）"""

    def __init__(self, inner):
        self.inner = inner
        self.pending = bytearray()

    @property
    def baudrate(self):
        return self.inner.baudrate

    @baudrate.setter
    def baudrate(self, value):
        self.inner.baudrate = value

    def write(self, data):
        self.pending += data
        self.inner.write(data)

    def read(self, size):
        if self.pending:
            out = bytes(self.pending[:size])
            del self.pending[:size]
            return out
        return self.inner.read(size)

    def reset_input_buffer(self):
        self.pending.clear()
        self.inner.reset_input_buffer()

    def __getattr__(self, name):
        return getattr(self.inner, name)


class Dead:
    """线没接上/串口选到别的设备：发得出去，什么也收不回来"""

    baudrate = 115200

    def write(self, data):
        pass

    def flush(self):
        pass

    def reset_input_buffer(self):
        pass

    def read(self, size):
        return b''

    def close(self):
        pass


def intact(servo, image):
    """舵机还是升级前那台：在 APP 里跑，固件没被动，标志页也没被改成"APP 作废"""
    return (servo.state()[0] == 'app' and servo.read_flash(0, len(image)) == image
            and servo.read_flash(0xF700, 4) != b'UPGD')


# ============================== 波特率 ==============================

def case_servo_baud_changed(image, fw_hw, fw_ver):
    """舵机波特率被改成 1M，脚本里还写着 115200"""
    servo = new_servo(image, app_baud=1000000)
    code, out = cli(servo, '--fw', HEX)
    assert code == 1, '不该报成功'
    assert '波特率(115200)' in out, out
    assert intact(servo, image), '失败以后舵机被写坏了'
    servo.close()
    return '舵机听不懂也就不应答，脚本提示"检查…ID(0)和波特率(115200)"，舵机照常运行'


def case_servo_baud_matched(image, fw_hw, fw_ver):
    """舵机波特率是 1M，脚本按 --baud 1000000 升级：升完波特率保持不变"""
    servo = new_servo(image, app_baud=1000000)
    code, out = cli(servo, '--fw', HEX, '--baud', 1000000)
    assert code == 0, out
    assert servo.read_flash(0, len(image)) == image and servo.state()[0] == 'app'
    assert '升级完成' in out, out
    servo.close()
    return '非 115200 的舵机照样能升级(Boot 段固定 115200，脚本自己切)，参数页保留'


def case_host_baud_wrong(image, fw_hw, fw_ver):
    """舵机是 115200，用户在脚本里填了 1M"""
    servo = new_servo(image)
    code, out = cli(servo, '--fw', HEX, '--baud', 1000000)
    assert code == 1, '不该报成功'
    assert '波特率(1000000)' in out, out
    assert intact(servo, image)
    servo.close()
    return '舵机收到的是乱码，不会应答；脚本提示检查波特率(1000000)后停下，不乱写 Flash'


def case_boot_baud_fallback(image, fw_hw, fw_ver):
    """舵机因为上次中断停在 Boot 里，而脚本里的波特率填的是别的值"""
    servo = new_servo(image, app_baud=1000000)
    link = link_for(servo)                          # 先按 1M 正常进 Boot
    info = U.query(link, 0)
    link.request(U.CMD_ENTER, bytes([0]) + bytes(info['hw']) + bytes(info['version']))
    link.set_baud(U.BOOT_BAUD)
    assert U.handshake(link, 3.0) is not None
    link.request(U.CMD_START, struct.pack('<IIB', len(image), U.crc32(image), 0), timeout=5.0)

    code, out = cli(servo, '--fw', HEX, '--baud', 1000000)   # 重新跑脚本
    assert '停在 Boot' in out, out
    assert code == 0, out
    assert servo.read_flash(0, len(image)) == image and servo.state()[0] == 'app'
    servo.close()
    return 'Boot 段永远是 115200，脚本查不到就自动去 115200 握手，接着升完'


# ============================== ID ==============================

def case_no_wire(image, fw_hw, fw_ver):
    """线没接好，或者串口号选到了别的设备"""
    code, out = cli(Dead(), '--fw', HEX)
    assert code == 1, '不该报成功'
    assert '接线' in out, out
    return '一直收不到应答就报"检查串口号、接线…"，不会空转'


def case_half_duplex_echo(image, fw_hw, fw_ver):
    """单线半双工转换板把自己发的字节回显了一份"""
    servo = new_servo(image)
    code, out = cli(Echo(servo), '--fw', HEX)
    assert code == 0, out
    assert servo.read_flash(0, len(image)) == image and servo.state()[0] == 'app'
    servo.close()
    return '自己的回显会被当成非应答帧跳过，升级照常完成'


def case_wrong_id(image, fw_hw, fw_ver):
    """舵机 ID 是 3，脚本里填的 0"""
    servo = new_servo(image, servo_id=3)
    code, out = cli(servo, '--fw', HEX)
    assert code == 1, '不该报成功'
    assert 'ID(0)' in out, out
    assert intact(servo, image)
    servo.close()
    return '提示里带上了用的 ID，舵机不受影响'


def case_broadcast_query(image, fw_hw, fw_ver):
    """不知道 ID：单独接一台，用 255 广播查"""
    servo = new_servo(image, servo_id=7)
    code, out = cli(servo, '--query', '--id', 255, '--fw', HEX)
    assert code == 0, out
    assert 'ID 7' in out, out
    servo.close()
    return '--query --id 255 能把忘掉的 ID 读回来(只能单独接一台)'


# ============================== 固件文件 ==============================

def case_wrong_model(image, fw_hw, fw_ver):
    """拿另一种舵机(类型2)的固件来刷"""
    other = bytearray(image)
    other[0x104] = 2
    path = write_hex('other_model.hex', bytes(other))
    servo = new_servo(image)
    code, out = cli(servo, '--fw', path)
    assert code == 1, '不该报成功'
    assert '不符' in out, out
    assert intact(servo, image), '拒绝之前就不该动 Flash'
    servo.close()
    return '查询到型号就比对，型号不符当场停手，一个字节都没写'


def case_not_our_hex(image, fw_hw, fw_ver):
    """随便一个 hex(不是本产品的固件)"""
    path = write_hex('foreign.hex', bytes(0x200))
    code, out = cli(None, '--fw', path)
    assert code == 1 and '信息块' in out, out
    return '固件里没有 SVFW 信息块，脚本连串口都不开就报错'


def case_bin_file(image, fw_hw, fw_ver):
    """把 bin 当成 hex 选了"""
    path = OUT / 'firmware.bin'
    path.write_bytes(image[:1024])
    code, out = cli(None, '--fw', str(path))
    assert code == 1 and '没有数据' in out, out
    return '选成 bin 文件会直接报"hex 里没有数据"'


def case_hex_out_of_range(image, fw_hw, fw_ver):
    """固件数据落到了参数区里(编译时改错了链接脚本)"""
    path = write_hex('too_big.hex', bytes(16), base=0x0800F500)
    code, out = cli(None, '--fw', path)
    assert code == 1 and '超出 APP 区' in out, out
    return '数据越过 0xF400 直接报错，不会把参数页冲掉'


def case_same_version(image, fw_hw, fw_ver):
    """重复刷同一个版本"""
    servo = new_servo(image)
    code, out = cli(servo, '--fw', HEX)
    assert code == 0, out
    assert '与待升级固件相同' in out, out
    servo.close()
    return '提示版本相同但照常刷完，重刷不会被卡住'


# ============================== 舵机状态 ==============================

def case_blank_hwinfo(image, fw_hw, fw_ver):
    """老舵机(旧固件刷进来的)硬件信息页还是空的"""
    servo = new_servo(image)
    servo.set_flash(0xF600, b'\xFF' * 512)          # 假装出厂时没写过
    code, out = cli(servo, '--fw', HEX)
    assert code == 0, out
    assert '硬件信息还没写入' in out, out
    assert servo.read_flash(0xF600, 4) == b'HWIF', '新固件启动后应补写硬件信息'
    assert servo.read_flash(0xF605, 4) == bytes(fw_hw), '补写的内容应来自固件'
    servo.close()
    return '硬件信息为空时提示"无法自动核对型号"，升级后由新固件补写'


def case_blank_hwinfo_wrong_model(image, fw_hw, fw_ver):
    """出厂新板子(硬件信息页还空着)刷成了别的型号的固件"""
    other = bytearray(image)
    other[0x104] = 2
    path = write_hex('other_model.hex', bytes(other))
    servo = new_servo(image)
    servo.set_flash(0xF600, b'\xFF' * 512)         # 从没跑过新固件，页是空的
    code, out = cli(servo, '--fw', path)
    assert code == 0 and '硬件信息还没写入' in out, out
    assert servo.read_flash(0x104, 1) == bytes([2]), '错型号的固件应该原样刷进去了'
    assert servo.read_flash(0xF700, 4) == b'APOK', 'Boot 在完成时也没拦住'
    servo.close()
    return '硬件信息页为空时两道型号校验都失效，错固件照样刷进去(量产必须先把硬件信息写上)'


def case_resume_after_abort(image, fw_hw, fw_ver):
    """传到一半把脚本关了，舵机一直停在 Boot，重新跑一遍脚本"""
    servo = new_servo(image)
    link = enter_boot(servo)
    link.request(U.CMD_START, struct.pack('<IIB', len(image), U.crc32(image), 0), timeout=5.0)
    for offset in range(0, 5 * U.PAGE, U.PAGE):
        link.request(U.CMD_WRITE, struct.pack('<I', offset) + image[offset:offset + U.PAGE],
                     timeout=0.3)
    servo.tick(30000)                               # 晾 30 秒
    assert servo.state()[0] == 'boot', 'APP 已作废时不能自己跳回去'

    code, out = cli(servo, '--fw', HEX)
    assert code == 0, out
    assert '停在 Boot' in out, out
    assert servo.read_flash(0, len(image)) == image and servo.state()[0] == 'app'
    servo.close()
    return '中途关脚本不会变砖：舵机停在 Boot 等着，重跑脚本直接接上'


def case_query_in_boot(image, fw_hw, fw_ver):
    """--query 一台停在 Boot 里的舵机"""
    servo = new_servo(image)
    link = enter_boot(servo)
    link.request(U.CMD_START, struct.pack('<IIB', len(image), U.crc32(image), 0), timeout=5.0)
    code, out = cli(servo, '--query', '--fw', HEX)
    assert code == 0, out
    assert '停在 Boot' in out, out
    servo.close()
    return '--query 能认出舵机停在 Boot 里，不会只报一句查不到'


# ============================== 总线 ==============================

def case_bus_other_servo(image, fw_hw, fw_ver):
    """一根总线上挂两台(ID 3 和 ID 5)，只升级 ID 3"""
    target = new_servo(image, servo_id=3)
    other = new_servo(image, servo_id=5)
    bus = SimBus([target, other])
    code, out = cli(bus, '--fw', HEX, '--id', 3)
    assert code == 0, out
    assert target.read_flash(0, len(image)) == image and target.state()[0] == 'app'
    assert intact(other, image), '旁边那台被写坏了'
    bus.close()
    return '按 ID 点名，旁边那台既不应答也没被写(仍建议只接一台)'


def case_bus_same_id(image, fw_hw, fw_ver):
    """两台舵机 ID 都是 0，一起挂在总线上"""
    a, b = new_servo(image, servo_id=0), new_servo(image, servo_id=0)
    bus = SimBus([a, b])
    code, out = cli(bus, '--fw', HEX)
    assert code == 1, '同 ID 不该被当成升级成功'
    assert '同 ID' in out, out
    assert intact(a, image) and intact(b, image), '同 ID 冲突时不该写 Flash'
    bus.close()
    return '两台同时应答变成乱码，脚本提示"可能有多台同 ID 的舵机"，两台都没被动'


CASES = [
    ('舵机波特率被改', case_servo_baud_changed),
    ('按实际波特率', case_servo_baud_matched),
    ('上位机波特率填错', case_host_baud_wrong),
    ('停在Boot+波特率', case_boot_baud_fallback),
    ('线没接好', case_no_wire),
    ('半双工回显', case_half_duplex_echo),
    ('ID填错', case_wrong_id),
    ('广播查ID', case_broadcast_query),
    ('固件型号选错', case_wrong_model),
    ('不是本产品固件', case_not_our_hex),
    ('选成bin文件', case_bin_file),
    ('固件越界', case_hex_out_of_range),
    ('重复刷同版本', case_same_version),
    ('硬件信息页空白', case_blank_hwinfo),
    ('空白硬件信息+错固件', case_blank_hwinfo_wrong_model),
    ('中途关掉脚本', case_resume_after_abort),
    ('查询停在Boot的', case_query_in_boot),
    ('总线上还有别人', case_bus_other_servo),
    ('两台同ID', case_bus_same_id),
]


def main(argv):
    global HEX

    build_sim()
    OUT.mkdir(parents=True, exist_ok=True)
    image, fw_hw, fw_ver = load_firmware(argv)
    HEX = argv[0] if argv else str(DEFAULT_HEX)
    failures = run(CASES, image, fw_hw, fw_ver)
    print('%d/%d 个场景通过' % (len(CASES) - failures, len(CASES)))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
