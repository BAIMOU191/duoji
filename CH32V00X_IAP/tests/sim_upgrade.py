"""整机仿真：用真实的 Boot / APP 协议代码和真实的固件 hex，把升级流程跑一遍。

用法：python tests/sim_upgrade.py [固件hex路径]
默认用 CH32V00X_APP_ADC/obj/CH32V00X_APP.hex（MounRiver 编译出来的那份）。
升级逻辑直接调用仓库根目录的 iap_upgrade.py，测的就是用户实际在用的那个脚本。
"""
import contextlib
import io
import struct
import sys

from sim_serial import (PARAM_PATTERN, REPO, build_sim, link_for,  # noqa: E402
                        new_servo)

import iap_upgrade as U  # noqa: E402  （sim_serial 已经把仓库根目录加进了搜索路径）

DEFAULT_HEX = REPO / 'CH32V00X_APP_ADC' / 'obj' / 'CH32V00X_APP.hex'


@contextlib.contextmanager
def quiet():
    """吞掉脚本的进度输出，需要时可以拿回来检查提示语"""
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        yield buf


def enter_boot(servo, servo_id=0):
    """走正常流程进 Boot，返回 link"""
    link = link_for(servo)
    info = U.query(link, servo_id)
    link.request(U.CMD_ENTER, bytes([servo_id]) + bytes(info['hw']) + bytes(info['version']))
    link.set_baud(U.BOOT_BAUD)
    assert U.handshake(link, 3.0) is not None, '没有握手上'
    return link


def write_all(link, image):
    for offset in range(0, len(image), U.PAGE):
        link.request(U.CMD_WRITE, struct.pack('<I', offset) + image[offset:offset + U.PAGE],
                     timeout=0.3, retries=5)


class DropReply:
    """包一层：丢掉第 n 次应答，模拟应答在总线上被干扰掉"""

    def __init__(self, inner, drop_at):
        self.inner = inner
        self.drop_at = drop_at
        self.writes = 0
        self.armed = False

    @property
    def baudrate(self):
        return self.inner.baudrate

    @baudrate.setter
    def baudrate(self, value):
        self.inner.baudrate = value

    def write(self, data):
        self.writes += 1
        if self.writes == self.drop_at:
            self.armed = True
        self.inner.write(data)

    def read(self, size):
        data = self.inner.read(size)
        if self.armed and data:
            self.armed = False
            self.inner.reset_input_buffer()
            return b''
        return data

    def __getattr__(self, name):
        return getattr(self.inner, name)


# ============================== 场景 ==============================

def scenario_normal(image, fw_hw, fw_ver):
    """正常升级：参数页、硬件信息页都不动，标志页写成 APOK，最后回到 APP"""
    servo = new_servo(image)
    assert servo.state()[0] == 'app', '上电后应该进 APP'
    assert servo.read_flash(0xF600, 4) == b'HWIF', 'APP 首次上电应写入硬件信息'
    hw_before = servo.read_flash(0xF600, 32)

    with quiet():
        U.upgrade(link_for(servo), 0, image, fw_hw, fw_ver, 115200, False, False)

    assert servo.state()[0] == 'app', '升级完应该回到 APP'
    assert servo.read_flash(0, len(image)) == image, 'Flash 内容与镜像不一致'
    assert servo.read_flash(0xF400, 512) == PARAM_PATTERN, '参数页被动了'
    assert servo.read_flash(0xF600, 32) == hw_before, '硬件信息页被动了'
    flag = servo.read_flash(0xF700, 12)
    assert flag[:4] == b'APOK', '标志页不是 APOK'
    assert int.from_bytes(flag[4:8], 'little') == len(image)
    assert int.from_bytes(flag[8:12], 'little') == U.crc32(image)
    servo.close()
    return '固件 %d 字节全部写入，参数和硬件信息页保持不变' % len(image)


def scenario_mismatch(image, fw_hw, fw_ver):
    """型号不符：上位机先拒绝；绕过上位机时 Boot 在完成时拒绝，旧 APP 已作废"""
    bad = bytearray(image)
    bad[0x104] = 2                                  # 改成另一种舵机类型
    other_hw = (2,) + tuple(fw_hw[1:])

    servo = new_servo(image)
    try:
        with quiet():
            U.upgrade(link_for(servo), 0, bytes(bad), other_hw, fw_ver, 115200, False, False)
        raise AssertionError('上位机应该拒绝不匹配的固件')
    except U.Fail as exc:
        assert '不符' in str(exc), str(exc)

    link = enter_boot(servo)                        # 绕过上位机的检查，直接走协议
    link.request(U.CMD_START, struct.pack('<IIB', len(bad), U.crc32(bytes(bad)), 0), timeout=5.0)
    write_all(link, bytes(bad))
    status, _ = link.request(U.CMD_FINISH, b'', timeout=2.0, allow=(0, 2))
    assert status == 2, 'Boot 应该以状态 2 拒绝，实际 %d' % status
    assert servo.state()[0] == 'boot' and servo.read_flash(0xF700, 4) == b'UPGD'
    servo.close()
    return '上位机与 Boot 两道检查都拦住了'


def scenario_power_loss(image, fw_hw, fw_ver):
    """升级中断电：上电停在 Boot，重新升级成功"""
    servo = new_servo(image)
    link = enter_boot(servo)
    link.request(U.CMD_START, struct.pack('<IIB', len(image), U.crc32(image), 0), timeout=5.0)
    for offset in range(0, 10 * U.PAGE, U.PAGE):
        link.request(U.CMD_WRITE, struct.pack('<I', offset) + image[offset:offset + U.PAGE],
                     timeout=0.3)

    servo.power_cycle()
    servo.tick(500)
    assert servo.state()[0] == 'boot', '断电后应停在 Boot'
    assert servo.read_flash(0xF700, 4) == b'UPGD'

    with quiet():
        U.upgrade(link_for(servo), 0, image, fw_hw, fw_ver, 115200, False, False)
    assert servo.read_flash(0, len(image)) == image
    assert servo.state()[0] == 'app'
    servo.close()
    return '断电后重新运行脚本即可接着升级，不需要 WCH-Link'


def scenario_lost_ack(image, fw_hw, fw_ver):
    """应答丢了：上位机重发同一页，结果照样正确"""
    servo = new_servo(image)
    flaky = DropReply(servo, drop_at=8)             # 第 8 次发送的应答丢掉
    with quiet():
        U.upgrade(link_for(flaky), 0, image, fw_hw, fw_ver, 115200, False, False)
    assert servo.read_flash(0, len(image)) == image
    assert servo.read_flash(0xF700, 4) == b'APOK'
    servo.close()
    return '丢一次应答后重发，Flash 内容仍与镜像一致'


def scenario_rescue(image, fw_hw, fw_ver):
    """救砖：APP 区是空的，靠上电握手窗口进 Boot 并刷入固件"""
    servo = new_servo(None)                         # 空白芯片，只有 Boot
    assert servo.state()[0] == 'boot', '没有 APP 时应停在 Boot'

    link = link_for(servo)
    servo.power_cycle()                             # 救砖：先上电，上位机立刻连发握手
    with quiet():
        U.upgrade(link, 0, image, fw_hw, fw_ver, 115200, True, False)
    assert servo.state()[0] == 'app'
    assert servo.read_flash(0, len(image)) == image
    assert servo.read_flash(0xF600, 4) == b'HWIF', '救回后 APP 应写入硬件信息'
    servo.close()
    return '断电上电 + 握手，把空白芯片刷成能跑的 APP'


def scenario_idle_return(image, fw_hw, fw_ver):
    """进了 Boot 又不升级：10 秒后自动回 APP"""
    servo = new_servo(image)
    enter_boot(servo)
    assert servo.state()[0] == 'boot'
    servo.tick(10000)
    assert servo.state()[0] == 'app', '空闲 10 秒后应回到 APP'
    servo.close()
    return '误进 Boot 不会卡死，10 秒后自己回来'


def scenario_erase_params(image, fw_hw, fw_ver):
    """擦除模式1：参数页一起擦掉，硬件信息页保留，升级后 ID/波特率回到出厂值"""
    servo = new_servo(image, servo_id=9)
    with quiet():
        U.upgrade(link_for(servo), 9, image, fw_hw, fw_ver, 115200, False, True)
    assert servo.read_flash(0xF400, 512) == b'\xFF' * 512, '参数页应被擦掉'
    assert servo.read_flash(0xF600, 4) == b'HWIF', '硬件信息页不该被擦'
    assert servo.state()[0] == 'app'
    servo.close()
    return '模式1 擦掉参数页，硬件信息页保留，脚本用默认 ID 确认成功'


SCENARIOS = [
    ('正常升级', scenario_normal),
    ('型号不符', scenario_mismatch),
    ('升级中断电', scenario_power_loss),
    ('应答丢失', scenario_lost_ack),
    ('救砖', scenario_rescue),
    ('空闲回退', scenario_idle_return),
    ('擦除模式1', scenario_erase_params),
]


# ============================== 驱动 ==============================

def run(scenarios, image, fw_hw, fw_ver):
    failures = 0
    for name, func in scenarios:
        try:
            print('  OK   %-12s %s' % (name, func(image, fw_hw, fw_ver)))
        except Exception as exc:                    # noqa: BLE001  仿真脚本，出什么都要报出来
            failures += 1
            print('  FAIL %-12s %s: %s' % (name, type(exc).__name__, exc))
    return failures


def load_firmware(argv):
    path = argv[0] if argv else str(DEFAULT_HEX)
    image, fw_hw, fw_ver = U.load_hex(path)
    print('固件 %s：版本 %d.%d.%d，适用硬件 %s，%d 字节'
          % (path, fw_ver[0], fw_ver[1], fw_ver[2], U.hw_text(fw_hw), len(image)))
    return image, fw_hw, fw_ver


def main(argv):
    build_sim()
    image, fw_hw, fw_ver = load_firmware(argv)
    failures = run(SCENARIOS, image, fw_hw, fw_ver)
    print('%d/%d 个场景通过' % (len(SCENARIOS) - failures, len(SCENARIOS)))
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
