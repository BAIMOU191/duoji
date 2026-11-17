"""舵机升级文件打包：APP hex -> 加密升级文件(<hex名>_upgrade.bin，与 hex 同目录)。只在出固件的电脑上用，不发给客户。
    python fw_pack.py                                  用下面配置里的 hex 和密钥
    python fw_pack.py 路径\\app.hex 路径\\boot_key.txt    运行时指定两个文件
    python fw_pack.py --hex app.hex --key boot_key.txt [--out xxx.bin]   同上，带参数名
密钥 txt 由 gen_key.py 生成，必须与 Boot 的 boot_key.h 里是同一串 16 个字符。

升级文件格式(小端)：
    0   'SVUP'
    4   格式版本 1
    5   适用硬件 4 字节(舵机类型/硬件版本/电压/扭力，0xFF=不限)  -- 取自固件信息块
    9   固件版本 3 字节
    12  镜像长度 u32(明文，256 字节整数倍，从 APP_BASE 起)
    16  镜像 CRC32 u32(明文，Boot 写完后核对)
    20  IV 12 字节(每次打包随机生成)
    32  密文(AES-128-CTR：第 n 个 16 字节块的计数器 = IV || n 大端32位)
    末尾 文件 CRC32 u32(覆盖前面全部字节，升级工具检查文件是否损坏)
密钥丢失后，已出货的舵机无法再升级，请另外备份密钥 txt。
"""
# ============================== 配置 ==============================
# 相对路径以本脚本所在目录为准，也可写绝对路径；命令行给了路径时以命令行为准
HEX_FILE = r'CH32V00X_APP_ADC\obj\CH32V00X_APP.hex'   # 要打包的 APP hex
KEY_FILE = r'boot_key.txt'                            # 密钥 txt(gen_key.py 生成)
# ==================================================================
import argparse
import os
import struct
import sys
import zlib

ROOT = os.path.dirname(os.path.abspath(__file__))

APP_BASE, APP_END, PAGE, FW_INFO_OFFSET = 0x1400, 0xF400, 256, 0x100   # 与 Boot iap.h 一致
MAGIC, FORMAT = b'SVUP', 1


class Fail(Exception):
    pass


# ------------------------------------------------------------------ AES-128(标准实现，只用加密方向)

def _sbox():
    box, p, q = [0] * 256, 1, 1
    while True:                                   # 在 GF(2^8) 上按生成元 3 遍历，求逆元后做仿射变换
        p = p ^ ((p << 1) & 0xFF) ^ (0x1B if p & 0x80 else 0)
        q ^= q << 1; q ^= q << 2; q ^= q << 4; q &= 0xFF
        if q & 0x80: q ^= 0x09
        x = q ^ (q << 1 | q >> 7) & 0xFF ^ (q << 2 | q >> 6) & 0xFF ^ (q << 3 | q >> 5) & 0xFF ^ (q << 4 | q >> 4) & 0xFF
        box[p] = (x ^ 0x63) & 0xFF
        if p == 1:
            break
    box[0] = 0x63
    return box


SBOX = _sbox()


def _xt(b):
    return ((b << 1) ^ (0x1B if b & 0x80 else 0)) & 0xFF


def expand_key(key):
    w, rc = [list(key[i:i + 4]) for i in range(0, 16, 4)], 1
    for i in range(4, 44):
        t = list(w[i - 1])
        if i % 4 == 0:
            t = [SBOX[t[1]] ^ rc, SBOX[t[2]], SBOX[t[3]], SBOX[t[0]]]
            rc = _xt(rc)
        w.append([w[i - 4][j] ^ t[j] for j in range(4)])
    return [sum(w[4 * r:4 * r + 4], []) for r in range(11)]


def aes_encrypt(rks, block):
    s = [b ^ k for b, k in zip(block, rks[0])]
    for r in range(1, 11):
        s = [SBOX[s[(i + 4 * (i % 4)) % 16]] for i in range(16)]          # SubBytes + ShiftRows
        if r < 10:
            m = []
            for c in range(4):                                             # MixColumns
                a = s[4 * c:4 * c + 4]
                m += [_xt(a[0]) ^ _xt(a[1]) ^ a[1] ^ a[2] ^ a[3],
                      a[0] ^ _xt(a[1]) ^ _xt(a[2]) ^ a[2] ^ a[3],
                      a[0] ^ a[1] ^ _xt(a[2]) ^ _xt(a[3]) ^ a[3],
                      _xt(a[0]) ^ a[0] ^ a[1] ^ a[2] ^ _xt(a[3])]
            s = m
        s = [b ^ k for b, k in zip(s, rks[r])]
    return bytes(s)


def ctr_crypt(key, iv, data):
    rks, out = expand_key(key), bytearray(data)
    for n in range(len(data) // 16):
        ks = aes_encrypt(rks, iv + struct.pack('>I', n))
        for i in range(16):
            out[16 * n + i] ^= ks[i]
    return bytes(out)


def self_test():
    """FIPS-197 附录 C.1 标准向量，确认实现没错"""
    ct = aes_encrypt(expand_key(bytes(range(16))), bytes.fromhex('00112233445566778899aabbccddeeff'))
    if ct.hex() != '69c4e0d86a7b0430d8cdb78070b4c55a':
        raise Fail('AES 自检失败')


# ------------------------------------------------------------------ hex

def load_hex(path):
    """解析 APP hex，返回 (从 APP_BASE 起的镜像, 适用硬件4项, 版本3项)"""
    cells, base = {}, 0
    try:
        text = open(path, 'r', encoding='ascii', errors='ignore').read()
    except OSError as exc:
        raise Fail('打不开 hex：%s' % exc)
    for line in text.splitlines():
        line = line.strip()
        if not line.startswith(':'):
            continue
        raw = bytes.fromhex(line[1:])
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
    low, top = min(cells), max(cells)
    if low < APP_BASE:
        raise Fail('hex 从 0x%X 开始，不是从 APP_BASE(0x%X) 链接的 APP(旧分区的固件或 Boot？)' % (low, APP_BASE))
    if top >= APP_END:
        raise Fail('固件数据到了 0x%X，超出 APP 区(0x%X)' % (top, APP_END))
    size = ((top + 1 - APP_BASE + PAGE - 1) // PAGE) * PAGE
    image = bytearray(b'\xFF' * size)
    for addr, value in cells.items():
        image[addr - APP_BASE] = value
    info = image[FW_INFO_OFFSET:FW_INFO_OFFSET + 16]
    if info[:4] != b'SVFW':
        raise Fail('固件里没有信息块(APP 偏移 0x100 不是 SVFW)')
    return bytes(image), bytes(info[4:8]), bytes(info[8:11])


# ------------------------------------------------------------------ 命令

def resolve(path):
    return path if os.path.isabs(path) else os.path.join(ROOT, path)


def load_key(path):
    """读密钥 txt：第一行去掉首尾空白后应为 16 个 ASCII 字符"""
    try:
        text = open(path, 'r', encoding='ascii').read().strip()
    except OSError:
        raise Fail('找不到密钥文件 %s，先运行 python gen_key.py' % path)
    except UnicodeDecodeError:
        raise Fail('密钥文件里有非 ASCII 字符：%s' % path)
    if len(text) != 16 or any(c in text for c in '"\\') or not text.isprintable():
        raise Fail('密钥必须是 16 个可见字符(不含引号和反斜杠)，现在是 %d 个：%s' % (len(text), path))
    return text.encode('ascii')


def pack(hex_path, key_path, out_path):
    key = load_key(key_path)
    image, hw, ver = load_hex(hex_path)
    iv = os.urandom(12)
    head = MAGIC + bytes([FORMAT]) + hw + ver + struct.pack('<II', len(image), zlib.crc32(image) & 0xFFFFFFFF) + iv
    body = head + ctr_crypt(key, iv, image)
    body += struct.pack('<I', zlib.crc32(body) & 0xFFFFFFFF)
    if out_path is None:
        out_path = os.path.splitext(hex_path)[0] + '_upgrade.bin'
    with open(out_path, 'wb') as f:
        f.write(body)
    print('已打包：%s\n  固件 %d.%d.%d，适用硬件 %s，镜像 %d 字节(%d 页)\n  hex：%s\n  密钥：%s'
          % (out_path, *ver, '/'.join('不限' if v == 0xFF else str(v) for v in hw), len(image), len(image) // PAGE,
             hex_path, key_path))


def main():
    sys.stdout.reconfigure(encoding='utf-8', errors='backslashreplace')
    ap = argparse.ArgumentParser(description='舵机升级文件打包(AES-128-CTR)')
    ap.add_argument('hex_pos', nargs='?', metavar='HEX', help='APP hex 路径(不给用配置里的 HEX_FILE)')
    ap.add_argument('key_pos', nargs='?', metavar='KEY', help='密钥 txt 路径(不给用配置里的 KEY_FILE)')
    ap.add_argument('--hex', default=None, help='同 HEX')
    ap.add_argument('--key', default=None, help='同 KEY')
    ap.add_argument('--out', default=None, help='输出路径，默认 <hex 同名>_upgrade.bin')
    args = ap.parse_args()
    try:
        self_test()
        hex_path = resolve(args.hex or args.hex_pos or HEX_FILE)
        key_path = resolve(args.key or args.key_pos or KEY_FILE)
        pack(hex_path, key_path, args.out)
        return 0
    except Fail as exc:
        print('失败：%s' % exc, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
