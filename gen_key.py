"""生成升级加密密钥(AES-128，16 个字符)，写入 txt。
    python gen_key.py                  写到下面 KEY_FILE
    python gen_key.py 路径\\boot_key.txt  写到指定位置
    python gen_key.py --force          已有密钥文件时覆盖(换密钥后，用旧密钥 Boot 的舵机将无法再升级)
生成后把 txt 里那一串原样粘贴到 CH32V00X_IAP/System/boot_key.h 的引号里，重新编译 Boot；
打包脚本 fw_pack.py 读的也是这个 txt，两边必须是同一串。
"""
# ============================== 配置 ==============================
KEY_FILE = r'boot_key.txt'          # 密钥文件；相对路径以本脚本所在目录为准，也可写绝对路径
# ==================================================================
import argparse
import os
import secrets
import sys

# 只用字母数字，去掉易混淆的 0 O 1 l I；不含引号和反斜杠，可直接放进 C 字符串
ALPHABET = 'ABCDEFGHJKLMNPQRSTUVWXYZabcdefghijkmnopqrstuvwxyz23456789'
KEY_LEN = 16


def main():
    sys.stdout.reconfigure(encoding='utf-8', errors='backslashreplace')
    ap = argparse.ArgumentParser(description='生成升级加密密钥')
    ap.add_argument('out', nargs='?', default=KEY_FILE, help='密钥 txt 路径')
    ap.add_argument('--force', action='store_true', help='覆盖已有密钥文件')
    args = ap.parse_args()

    path = args.out if os.path.isabs(args.out) else os.path.join(os.path.dirname(os.path.abspath(__file__)), args.out)
    if os.path.exists(path) and not args.force:
        print('失败：%s 已存在。换密钥会让已出货的舵机无法升级；确实要换请加 --force' % path, file=sys.stderr)
        return 1
    key = ''.join(secrets.choice(ALPHABET) for _ in range(KEY_LEN))
    with open(path, 'w', encoding='ascii', newline='\n') as f:
        f.write(key + '\n')
    print('密钥：%s\n已写入：%s\n把这一串粘贴到 boot_key.h 后重新编译 Boot；请另外备份这个 txt。' % (key, path))
    return 0


if __name__ == '__main__':
    sys.exit(main())
