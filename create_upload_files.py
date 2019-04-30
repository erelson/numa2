#! /usr/bin/python3

import hashlib
import os

from argparse import ArgumentParser
from minify_write import write_minified
from shutil import copyfile


BOOT_FILE = "numac_porting/boot.py"

SOURCES_LIST = [
        "numac_porting/ax.py",
        "numac_porting/commander.py",
        "numac_porting/helpers.py",
        "numac_porting/init.py",
        "numac_porting/IK.py",
        "numac_porting/poses.py",
        "numac_porting/MotorDriver.py",
        "numac_porting/numa.py",
        #"numac_porting/numa2.py",
        "bioloid3/bus.py",
        "bioloid3/device.py",
        "bioloid3/dump_mem.py",
        "bioloid3/log.py",
        "bioloid3/packet.py",
        "bioloid3/scan.py",
        "bioloid3/serial_bus.py",
        "bioloid3/serial_port.py",
        "bioloid3/socket_port.py",
        "bioloid3/stm_uart_port.py",
        "bioloid3/stm_usb_port.py",
        #"numac_porting/vecmath.py",
        ]

OUTPUT_FOLDER = "micropy-to-upload"

def get_hash(filepath):
    if not os.path.isfile(filepath):
        return None
    with open(filepath, 'r') as fr:
        text = fr.read()#.decode("utf-8")
    return hashlib.md5(text.encode("utf-8")).hexdigest()


def main(args):
    changed_files = []

    if not os.path.isdir("micropy-to-upload"):
        os.mkdir("micropy-to-upload")

    for filename in SOURCES_LIST:
        newfile = os.path.join(OUTPUT_FOLDER, os.path.basename(filename))
        old_hash = get_hash(newfile)
        write_minified(filename, newfile, not args.full)
        new_hash = get_hash(newfile)
        if old_hash != new_hash:
            changed_files.append(newfile)

    newfile = os.path.join(OUTPUT_FOLDER, "boot.py")
    write_minified(BOOT_FILE, newfile, not args.full)

    print("\nThe following files changed:", changed_files)

    if not args.write:
        return

    if not os.path.exists(args.write_drive):
        print("Target location {0} does not exist. Exiting.".format(args.write_drive))
        return

    # Replace files on pyboard with those we've changed
    for newfile in changed_files:
        target = os.path.join(args.write_drive, os.path.basename(newfile))
        if os.path.isfile(target):
            os.remove(target)
            copyfile(newfile, target)


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-f", "--full", action="store_true",
                        help="Don't minify produced files.")
    parser.add_argument("-w", "--write", action="store_true",
                        help="Write changed files to pyboard.")
    parser.add_argument("-d", "--write-drive", action="store", default="T:",
                        help="Target drive to put files")

    args = parser.parse_args()

    main(args)
