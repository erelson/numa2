#! /usr/bin/python3

import os
from argparse import ArgumentParser

from minify_write import write_minified


BOOT_FILE = "testing/test_iteration_speed_boot.py"

SOURCES_LIST = [
        #"src/ax.py",
        #"bioloid3/bus.py",
        #"src/commander.py",
        #"bioloid3/device.py",
        #"bioloid3/dump_mem.py",
        #"src/helpers.py",
        #"src/init.py",
        "src/IK.py",
        #"bioloid3/log.py",
        "testing/test_iteration_speed.py",
        #"src/numa.py",
        #"src/numa2.py",
        #"bioloid3/packet.py",
        #"bioloid3/scan.py",
        #"bioloid3/serial_bus.py",
        #"bioloid3/serial_port.py",
        #"bioloid3/socket_port.py",
        #"bioloid3/stm_uart_port.py",
        #"bioloid3/stm_usb_port.py",
        #"src/vecmath.py",
        ]

OUTPUT_FOLDER = "micropy-to-upload"


if __name__ == "__main__":
    parser = ArgumentParser()
    parser.add_argument("-f", "--full", action="store_true",
                        help="Don't minify produced files.")

    args = parser.parse_args()

    if not os.path.isdir("micropy-to-upload"):
        os.mkdir("micropy-to-upload")

    for filename in SOURCES_LIST:
        newfile = os.path.join(OUTPUT_FOLDER, os.path.basename(filename))
        write_minified(filename, newfile, not args.full)

    newfile = os.path.join(OUTPUT_FOLDER, "boot.py")
    write_minified(BOOT_FILE, newfile, not args.full)
