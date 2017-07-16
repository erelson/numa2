#! /usr/bin/python3

import os

# Python3 minify lib that is called 
try:
    import mnfy
    HAVE_MNFY = True
except ImportError:
    HAVE_MNFY = False


SOURCES_LIST = [
        "numac_porting/ax.py",
        "bioloid3/boot.py",
        "bioloid3/bus.py",
        "numac_porting/commander.py",
        "bioloid3/device.py",
        "bioloid3/dump_mem.py",
        "numac_porting/helpers.py",
        "numac_porting/init.py",
        "numac_porting/IK.py",
        "bioloid3/log.py",
        #"numac_porting/numa.py",
        "numac_porting/numa2.py",
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


if not os.path.isdir("micropy-to-upload"):
    os.mkdir("micropy-to-upload")

for filename in SOURCES_LIST:
    newfile = os.path.join(OUTPUT_FOLDER, os.path.basename(filename))
    print("Turning {0} into {1}...".format(filename, newfile))

    try:
        # Use mnfy lib to minify
        if HAVE_MNFY:
            # Adapted from mnfy.py
            with open(filename, 'r') as source_file:
                source = source_file.read()
            import ast
            source_ast = ast.parse(source)
            # Use safe transforms
            for transform in mnfy.safe_transforms:
                transformer = transform()
                source_ast = transformer.visit(source_ast)
            minifier = mnfy.SourceCode()
            minifier.visit(source_ast)
            with open (newfile, 'w') as fw:
                fw.write(str(minifier))
        # Backup simple removal of blank lines and comments
        else:
            with open(filename, 'r') as fr, open(newfile, 'w') as fw:
                for line in fr:
                    # Check for empty lines or comment lines, and skip them
                    if not line.lstrip() or line.lstrip()[0] == '#':
                        continue
                    fw.write(line)
    except SyntaxError as e:
        print("ERROR: Syntax error when minifying {0}".format(filename))
