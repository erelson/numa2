# Python3 minify lib that is called 
try:
    import mnfy  # Only supports python3.4, nothing newer; no longer maintained
    HAVE_MNFY = True
except ImportError:
    HAVE_MNFY = False


def write_minified(infile, outfile, minify=True, readable=False):
    
    print("Turning {0} into {1}...".format(infile, outfile))

    try:
        if not minify:
            print("Not minifying.")
            with open(infile, 'r') as source_file:
                source = source_file.read()
            with open (outfile, 'w') as fw:
                fw.write(source)
        elif minify and readable:
            print("Using simple readable minification")
            with open(infile, 'r') as fr, open(outfile, 'w') as fw:
                for line in fr:
                    # Check for empty lines or comment lines, and skip them
                    if not line.lstrip() or line.lstrip()[0] == '#':
                        continue
                    fw.write(line)
        # Use mnfy lib to minify
        elif HAVE_MNFY:
            print("Using mnfy library!")
            # Adapted from mnfy.py
            with open(infile, 'r') as source_file:
                source = source_file.read()
            import ast
            source_ast = ast.parse(source)
            # Use safe transforms
            for transform in mnfy.safe_transforms:
                transformer = transform()
                source_ast = transformer.visit(source_ast)
            minifier = mnfy.SourceCode()
            minifier.visit(source_ast)
            with open (outfile, 'w') as fw:
                fw.write(str(minifier))
        # More aggressive simple removal of blank lines and comments; combines some continued lines
        else:
            print("Using simple minification")
            with open(infile, 'r') as fr, open(outfile, 'w') as fw:
                commabuffer = ""
                for line in fr:
                    # Check for empty lines or comment lines, and skip them
                    if not line.lstrip() or line.lstrip()[0] == '#':
                        continue

                    if commabuffer:
                        commabuffer += line.strip()
                        #if commabuffer[-1] != ",":
                        if "#" in line or commabuffer[-1] not in "[(,":
                            fw.write(commabuffer + "\n")
                            commabuffer = ""
                    #elif line.rstrip()[-1] == ",":
                    elif "#" in line:
                        fw.write(line)
                        commabuffer = ""
                    elif line.rstrip()[-1] in "[(,":
                        commabuffer = line.rstrip()
                    else:
                        fw.write(line)
                        commabuffer = ""
                if commabuffer:
                    fw.write(commabuffer)

    except SyntaxError as e:
        print("ERROR: Syntax error when minifying {0}".format(infile))
