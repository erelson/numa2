# Python3 minify lib that is called 
try:
    import mnfy
    HAVE_MNFY = True
except ImportError:
    HAVE_MNFY = False


def write_minified(infile, outfile, minify=True):
    
    print("Turning {0} into {1}...".format(infile, outfile))

    try:
        if not minify:
            print("Not minifying.")
            with open(infile, 'r') as source_file:
                source = source_file.read()
            with open (outfile, 'w') as fw:
                fw.write(source)
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
        # Backup simple removal of blank lines and comments
        else:
            print("Using simple minification")
            with open(infile, 'r') as fr, open(outfile, 'w') as fw:
                for line in fr:
                    # Check for empty lines or comment lines, and skip them
                    if not line.lstrip() or line.lstrip()[0] == '#':
                        continue
                    fw.write(line)
    except SyntaxError as e:
        print("ERROR: Syntax error when minifying {0}".format(infile))
