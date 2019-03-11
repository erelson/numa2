Overview
--------
This repos is my code for a 4 legged walking "mech" named Numa2.

Numa 2 is built to compete in the Mech Warefare competitions, which originated
at RoboGames circa 2009.

Code is micropython, which is essentially python3. However, a key source of
code smells is that this code originally was C++ code that I decided to
convert to micropython. When inspired, I refactor to improve this...

Thanks the the python3 simularity, I wrote my micropython such that key minimal
portions of hardware interaction code could be easily mocked, letting me run e.g.
the inverse kinematics code on a full computer, with outputs being plotted
in a Jupyter notebook.


Credits:
--------

Math functions for vectors adapted from WebbotLib 2.0 source code.

Bioloid micropython library by DHylands with general micropython implmentations for microcontrollers.

uCee project by JHylands building on the bioloid library and providing AX-12 specific functionality.

Python 3.x library `mnfy` mnfy.py script from which I base minification of all modules to be uploaded.
https://github.com/brettcannon/mnfy

IK code is my own, ported from C for first version of Numa.


Dependencies:
-------------
Written for micropython, which is based on python 3.4 feature set.


For minifying the code, uses `mnfy` (install with pip3)


Uploading:
----------
Steps:

1. Save everything in `numac_porting`
2. Run the `create_upload_files.py` to copy just the files needed on robot, and to reduce their size:
   `python3 create_upload_files.py`
3. Using Windows Explorer (? or better?) copy the created files in micropy-to-upload
4. ???
