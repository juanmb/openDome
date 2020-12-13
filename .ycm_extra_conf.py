"""
YouCompleteMe "ycm_extra_conf.py" for Platformio based projects.

The script calls [platformio -v run -t idedata] and process the output
to fill list with the SAME SET OF FLAGS used by platformio for the
project (compiler flags, Include dirs, compiler DEFINES, etc)

Tested on the following development environment:
    - Linux (LMDE 2)
    - PlatformIO, version 3.3.0a1
    - VIM 8.0
    - YCM 2016-12-22

This file should be copied in the "base" project dir (one ycm_extra_conf.py
for every project)

Installation:
 Copy this scrit in the project base dir (together with platformio.ini, etc)
 NOTE:
   make sure you've already run "platformio init --board xxx" in the project dir

Usage
    if everything works as expected you just call VIM to edit projet files

    To test the script behaviour, run it as a standalone script:
        phython .ycm_extra_conf
    It outputs the FLAGS that will be handle back to YCM

    Ideally You don't need to edit this file to add extra flags but,
    in case you need it, add them to the "flags" list at the beginning of
    this script


Lucabuka / 2016-12-27
juanmb / 2020-12-13

Based on Anthony Ford <github.com/ajford>ajford/.ycm_extra_conf.py
    Based on the `.ycm_extra_conf.py` by @ladislas in his Bare-Arduino-Project.

Based on the `neomake-platformio.py` by github -> coddingtonbear/neomake-platformio

"""

# This is free and unencumbered software released into the public domain.
#
# Anyone is free to copy, modify, publish, use, compile, sell, or
# distribute this software, either in source code form or as a compiled
# binary, for any purpose, commercial or non-commercial, and by any
# means.
#
# In jurisdictions that recognize copyright laws, the author or authors
# of this software dedicate any and all copyright interest in the
# software to the public domain. We make this dedication for the benefit
# of the public at large and to the detriment of our heirs and
# successors. We intend this dedication to be an overt act of
# relinquishment in perpetuity of all present and future rights to this
# software under copyright law.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
# EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
# IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
# OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.
#
# For more information, please refer to <http://unlicense.org/>

import os
import subprocess
import logging
import json
try:
    import ycm_core
except ImportError:
    pass

# Logger for additional logging.
# To enable debug logging, add `let g:ycm_server_log_level = 'debug'` to
# your .vimrc file.
logger = logging.getLogger('ycm-extra-conf')


# Add here any extra flag not automatically provided by "platformio run -t ide"
# usually You don't need to 
flags = [
    # General flags
    # You 100% do NOT need -DUSE_CLANG_COMPLETER in your flags; only the YCM
    # source code needs it.
    '-DUSE_CLANG_COMPLETER'
    # ,'-MMD -DUSB_VID=null'
    # ,'-DUSB_PID=null'
]


def uniq(seq):
    seen = set()
    seen_add = seen.add
    return [x for x in seq if not (x in seen or seen_add(x))]


def get_idestate(path):
    """Calls [pio -f run -t idedata] to get compiler flags from PlatformIO env """

    cmd = ('pio', '-f', '-c', 'vim', 'run', '-t', 'idedata', '-d', path)
    out = subprocess.check_output(cmd, universal_newlines=True)

    envs = []
    for line in out.splitlines():
        if line.startswith('{'):
            envs.append(json.loads(line))

    return envs


def get_platformio_environment(path):
    """Generate the complete flags list (-I, -D, ...)"""

    envs = get_idestate(path)

    if len(envs) < 1:
        return ["ERROR: get_idestate() returns -1"]

    env = envs[0]

    _includes = []
    for k, inc in env['includes'].items():
        _includes.extend(inc)

    _cxx_path  = env['cxx_path']
    _cxx_flags = env['cxx_flags']
    _cc_path   = env['cc_path']
    _cc_flags  = env['cc_flags']
    _defines   = env['defines']
    _lisbsource_dirs  = env['libsource_dirs']

    # ADD to _cxx_flags symbols found only in "_defines"
    new_cxx_flags = _cxx_flags.split()
    for define in _defines:
        if _cxx_flags.find(define) == -1:
            # not found -> add -d<define> to cxx_flags
            new_cxx_flags.append('-D' + define)

    # insert into "includes" the working dir and ".pioenvs" (Platformio Autogen libs)
    ## Platformio automatically copies over the libs you use after your first run.
    ## Be warned that you will not receive autocompletion on libraries until after
    ## your first `platformio run`.

    for d in ['src', '.pioenvs']:
        _includes.insert(0, os.path.join(path, d))

    # Create "-I<include_file>" list
    inc_list = ['-I' + i for i in uniq(_includes)]

    return flags + new_cxx_flags + inc_list


def FlagsForFile(filename, **kwargs ):
    path = os.path.dirname(os.path.abspath( __file__))
    all_flags = get_platformio_environment(path)

    logger.debug("List of FLAGS hand back to YCM:")
    for flag in all_flags:
        logger.debug(flag)

    return {'flags': all_flags, 'do_cache': True}


# Used to TEST module output (Executd only if the module is used as a script)
if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        cwd = sys.argv[1]
    else:
        cwd = os.path.dirname(os.path.abspath(__file__))

    all_flags = get_platformio_environment(cwd)

    for a in all_flags:
        print(a)
