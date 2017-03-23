#!/usr/bin/python

# Copyright (C) 2014 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
# Author: Klas Kronander
# email:   klas.kronander@epfl.ch
# website: lasa.epfl.ch
#
# Permission is granted to copy, distribute, and/or modify this program
# under the terms of the GNU General Public License, version 2 or any
# later version published by the Free Software Foundation.
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details

import os
import subprocess
import glob
import sys
from shutil import copyfile

def get_install_path():
    rtk_src_path = os.path.dirname(sys.argv[0])
    rtk_src_path = os.path.dirname(rtk_src_path)
    return rtk_src_path


ode_file = 'ode-0.13.tar.bz2'


if __name__ == '__main__':
    rtk_src_path = get_install_path()
    rtk_bin_path = os.getcwd()
    #print rtk_bin_path

    if (len(sys.argv) > 1):
        if (sys.argv[1] == '--clean'):
            print 'cleaning ODE'
            print 'finished cleaning'
            exit(1)

    # download and unpack if necessary
    if glob.glob(os.path.join(rtk_src_path, ode_file)):
        print 'libode source already downloaded'
        if not glob.glob(os.path.join(rtk_bin_path, ode_file)):
            copyfile(os.path.join(rtk_src_path, ode_file), os.path.join(rtk_bin_path, ode_file))
            subprocess.call(['tar', '-xf', ode_file])
    else:
        print 'no tarball found, downloading..'
        subprocess.call([rtk_src_path + '/scripts/downloadODE.sh'])
        copyfile(os.path.join(rtk_src_path, ode_file), os.path.join(rtk_bin_path, ode_file))
        print 'Finished downloading'

    os.chdir(rtk_bin_path)
    # configure and compile if necessary
    if glob.glob(rtk_src_path + '/lib/libode.so'):
        print 'libode source already compiled'
    else:
        print 'libode not compiled. configuring..'
        subprocess.call([rtk_bin_path + '/scripts/configureODE.sh', rtk_src_path])

        print 'Finished configuring'
        print 'compiling..'
        subprocess.call([rtk_bin_path + '/scripts/compileODE.sh', rtk_src_path])

