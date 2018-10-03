# !/usr/bin/python

import os, sys

final_so_file = os.environ['GAMS_ROOT'] + "/lib/gams.so"
final_dll_file = os.environ['GAMS_ROOT'] + "/lib/gams.dll"
source_so_file = os.environ['GAMS_ROOT'] + "/lib/libgams.pyd.so"
source_dll_file = os.environ['GAMS_ROOT'] + "/lib/gams.pyd.dll"
source_lib_file = os.environ['GAMS_ROOT'] + "/lib/gams.pyds.lib"
# init_file = os.environ['GAMS_ROOT'] + "/lib/__init__.py"

# delete any lib/gams.pyd if it existed
if os.path.isfile(final_so_file):
  os.remove(final_so_file)

elif os.path.isfile(final_dll_file):
  os.remove(final_dll_file)

if os.path.isfile(source_so_file):
  os.rename(source_so_file, final_so_file)

elif os.path.isfile(source_dll_file):
  os.rename(source_dll_file, final_dll_file)

elif os.path.isfile(source_lib_file):
  final_file = source_lib_file
  final_file.replace ('.pyd', '')
  os.rename(source_lib_file, final_file)

#if os.path.isfile(final_file):
#  file = open(init_file, 'w')
#  if file:
#    file.close()
