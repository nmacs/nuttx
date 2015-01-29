#!/bin/bash
# configs/omega/nsh/setenv.sh
#

if [ "$_" = "$0" ] ; then
  echo "You must source this script, not run it!" 1>&2
  exit 1
fi

WD=`pwd`
if [ ! -x "setenv.sh" ]; then
  echo "This script must be executed from the top-level NuttX build directory"
  exit 1
fi

if [ -z "${PATH_ORIG}" ]; then
  export PATH_ORIG="${PATH}"
fi

# This is the Cygwin path to the location where I installed the CodeSourcery
# toolchain under windows.  You will also have to edit this if you install
# the CodeSourcery toolchain in any other location
#export TOOLCHAIN_BIN="/cygdrive/c/Program Files (x86)/CodeSourcery/Sourcery G++ Lite/bin"

# These are the Cygwin paths to the locations where I installed the Atollic
# toolchain under windows.  You will also have to edit this if you install
# the Atollic toolchain in any other location.  /usr/bin is added before
# the Atollic bin path because there is are binaries named gcc.exe and g++.exe
# at those locations as well.
#export TOOLCHAIN_BIN="/usr/bin:/cygdrive/c/Program Files (x86)/Atollic/TrueSTUDIO for ARM Pro 2.3.0/ARMTools/bin"
#export TOOLCHAIN_BIN="/usr/bin:/cygdrive/c/Program Files (x86)/Atollic/TrueSTUDIO for STMicroelectronics STM32 Lite 2.3.0/ARMTools/bin"

# This is the Cygwin path to the location where I build the buildroot
# toolchain.
export TOOLCHAIN_BIN="${WD}/../misc/buildroot/build_arm/staging_dir/bin"

# The colibri/tools directory
export TOOL_DIR="${WD}/configs/omega/tools"

# Add the path to the toolchain and tools directory to the PATH varialble
export PATH="${TOOLCHAIN_BIN}:${TOOL_DIR}:/sbin:/usr/sbin:${PATH_ORIG}"

echo "PATH : ${PATH}"
