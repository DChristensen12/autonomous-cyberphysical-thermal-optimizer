#!/usr/bin/env bash
# One shot environment setup for ACPTO. Installs the PlatformIO command line
# tool, fixes the known filename casing issues, and does a first build so the
# Teensy framework downloads and the editor stops complaining about missing
# headers. Run it once with: bash setup.sh

set -e  # stop on the first error so we are not chasing a half done setup

echo "==> checking for python and pip"
# PlatformIO installs as a python package. The official installer script is the
# supported path, the apt package is old and tends to break.
if ! command -v python3 >/dev/null 2>&1; then
    echo "python3 is missing. install it first with: sudo apt install python3 python3-venv"
    exit 1
fi

echo "==> installing the PlatformIO core (pio command)"
# This is the official installer. It drops pio into ~/.platformio/penv/bin.
python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py)"

# Make pio reachable in this shell session. The installer puts it here.
export PATH="$PATH:$HOME/.platformio/penv/bin"

echo "==> fixing filename casing in sim/ and test/"
# The Makefiles expect lowercase names. Rename only if the capitalized ones
# exist, so re-running this script does not error out.
[ -f sim/Sim_main.cpp ] && mv sim/Sim_main.cpp sim/sim_main.cpp
[ -f test/Test_units.cpp ] && mv test/Test_units.cpp test/test_units.cpp
# Drop the old Unity test we replaced ages ago.
[ -f test/test_gp_native.cpp ] && rm test/test_gp_native.cpp

echo "==> first build, this downloads the Teensy platform and framework"
# This is the step that pulls IntervalTimer.h and the rest into place, which
# is what clears the red squiggles in the editor.
"$HOME/.platformio/penv/bin/pio" run -e teensy40

echo
echo "==> done"
echo "pio now lives at ~/.platformio/penv/bin/pio"
echo "to use pio in any new terminal, add this line to your ~/.bashrc:"
echo '  export PATH="$PATH:$HOME/.platformio/penv/bin"'
