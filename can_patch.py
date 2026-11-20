# Older releases of the ESP32CAN library include a deprecated header file. This script patches that:
# in src/CAN.c the header esp_intr.h is replaced with esp_intr_alloc.h
# Enable it with "extra_scripts = pre:can_patch.py" in platformio.ini.

from os.path import join, isfile

Import("env")

LIBRARY_DIR = join(env['PROJECT_LIBDEPS_DIR'], env['PIOENV'], "ESP32CAN")
SRC_PATH = join(LIBRARY_DIR, "src", "CAN.c")

# Pure Python so it behaves the same on macOS, Linux and Windows (the old "sed -I .bak"
# form only worked with the macOS version of sed). Safe to run more than once.
if isfile(SRC_PATH):
    with open(SRC_PATH) as f:
        source = f.read()

    patched = source.replace('"esp_intr.h"', '"esp_intr_alloc.h"').replace('<esp_intr.h>', '<esp_intr_alloc.h>')

    if patched != source:
        if not isfile(SRC_PATH + ".bak"):
            with open(SRC_PATH + ".bak", "w") as f:
                f.write(source)
        with open(SRC_PATH, "w") as f:
            f.write(patched)
        print("can_patch.py: patched %s" % SRC_PATH)
else:
    print("can_patch.py: %s not found yet - run the build again after the library is installed" % SRC_PATH)
