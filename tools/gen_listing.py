# PlatformIO pre-build hook. Tells GCC to keep the .s intermediates and
# annotate them with the original source lines, then runs objdump after the
# link to produce a full disassembly of the firmware.
#
# Find the output under .pio/build/teensy40/ — the per-TU .s files live next
# to the .o files, and the linked disassembly is at firmware.lst.
#
# Idea borrowed from per1234/ArduinoDisassembly, adapted to PlatformIO's
# SCons setup.

Import("env")

env.Append(CCFLAGS=["-save-temps=obj", "-fverbose-asm"])

def post_build_disassemble(source, target, env):
    import subprocess, os
    elf = str(target[0])
    out = os.path.join(os.path.dirname(elf), "firmware.lst")
    # The toolchain exposes objcopy; objdump sits right next to it.
    objdump = env.subst("$OBJCOPY").replace("objcopy", "objdump")
    try:
        with open(out, "w") as f:
            subprocess.check_call([objdump, "-d", "-S", "-l", elf], stdout=f)
        print("Disassembly written to", out)
    except Exception as e:
        print("objdump failed:", e)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", post_build_disassemble)
