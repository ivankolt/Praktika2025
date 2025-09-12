import os
from SCons.Script import DefaultEnvironment

env = DefaultEnvironment()

def gen_bin(source, target, env):
    elf = str(target[0])
    bin_path = os.path.join(env.subst("$BUILD_DIR"), "firmware.bin")
    env.Execute("$OBJCOPY -O binary %s %s" % (elf, bin_path))
    print("BIN saved:", bin_path)

env.AddPostAction("$BUILD_DIR/${PROGNAME}.elf", gen_bin)


