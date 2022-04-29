import os
Import("env", "projenv")

def after_build_action(source, target, env):
  print(source)
  print(target)
  print(env)
  print("===== after_build_action")
  os.system("pio run -t pack")

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", after_build_action)