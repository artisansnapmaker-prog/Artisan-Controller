import sys
import os
from os.path import join
Import("env", "projenv")

def after_build_action(source, target, env):
  print("===== snapmaker pack =====")
  # os.system("pio run -t pack")
  project_dir = projenv.get("PROJECT_DIR")
  pack_script = join(project_dir, 'snapmaker', 'scripts', 'pack.py')

  PIOENV = projenv.get("PIOENV")
  if PIOENV.endswith('_boot'):
    APP_PIOENV = PIOENV[:-5]
    BOOT_PIOENV = PIOENV
  else:
    APP_PIOENV = PIOENV
    BOOT_PIOENV = PIOENV + '_boot'
      
  app_fw_bin = join(projenv.get("PROJECT_BUILD_DIR"), APP_PIOENV, projenv.get("PROGNAME") + '.bin')
  boot_fw_bin = join(projenv.get("PROJECT_BUILD_DIR"), BOOT_PIOENV, projenv.get("PROGNAME") + '.bin')
  os.system("python {0} -a {1} -b {2} -o {3}".format(pack_script, app_fw_bin, boot_fw_bin, project_dir))

env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", after_build_action)