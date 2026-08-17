import sys
import os
import re, datetime
from os.path import join
Import("env", "projenv")

def pack_raw_bootloader_app(source, target, env):
  print("===== snapmaker pack =====")
  # os.system("pio run -t pack")
  project_dir = projenv.get("PROJECT_DIR")
  pack_script = join(project_dir, 'snapmaker', 'scripts', 'pack_for_programming.py')

  release_dir = join(project_dir, "release")
  if not os.path.exists(release_dir):
    os.mkdir(release_dir)

  PIOENV = projenv.get("PIOENV")
  if PIOENV.endswith('_boot'):
    APP_PIOENV = PIOENV[:-5]
    BOOT_PIOENV = PIOENV
  else:
    APP_PIOENV = PIOENV
    BOOT_PIOENV = PIOENV + '_boot'

  app_fw_bin = join(projenv.get("PROJECT_BUILD_DIR"), APP_PIOENV, projenv.get("PROGNAME") + '.bin')
  boot_fw_bin = join(projenv.get("PROJECT_BUILD_DIR"), BOOT_PIOENV, projenv.get("PROGNAME") + '.bin')
  os.system("python {0} -a {1} -b {2} -o {3}".format(pack_script, app_fw_bin, boot_fw_bin, release_dir))


def pack_minor_app(source, target, env):
  PIOENV = projenv.get("PIOENV")
  if PIOENV.endswith('_boot'):
    print("You are building bootloader, won't package bin for HMI")
    return

  project_dir = projenv.get("PROJECT_DIR")
  release_dir = join(project_dir, "release")
  if not os.path.exists(release_dir):
    os.mkdir(release_dir)

  # get version from Marlin\src\inc\Version.h
  with open(join(project_dir, 'Marlin', 'src', 'inc','Version.h'), 'r', encoding='utf-8') as version_file:
      lines = version_file.readlines()

  version = None
  pattern = r"V\d+\.\d+\.\d+"
  for line in lines:
      match_obj = re.search(pattern, line, re.I)
      if match_obj:
          version = match_obj[0]
          break

  if not version:
    print("cannot get app version from Marlin\src\inc\Version.h")
    print("won't use default version: V0.0.0-2201")
    version = "V0.0.0"
  else:
    print("got app version: {}".format(version))

  app_fw_bin = join(projenv.get("PROJECT_BUILD_DIR"), PIOENV, projenv.get("PROGNAME") + '.bin')

  date = datetime.datetime.today().strftime('%Y%m%d')
  minor_bin = join(release_dir, "A400_MC_{}_{}.bin".format(version, date))
  if os.path.exists(minor_bin):
    try:
      os.remove("{}.old".format(minor_bin))
    except Exception:
      pass
    os.rename(minor_bin, "{}.old".format(minor_bin))

  print("app raw bin: {}".format(app_fw_bin))
  print("min bin name: {}".format(minor_bin))
  # tools\ota_python\gen_header.py
  pack_script = join(project_dir, 'tools', 'ota_python', 'gen_header.py')
  os.system("python {} -t 2 -f {} -c 1 -v {} -o {}".format(pack_script, app_fw_bin, version, minor_bin))


def pack_major_app(source, target, env):
  PIOENV = projenv.get("PIOENV")
  if PIOENV.endswith('_boot'):
    print("You are building bootloader, won't package bin for HMI")
    return

  project_dir = projenv.get("PROJECT_DIR")
  release_dir = join(project_dir, "release")
  if not os.path.exists(release_dir):
    print("release dir not found! please make minor image firstly!!!")
    return

  # get version from Marlin\src\inc\Version.h
  with open(join(project_dir, 'Marlin', 'src', 'inc','Version.h'), 'r', encoding='utf-8') as version_file:
      lines = version_file.readlines()

  version = None
  pattern = r"V\d+\.\d+\.\d+"
  for line in lines:
      match_obj = re.search(pattern, line, re.I)
      if match_obj:
          version = match_obj[0]
          break

  if not version:
    print("cannot get app version from Marlin\src\inc\Version.h")
    print("won't use default version: V0.0.0-2201")
    version = "V0.0.0"
  else:
    print("got app version: {}".format(version))

  date = datetime.datetime.today().strftime('%Y%m%d')
  minor_bin = join(release_dir, "A400_MC_{}_{}.bin".format(version, date))
  if not os.path.exists(minor_bin):
    print("cannot found minor image: {} !!!".format(minor_bin))
    print("please make minor inmage firstly!!!")
    return

  pack_script = join(project_dir, 'snapmaker', 'scripts', 'pack_for_hmi.py')
  os.system("python {} -c {} -v {}".format(pack_script, minor_bin, version))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", pack_raw_bootloader_app)
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", pack_minor_app)
env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", pack_major_app)


if __name__ == "__main__":
  pack_raw_bootloader_app()
  pack_minor_app()
  pack_major_app()
