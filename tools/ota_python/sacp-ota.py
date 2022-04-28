import serial
import time
import argparse
import fsm

PEER_ID = 2
BAUDRATE = 115200
# ================================================================================================
parser = argparse.ArgumentParser(description="sacp upgrade")
parser.add_argument('--com', '-p', help='specify the serial port')
parser.add_argument('--baud', '-b', help='baudrate')
parser.add_argument('--file', '-f', help='file')
args = parser.parse_args()
COM = args.com
try: 
  BAUDRATE = int(args.baud)
except:
  pass

# ================================================================================================
ser = serial.Serial(COM, BAUDRATE, timeout=0.5)
sf = fsm.fsm()
seq = 0
bin_file = args.file

def time_after(a, b):
  return int(b) - int(a) < 0

#####################################################
OTA_MAGIC = 0x534E4150
RET_OK = 0x00
RET_ERROR = 0x01
CMD_GET_VER = 0x11
CMD_OTA_CMD_SET = 0xAD
CMD_OTA_START = 0x01
CMD_OTA_TRNAS = 0x02
CMD_OTA_FINISH = 0x03

def get_seq():
  global seq
  ret = seq
  seq += 1
  seq = seq & 0xff
  return ret

#####################################################
def clear_ser(ser):
  while True:
    c = ser.read()
    if 0 == len(c):
      return
    else:
      print("clear %d" + c)
    
def get_frame(ser):
  start_tick_ms = int(round(time.time() * 1000))
  while True:
    c = ser.read()
    if len(c):
      if sf.push_char(c[0]):
        # frame_str = " ".join(["{:02x}".format(x) for x in sf.frame])
        # print("RX : " + frame_str)
        return sf.frame
    cur = int(round(time.time() * 1000))
    if time_after(cur, start_tick_ms + 4000):
      print("timeout")
      return None

def _4_byte_xor(data):
  l = int(len(data)>>2)
  if len(data)%4:
    cutoff = data[l*4:]
    while True:
      if len(cutoff) < 4:
        cutoff.extend(0)
      else:
        break
  else:
    cutoff = None

  ret = bytearray(4)
  for i in range(l):
    ret[0] ^= data[i*4+0]
    ret[1] ^= data[i*4+1]
    ret[2] ^= data[i*4+2]
    ret[3] ^= data[i*4+3]
  if cutoff:
    ret[0] ^= cutoff[0]
    ret[1] ^= cutoff[1]
    ret[2] ^= cutoff[2]
    ret[3] ^= cutoff[3]
  return ret    

def reset_target(ser):
  pl = bytearray()
  pl.append(0x01)
  pl.append(0x03)
  frame = sf.build_frame(pl, PEER_ID)
  frame_str = " ".join(["{:02x}".format(x) for x in frame])
  print("TX: " + frame_str)
  ser.write(frame)

def ota_start(ser, bin_data):
  pl = bytearray()
  pl.append(CMD_OTA_CMD_SET)
  pl.append(CMD_OTA_START)
  pl.append(0x00)
  pl.append(0x01)
  pl.extend(bin_data[0:256])
  frame = sf.build_frame(pl, PEER_ID)
  frame_str = " ".join(["{:02x}".format(x) for x in frame])
  print("TX: " + frame_str)
  ser.write(frame)
  
  ret_frame = get_frame(ser)
  if None == ret_frame:
    return False

  cmd_set, cmd_id, ret_code = sf.pars_ret_frame(ret_frame)
  if cmd_set != CMD_OTA_CMD_SET:
    print("ota start ack code error")
    return False
  
  if cmd_id != CMD_OTA_START:
    print("ota start ack code error")
    return False

  if ret_code != RET_OK:
    print("ota start return %d" % ret_code)
    return False
  
  return True

def ota_trans(ser, offset, block):
  pl = bytearray()
  pl.append(CMD_OTA_CMD_SET)
  pl.append(CMD_OTA_TRNAS)
  pl.append(0)
  pl.extend(offset.to_bytes(4, 'little'))
  pl.extend(len(block).to_bytes(2, 'little'))
  pl.extend(block)
  frame = sf.build_frame(pl, PEER_ID, 1)
  # frame_str = " ".join(["{:02x}".format(x) for x in frame])
  # print(frame_str)
  ser.write(frame)
  return None

def ota_finish(ser, ret):
  pl = bytearray()
  pl.append(CMD_OTA_CMD_SET)
  pl.append(CMD_OTA_FINISH)
  pl.append(ret & 0xFF)
  frame = sf.build_frame(pl, PEER_ID, 1)
  # frame_str = " ".join(["{:02x}".format(x) for x in frame])
  # print(frame_str)
  ser.write(frame)
  return None

def ota(ser, bin_data):
  bin_size = len(bin_data) - 256
  start_time = time.time()
  if True != ota_start(ser, bin_data):
    return False

  time_out_cnt = 0
  while True:
    ret_code = RET_ERROR
    ret_frame = get_frame(ser)
    if None == ret_frame:
      break
      
    cmd_set, cmd_id, ret_code = sf.pars_ret_frame(ret_frame)
    if cmd_set != CMD_OTA_CMD_SET:
      continue
    
    if cmd_id == CMD_OTA_TRNAS:
      req_offset = int.from_bytes(ret_frame[13:17], 'little')
      block_size = int.from_bytes(ret_frame[17:19], 'little')
      print("req offset %d, block size %d, progress %f%%, time escape %d second" % (req_offset, block_size, 100 * req_offset/bin_size, time.time() - start_time))
      block = bin_data[256 + req_offset : 256 + req_offset + block_size]
      ota_trans(ser, req_offset, block)
      
    elif cmd_id == CMD_OTA_FINISH:
      ota_finish(ser, ret_code)
      break

  if ret_code == RET_OK:
    return True
  else:
    return False

app_file = "A400_MC.bin.pack"
esp32_file = "esp32-cam-demo.bin.pack"
sm2_file = "CNC_firmware.bin.pack"

f = open(app_file, 'rb')
app_bin_data = f.read()

f = open(esp32_file, 'rb')
esp32_bin_data = f.read()

f = open(sm2_file, 'rb')
sm2_bin_data = f.read()

success_cnt = 0
failed_cnt = 0

while True:

  # in boot upgrade app
  # if ota(ser, app_bin_data):
  #   print("boot upgrade application seccussful!")
  #   success_cnt += 1
  # else:
  #   print("boot upgrade application failed")
  #   failed_cnt += 1
  # time.sleep(10)
  
  # # in app upgrade app
  # if ota(ser, app_bin_data):
  #   print("app upgrade application seccussful!")
  #   success_cnt += 1
  # else:
  #   print("app upgrade application failed")
  #   failed_cnt += 1
  # time.sleep(10)

  # in app upgrade esp32
  # if ota(ser, esp32_bin_data):
  #   print("app upgrade esp32 seccussful!")
  #   success_cnt += 1
  # else:
  #   print("app upgrade esp32 failed")
  #   failed_cnt += 1
  # reset_target(ser)
  # time.sleep(10)

  # in app upgrade sm2
  if ota(ser, sm2_bin_data):
    print("app upgrade sm2 seccussful!")
    success_cnt += 1
  else:
    print("app upgrade sm2 failed")
    failed_cnt += 1
  break
  reset_target(ser)
  time.sleep(10)
  
  if success_cnt + failed_cnt > 100:
    print("success %d, failed %d" % (success_cnt, failed_cnt))
    while True: 
      time.sleep(2)

  time.sleep(10)
  