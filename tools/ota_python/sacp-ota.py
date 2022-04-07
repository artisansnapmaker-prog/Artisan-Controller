import serial
import time
import argparse
import fsm

# ================================================================================================
parser = argparse.ArgumentParser(description="snapIvesFOC ota")
parser.add_argument('--com', '-p', help='specify the serial port')
parser.add_argument('--baud', '-b', help='baudrate')
parser.add_argument('--file', '-f', help='file')
args = parser.parse_args()
COM = args.com
try: 
  BAUDRATE = int(args.baud)
except:
  print("error in the baudrate")

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
CMD_GET_VER = 0x11
CMD_OTA_START = 0x13
CMD_OTA_TRNAS = 0x15
CMD_OTA_FINISH = 0x17

def get_seq():
  global seq
  ret = seq
  seq += 1
  seq = seq & 0xff
  return ret

#####################################################
def get_frame(ser):
  start_tick_ms = int(round(time.time() * 1000))
  while True:
    c = ser.read()
    if len(c):
      if sf.push_char(c[0]):
        frame_str = " ".join(["{:02x}".format(x) for x in sf.frame])
        print("RX : " + frame_str)
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

def ota_start(ser, ra, size, checksum):
  pl = bytearray()
  pl.append(CMD_OTA_START)
  seq = get_seq()
  pl.append(seq)
  pl.extend(OTA_MAGIC.to_bytes(4, 'little'))
  pl.extend(ra.to_bytes(4, 'little'))
  pl.extend(size.to_bytes(4, 'little'))
  pl.extend(checksum.to_bytes(4, 'little'))
  frame = sf.build_frame(pl)
  frame_str = " ".join(["{:02x}".format(x) for x in frame])
  print("TX: " + frame_str)
  ser.write(frame)
  
  ret_frame = get_frame(ser)
  if None != ret_frame:
    cmd, ret_seq, ret_code = sf.pars_ret_frame(ret_frame)
    if cmd != CMD_OTA_START+1:
      print("ota start ack code error")
      return False
    if ret_seq != seq:
      print("ota start seq error")
      return False
    if ret_code != RET_OK:
      print("ota start return %d" % ret_code)
      return False
  return True

def ota_trans(ser, offset, block):
  pl = bytearray()
  pl.append(CMD_OTA_TRNAS)
  seq = get_seq()
  pl.append(seq)
  pl.extend(offset.to_bytes(4, 'little'))
  pl.extend(block)
  frame = sf.build_frame(pl)
  frame_str = " ".join(["{:02x}".format(x) for x in frame])
  print(frame_str)
  ser.write(frame)
  
  ret_frame = get_frame(ser)
  if None != ret_frame:
    cmd, ret_seq, ret_code = sf.pars_ret_frame(ret_frame)
    if cmd != CMD_OTA_TRNAS+1:
      print("ota trans ack code error")
      return None
    if ret_seq != seq:
      print("ota trnas seq error")
      return None
    if ret_code != RET_OK:
      print("ota trans ack return %d" % ret_code)
      return None
    ret_offset = int.from_bytes(ret_frame[7:10], 'little')
    if ret_offset != offset + len(block):
      print("return offset error")
      return None
    else:
      return ret_offset
  return None

def ota_finish(ser):
  pl = bytearray()
  pl.append(CMD_OTA_FINISH)
  seq = get_seq()
  pl.append(seq)
  frame = sf.build_frame(pl)
  frame_str = " ".join(["{:02x}".format(x) for x in frame])
  print(frame_str)
  ser.write(frame)
  
  ret_frame = get_frame(ser)
  if None != ret_frame:
    cmd, ret_seq, ret_code = sf.pars_ret_frame(ret_frame)
    if cmd != CMD_OTA_FINISH+1:
      print("ota finish ack code error")
      return False
    if ret_seq != seq:
      print("ota finish seq error")
      return False
    if ret_code != RET_OK:
      print("ota finish ack return %d" % ret_code)
      return False
  return True

def ota(ser, bin_data, run_addr, block_size):
  bin_size = len(bin_data)
  checksum = _4_byte_xor(bin_data)
  # ver_str = ota_get_ver(ser)
  # if None == ver_str:
  #   return False
  # print("get version " + ver_str )
  
  if True != ota_start(ser, run_addr, bin_size, int.from_bytes(checksum, 'little')):
    return False

  offset = 0
  while(offset < bin_size):
    if (bin_size - offset) > block_size:
      tx_block_size = block_size
    else:
      tx_block_size = bin_size - offset

    ret_offset = ota_trans(ser, offset, bin_data[offset:offset + tx_block_size])
    if None == ret_offset:
      return False

    offset = ret_offset
  return ota_finish(ser)
      

# bin_data = bytearray()
# for c in range(0, 0x100):
#   bin_data.append(c)

# open bin file
f = open(bin_file, 'rb')
bin_data = f.read()
print("len %d" % len(bin_data))
ser.read_all()

APP_RUN_ADDR = 0x08010800
while True:
  if ota(ser, bin_data, APP_RUN_ADDR, 128+64):
    print("ota seccussful!")
  else:
    print("ota failed")
    
  while(1):
    time.sleep(2)
  