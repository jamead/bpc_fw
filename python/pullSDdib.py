# Read SD card over ethernet and save data to file.
# Usage - python3 pullSDdib.py IP 6x0x_00xx
import socket
import time
import sys


try:
	IP = sys.argv[1]
	mod_ser = sys.argv[2] #to be appended to filename of output file
except:
	print("Usage - pullSDdib.py IP model_serial")
	sys.exit()

try:
	sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
	sock.settimeout(3)
	server_address = (IP, 5000)
except Exception as err:
	print("Socket error: %s" % err)


# get data from SD card
print()
print("Pulling data from SD card...")
print()
sock.sendto(b"SDrd", server_address)
data, addr = sock.recvfrom(2048)
data_str = data.decode('UTF-8')
print(data_str)

#save SD data to file
filename = "config_" + mod_ser + ".txt"
print("\n")
print("...Saving data to file %s" % filename)
print()
with open(filename, "w") as fp:
  fp.write(data_str)
fp.close()
