import pyrealsense2 as rs

#
# query realsense context and list the connected devices
#
ctx = rs.context()
devices = ctx.query_devices()
for device in devices:
    print(device)
quit()
