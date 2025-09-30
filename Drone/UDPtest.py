from pymavlink import mavutil
m = mavutil.mavlink_connection('udp:127.0.0.1:14551')
while True:
    msg = m.recv_match(blocking=True)
    print(msg)
