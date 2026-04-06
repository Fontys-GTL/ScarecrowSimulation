import time
from pymavlink import mavutil

def send_heartbeat():
    master = mavutil.mavlink_connection('udpout:127.0.0.1:14550')  # GCS port for PX4

    while True:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )
        print("Heartbeat sent")
        time.sleep(1)

if __name__ == "__main__":
    send_heartbeat()
