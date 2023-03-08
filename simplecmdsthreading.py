from pymavlink import mavutil
import time
import math
import threading

buttons = 1 + 1 << 3 + 1 << 7

def manual_control(x=0, y=0, z=500, yaw=0):
    master.mav.manual_control_send(
        master.target_system,
        x,
        y,
        z,  # 500 means neutral throttle
        yaw,
        buttons
    )

def move_frd():
    for i in range(5):
        print(i)
        manual_control(x=300)
        time.sleep(2)

def move_rev():
    for i in range(5):
        print(i)
        manual_control(x=-300)
        time.sleep(2)

def turn_rgt():
    for i in range(5):
        print(i)
        manual_control(y=300)
        time.sleep(2)

def turn_lft():
    for i in range(5):
        print(i)
        manual_control(y=-300)
        time.sleep(2)

master = mavutil.mavlink_connection('/dev/ttyACM1', baud=115200)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat from APM (system %u component %u)" % (master.target_system, master.target_system))
master.arducopter_arm()
print("Arming motors")
master.motors_armed_wait()
print("Armed")

print("1.Forward \n2.Turn right \n3.Turn left")
movExists = int(input("Input goes here:"))

if(movExists == 1):
    thread = threading.Thread(target=move_frd)
    thread.start()
elif(movExists == 2):
    thread = threading.Thread(target=turn_rgt)
    thread.start()
elif(movExists == 3):
    thread = threading.Thread(target=turn_lft)
    thread.start()
else:
    print("Null_Options")

