#!/usr/bin/env python
import rospy
import argparse

from time import sleep
from numpy import deg2rad
from std_msgs.msg import String
from nmpc_astrobee.msg import NMPCInstruct
from tf.transformations import quaternion_from_euler

# For rosbag and CTL-C
import subprocess, shlex
from signal import signal, SIGINT, pause
from sys import exit

ROSBAG_NAME = "test16"
astro_bag = None 

def test_coordinator(test_num, pub):
    print('Beginning test...')
    if test_num == "kill":
        test_kill(pub)
    elif test_num == "zero_hold":
        test_zero_hold(pub)
    elif test_num == "normal_nmpc":
        test_normal_nmpc(pub)
    elif test_num == "cool_spin":
        test_cool_spin(pub)
    elif test_num == "all_sys_id":
        test_all_sys_id(pub)
    elif test_num == "obs_goal":
        test_obs_goal(pub)
    elif test_num == "nmpc_weight_throughout":
        test_nmpc_weight_throughout(pub)
    elif test_num == "nmpc_weight_first":
        test_nmpc_weight_first(pub)
    elif test_num == "nmpc_no_weight":
        test_nmpc_no_weight(pub)
        
    # wait until the process is killed
    pause()

def callback(data):
    rospy.loginfo("Data received from publishing node: %s",data.data)
    
# Send a message with updated position and weighting.
def send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, quat1, quat2, quat3, quat4, ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz):
    msg = NMPCInstruct()
    msg.x = x
    msg.y = y
    msg.z = z
    msg.vel_x = vel_x
    msg.vel_y = vel_y
    msg.vel_z = vel_z
    msg.quat1 = quat1
    msg.quat2 = quat2
    msg.quat3 = quat3
    msg.quat4 = quat4
    msg.ang_vel_x = ang_vel_x
    msg.ang_vel_y = ang_vel_y
    msg.ang_vel_z = ang_vel_z
    msg.status = status
    msg.info_weight_m = info_weight_m
    msg.info_weight_Izz = info_weight_Izz
    # t1 = rospy.get_time()
    # while (pub.get_num_connections() == 0 and ((rospy.get_time()-t1) < 2.0)):  # wait until someone actually is subscribed. This can take a few hundred ms
    #     poll_rate.sleep()
    sleep(0.7)  # there is a delay in setting up subscribers/publishers, which gets longer if more computation is going on
    pub.publish(msg)

# rosbag handling
def rosbag_start():
    global astro_bag

    # Top one is for hardware, bottom is for local machine
    command = "rosbag record -O /data/info-weight/02-12-20/"+ROSBAG_NAME+".bag gnc/ctl/command /gnc/ctl/nmpc_instruct /gnc/ekf /hw/pmc/command /mob/estimator"
    # command = "rosbag record -O /home/albee/workspaces/astrobee-ws-shared/freeflyer-shared/data/tester/"+ROSBAG_NAME+".bag gnc/ctl/command /gnc/ctl/nmpc_instruct /gnc/ekf /hw/pmc/command /mob/estimator"
    command = shlex.split(command)
    astro_bag = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, stdin=subprocess.PIPE)  # we do this so SIGINT is caught by the parent
    print('Starting rosbag...')

def rosbag_stop():
    global astro_bag
    astro_bag.send_signal(subprocess.signal.SIGINT)

def handler(signal_received, frame):
    print('\nSIGINT or CTRL-C detected, killing rosbags...')
    try:
        rosbag_stop()
        print('...complete.')
    except:
        print('...no rosbag started.')

"""
Custom tests below!
"""

# Turn off the dumb fans!
def test_kill(pub):
    x = 0.0; y = 0.0; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    quat1 = 0.0; quat2 = 0.0; quat3 = 0.0; quat4 = 1.0
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 2
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, quat1, quat2, quat3, quat4, ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)

    print('Shutting off impellers...')

# Hold at zero
def test_zero_hold(pub):
    x = 0.0; y = 0.0; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    quat1 = 0.0; quat2 = 0.0; quat3 = 0.0; quat4 = 1.0
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    rosbag_start()

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, quat1, quat2, quat3, quat4, ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)

# Hold at zero
def test_obs_goal(pub):
    x = 0.35; y = 0.35; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    quat1 = 0.0; quat2 = 0.0; quat3 = 0.0; quat4 = 1.0
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, quat1, quat2, quat3, quat4, ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)

# Move from rear of table to front after hold.
def test_normal_nmpc(pub):
    # hold at [0.0 0.0 0.0], wait for EKF
    x = -0.3; y = -0.3; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)

    print('Moving to start pose...')
    sleep(25)  
    rosbag_start()
    sleep(5)

    # hold at [0.0 0.0 0.0], wait for EKF
    x = 0.35; y = 0.35; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)
    print('Moving to final pose...')

# Move from rear of table to front after hold.
def test_nmpc_no_weight(pub):
    # hold at [0.0 0.0 0.0], wait for EKF
    x = -0.3; y = -0.3; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)

    print('Moving to start pose...')
    rosbag_start()
    sleep(40)


    x = 0.35; y = 0.35; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(0))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)
    print('Moving to next pose...')
    sleep(25)

    x = -0.3; y = -0.3; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)
    print('Moving to final pose...')

# Move from rear of table to front after hold.
def test_nmpc_weight_first(pub):
    # hold at [0.0 0.0 0.0], wait for EKF
    x = 0.0; y = 0.0; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.1

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)

    print('Moving to start pose...')
    rosbag_start()
    sleep(25)

    # hold at [0.0 0.0 0.0], wait for EKF
    x = -0.3; y = -0.3; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)

    print('Moving to next pose...')
    sleep(25)


    x = 0.35; y = 0.35; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(0))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)
    print('Moving to next pose...')
    sleep(25)

    x = -0.3; y = -0.3; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)
    print('Moving to final pose...')

# Move from rear of table to front after hold.
def test_nmpc_weight_throughout(pub):
    # hold at [0.0 0.0 0.0], wait for EKF
    x = -0.3; y = -0.3; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 1
    info_weight_m = 100
    info_weight_Izz = 0.05

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)

    print('Moving to start pose...')
    rosbag_start()
    sleep(40)


    x = 0.35; y = 0.35; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(0))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 1
    info_weight_m = 50
    info_weight_Izz = 0.02

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)
    print('Moving to next pose...')
    sleep(25)

    x = -0.3; y = -0.3; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)
    print('Moving to final pose...')

# For show.
def test_cool_spin(pub):
    # hold at [0.0 0.0 0.0], wait for EKF
    x = -0.3; y = -0.3; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.2

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)
    print('Moving to start pose...')
    rosbag_start()
    sleep(30)  

    x = 0.3; y = 0.3; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.2

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)
    print('Moving to next pose...')
    sleep(30)  

    x = -0.3; y = 0.3; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.2

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)
    print('Moving to next pose...')
    sleep(30)

    x = 0.3; y = -0.3; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.2

    status = 1
    info_weight_m = 0.0
    info_weight_Izz = 0.0

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)
    print('Moving to final pose...')

# Only info weight, turn off other cost
def test_all_sys_id(pub):
    # hold at [0.0 0.0 0.0], wait for EKF
    x = 0.0; y = 0.0; z = 0.0
    vel_x = 0.0; vel_y = 0.0; vel_z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    ang_vel_x = 0.0; ang_vel_y = 0.0; ang_vel_z = 0.0

    status = 3
    info_weight_m = 0.0
    info_weight_Izz = 0.1

    send_update_msg(pub, x, y, z, vel_x, vel_y, vel_z, q[0], q[1], q[2], q[3], ang_vel_x, ang_vel_y, ang_vel_z, status, info_weight_m, info_weight_Izz)

    print('Moving to start pose...')
    rosbag_start()

# Go to zero and hold
def test2(pub):
    # hold at [0.0 0.0 0.0], wait for EKF
    x = 0.35; y = 0.35; z = 0.0
    q = quaternion_from_euler(0, 0, deg2rad(-90))
    status = 1
    send_update_msg(pub, x, y, z, q[0], q[1], q[2], q[3], status)
    print('Holding zero!')


def test3(pub):
    # hold at [0.0 0.0 0.0], wait for EKF
    x = 0.4; y = 0.0; z = 0.0
    quat1 = 0.0; quat2 = 0.0; quat3 = 0.0; quat4 = 1.0
    # RPY to convert: 0,0,-50
    # q = quaternion_from_euler(0, 0, deg2rad(-50))
    status = 1
    send_update_msg(pub, x, y, z, q[0], q[1], q[2], q[3], status)
    print('Rotating!')


# Main
if __name__ == '__main__':
    signal(SIGINT, handler)

    try:
        parser = argparse.ArgumentParser(description='Run a desired test session.')
        parser.add_argument('test_num', metavar='TEST', type=String, nargs=1,
                            help='A test number to run.')
        args = parser.parse_args()
        test_num = args.test_num[0]  # the test number to run

        # Set up a publisher to write to our node
        pub = rospy.Publisher('gnc/ctl/nmpc_instruct', NMPCInstruct, queue_size=10, latch=True)
        rospy.init_node('run_test', anonymous=True)

        test_coordinator(test_num.data, pub)
    except rospy.ROSInterruptException:
        pass