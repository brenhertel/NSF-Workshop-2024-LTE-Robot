#!/usr/bin/env python3

#all necessary imports
import rospy
import roslib
import message_filters
import tf
from sensor_msgs.msg import JointState
from tf.msg import tfMessage

import time
import h5py
import numpy as np
import os

import demo_lte_playback

# create subscribers for joint state, tf, wrench
#each subscriber writes data and timestamps to file (txt)
#if demo is chosen to be saved, read from the txt files
#compare timestamps
#if timestamps match, write to an array
#save array to h5 file

name = 'recorded_demo ' + time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) + '.h5'
HOST = "192.168.50.3" # The UR IP address
PORT = 30002 # UR secondary client

def tf_callback(tfmsg, tf_file):    
    if tfmsg.transforms[0].child_frame_id == 'tool0_controller':
        tf_file.write(str(tfmsg.transforms[0].header.stamp.secs) + ', ' + str(tfmsg.transforms[0].header.stamp.nsecs) + ', ' + str(tfmsg.transforms[0].transform.translation.x) + ', ' + str(tfmsg.transforms[0].transform.translation.y) + ', ' + str(tfmsg.transforms[0].transform.translation.z) + ', ' + str(tfmsg.transforms[0].transform.rotation.x) + ', ' + str(tfmsg.transforms[0].transform.rotation.y) + ', ' + str(tfmsg.transforms[0].transform.rotation.z) + ', ' + str(tfmsg.transforms[0].transform.rotation.w) + '\n')

def js_callback(jsmsg, js_file):
    if len(jsmsg.velocity) > 0:
        js_file.write(str(jsmsg.header.stamp.secs) + ', ' + str(jsmsg.header.stamp.nsecs).replace(')', '').replace('(', '') + ', ' + str(jsmsg.position).replace(')', '').replace('(', '') + ', ' + str(jsmsg.velocity).replace(')', '').replace('(', '') + ', ' + str(jsmsg.effort).replace(')', '').replace('(', '') + '\n')
 
def getline_data(fp):
    return np.array([float(i) for i in fp.readline().split(', ')])
   
def save_demo():
    js_fp = open('joint_data.txt', 'r')
    tf_fp = open('tf_data.txt', 'r')
    js_time_arr = np.zeros((1, 2))
    js_pos_arr = np.zeros((1, 6))
    js_vel_arr = np.zeros((1, 6))
    js_eff_arr = np.zeros((1, 6))
    tf_time_arr = np.zeros((1, 2))
    tf_pos_arr = np.zeros((1, 3))
    tf_rot_arr = np.zeros((1, 4))
    
    
    try:
        js_data = getline_data(js_fp)
        tf_data = getline_data(tf_fp)
        while True:
            js_time = js_data[0] + (js_data[1] * 10.0**-9)
            tf_time = tf_data[0] + (tf_data[1] * 10.0**-9)
            ctime_arr = [js_time, tf_time]
            if max(ctime_arr) - min(ctime_arr) < 0.001:
                print('found collective record at t=' + str(js_time))
                #record
                #print(len(js_data[0:2]))
                js_time_arr = np.vstack((js_time_arr, js_data[0:2]))
                #print(len(js_data[2:8]))
                js_pos_arr = np.vstack((js_pos_arr, js_data[2:8]))
                #print(len(js_data[8:14]))
                js_vel_arr = np.vstack((js_vel_arr, js_data[8:14]))
                #print(len(js_data[14:20]))
                js_eff_arr = np.vstack((js_eff_arr, js_data[14:20]))
                
                tf_time_arr = np.vstack((tf_time_arr, tf_data[0:2]))
                tf_pos_arr = np.vstack((tf_pos_arr, tf_data[2:5]))
                tf_rot_arr = np.vstack((tf_rot_arr, tf_data[5:9]))
                
                #print('reading js')
                js_data = getline_data(js_fp)
                #print('reading tf')
                tf_data = getline_data(tf_fp)
            else:
                if min(ctime_arr) == js_time:
                    #print('reading js')
                    js_data = getline_data(js_fp)
                elif min(ctime_arr) == tf_time:
                    #print('reading tf')
                    tf_data = getline_data(tf_fp)
                else:
                    rospy.loginfo('Should never get here')
    except ValueError:
        rospy.loginfo('Finished demo recording')
        
    js_fp.close()
    tf_fp.close()
    
    #delete first row of 0's
    js_time_arr = np.delete(js_time_arr, 0, 0)
    js_pos_arr = np.delete(js_pos_arr, 0, 0)
    js_vel_arr = np.delete(js_vel_arr, 0, 0)
    js_eff_arr = np.delete(js_eff_arr, 0, 0)
    
    tf_time_arr = np.delete(tf_time_arr, 0, 0)
    tf_pos_arr = np.delete(tf_pos_arr, 0, 0)
    tf_rot_arr = np.delete(tf_rot_arr, 0, 0)
    
    if len(js_time_arr) > 0:
        fp = h5py.File(name, 'w')
        
        dset_jt = fp.create_dataset('/joint_state_info/joint_time', data=js_time_arr)
        dset_jp = fp.create_dataset('/joint_state_info/joint_positions', data=js_pos_arr)
        dset_jv = fp.create_dataset('/joint_state_info/joint_velocities', data=js_vel_arr)
        dset_je = fp.create_dataset('/joint_state_info/joint_effort', data=js_eff_arr)
        
        dset_tt = fp.create_dataset('/transform_info/transform_time', data=tf_time_arr)
        dset_tp = fp.create_dataset('/transform_info/transform_positions', data=tf_pos_arr)
        dset_tr = fp.create_dataset('/transform_info/transform_orientations', data=tf_rot_arr)
        
        fp.close()
    else:
        rospy.logerr('NO DATA RECORDED! TRY AGAIN')
        exit()
    
def end_record(js_file, tf_file):
    print(time.time())
    
    js_file.close()
    tf_file.close()
    
    save = input('Would you like to save this demo? (y/n)')
    rospy.loginfo("You entered: %s", save)  
    if (save == 'y'):
        save_demo()
    else:
        exit()
        
    print('Please move the robot to the position of the new constraint. Press [Enter] to record this new constraint position.')
    input()
    tfmsg = rospy.wait_for_message("/tf", tfMessage)
    while not tfmsg.transforms[0].child_frame_id == 'tool0_controller':
        tfmsg = rospy.wait_for_message("/tf", tfMessage)
    pos_const = np.array([tfmsg.transforms[0].transform.translation.x, tfmsg.transforms[0].transform.translation.y, tfmsg.transforms[0].transform.translation.z])
    rot_const = np.array([tfmsg.transforms[0].transform.rotation.x, tfmsg.transforms[0].transform.rotation.y, tfmsg.transforms[0].transform.rotation.z, tfmsg.transforms[0].transform.rotation.w])
    #os.system("echo -e 'def test(): \n freedrive_mode() \n end_freedrive_mode() \nend \n' | telnet " + HOST + " " + str(PORT))
    demo_lte_playback.lte_demo_playback(name, pos_const, rot_const)
   
def demo_recorder():
    #create joint states file
    js_fp = open('joint_data.txt', 'w')
    #create tf data file
    tf_fp = open('tf_data.txt', 'w')
    
    rospy.init_node('demo_recorder', anonymous=True)
    
    print('Press [Enter] to start recording')
    input()
    print(time.time())
    
    #create subscribers to topics
    
    sub_js = rospy.Subscriber("/joint_states", JointState, js_callback, js_fp)
        
    sub_tf = rospy.Subscriber("/tf", tfMessage, tf_callback, tf_fp)
    
        
    rospy.loginfo('Recording has started')
    
    #start freedrive mode
    #os.system("echo -e 'def test(): \n freedrive_mode() \nend \n' | telnet " + HOST + " " + str(PORT))
        
    print('Press [Enter] to finish recording')
    input()
    print(time.time())
    sub_js.unregister()
    sub_tf.unregister()
    end_record(js_fp, tf_fp)
        
if __name__ == '__main__':
    demo_recorder()



