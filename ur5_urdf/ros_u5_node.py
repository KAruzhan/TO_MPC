#!/usr/bin/env python3
import pybullet as p
import time
import pybullet_data


import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray
from math import copysign

goal_data = [0,0,0,0,0,0]
acceleration_limit = 4.6 # from URsim
single_step = acceleration_limit*1.0/240.0 # limit acceleration per step
current_mode = 'vels'

def vels_goal_callback(msg):
    global current_mode, goal_data
    current_mode = 'vels'
    goal_data = msg.data
    print('vels recived')

def pose_goal_callback(msg):
    global current_mode, goal_data
    current_mode = 'pose'
    goal_data = msg.data[0:6]
    print('pose recived')

def main():
    global goal_data
    pub = rospy.Publisher('/joint_states', JointState, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(240) # 120hz x2 from gui speed
    rospy.Subscriber("/my_UR5/velocity_command", Float64MultiArray, vels_goal_callback)
    rospy.Subscriber("/my_UR5/position_command", Float64MultiArray, pose_goal_callback)
    
    ## Load gui and URDF
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0.0,0.0,0.0) #-9.81
    planeId = p.loadURDF("plane.urdf")
    startPos = [0,0,1]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    boxId = p.loadURDF("full_UR5.urdf",startPos, startOrientation)
    #maxForce = [25.0, 25.0, 25.0, 13.0, 13.0, 13.0] # from urdf
    maxForce = [40.0, 40.0, 40.0, 13.0, 13.0, 13.0] # from urdf
    maxSpeed = [3.15, 3.15, 3.15, 3.15, 3.15, 3.20] # from urdf
    #start_pose = [0.00104, -2.29971, -1.10022, -1.20020, -1.20016, 0.50019]
    start_pose = [0.0, -1.5708, 0.0, -1.5708, 0, 0]
    p.resetJointStateMultiDof(boxId, 1, targetValue=[start_pose[0]], targetVelocity=[0.0])
    p.resetJointStateMultiDof(boxId, 2, targetValue=[start_pose[1]], targetVelocity=[0.0])
    p.resetJointStateMultiDof(boxId, 3, targetValue=[start_pose[2]], targetVelocity=[0.0])
    p.resetJointStateMultiDof(boxId, 4, targetValue=[start_pose[3]], targetVelocity=[0.0])
    p.resetJointStateMultiDof(boxId, 5, targetValue=[start_pose[4]], targetVelocity=[0.0])
    p.resetJointStateMultiDof(boxId, 6, targetValue=[start_pose[5]], targetVelocity=[0.0])
    mode = p.VELOCITY_CONTROL
    # Start simulation
    p.setRealTimeSimulation(True);
    hello_str = JointState()
    print('runnung...')
    while not rospy.is_shutdown():
        # simulate for 1/120
        try:
            # if GUI disabled use manual steps
            #p.stepSimulation()
            #time.sleep(1./240.)
            # as real time simulation selected just use ros sleep
            states = p.getJointStates(boxId,[1,2,3,4,5,6]);
            positions_short = [x[0] for x in states]
            velocities_short = [x[1] for x in states]
            if current_mode=='pose':
                p.setJointMotorControlArray(boxId, [1, 2, 3, 4, 5, 6],
                    controlMode=p.POSITION_CONTROL, targetPositions = goal_data,
                    forces = maxForce)
            else:
                diff_array = [a - b for a, b in zip(goal_data, velocities_short)]
                #diff_array = goal_data - velocities_short
                #setpoint = velocities_short + [acceleration_limit*1.0/240.0*copysign(1, x) for x in diff_array]
                #setpoint = [a + single_step*copysign(1, b) for a, b in zip(velocities_short, diff_array)]
                setpoint = [0, 0, 0, 0, 0, 0]
                for i in range(6):
                    if abs(diff_array[i])<single_step:
                        setpoint[i] = velocities_short[i] + diff_array[i]
                    else:
                        setpoint[i] = velocities_short[i] + single_step*copysign(1, diff_array[i])
                p.setJointMotorControlArray(boxId, [1, 2, 3, 4, 5, 6],
                    controlMode=p.VELOCITY_CONTROL, targetVelocities = setpoint,
                    forces = maxForce)
            #
            hello_str.header = Header()
            hello_str.header.stamp = rospy.Time.now()
            hello_str.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            hello_str.position = positions_short
            hello_str.velocity = velocities_short
            hello_str.effort = []
            pub.publish(hello_str)
            rate.sleep()
        except KeyboardInterrupt:
            print('Stopped')
            break
    print('simulation stopped')
    p.disconnect()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
