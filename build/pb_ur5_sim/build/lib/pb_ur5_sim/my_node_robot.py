import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import pybullet as p
import time
import pybullet_data
import os

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray
from math import copysign, pi

# goal_data = [0,0,0,0,0,0]
acceleration_limit = 4.6 # from URsim
single_step = acceleration_limit*1.0/240.0 # limit acceleration per step
current_mode = 'vels'


class RobotNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.subscription = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 1)
        # self.subscription = self.create_subscription(Float64MultiArray,"/my_UR5/velocity_command", self.vels_goal_callback,1)
        # self.subscription = self.create_subscription(Float64MultiArray, "/my_UR5/position_command", self.pose_goal_callback, 1)
        self.subscription_human = self.create_subscription(Float64MultiArray, '/Sphere_poses_plotter', self.human_data_callback, 1)
        timer_period = 1.0/60.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        ## Load gui and URDF
        physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
        p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
        p.setGravity(0.0,0.0,0.0) #-9.81
        planeId = p.loadURDF("plane.urdf", [0,0,-1.2])
        startPos = [0,0,0]
        startOrientation = p.getQuaternionFromEuler([0,0,0])
        self.boxId = p.loadURDF("ur5_urdf/full_UR5.urdf",startPos, startOrientation)
        
        for j in [1,2,3,4,5,6]:
            p.setJointMotorControl2(
                self.boxId,
                j,
                controlMode=p.VELOCITY_CONTROL,
                force=0
            )

        self.human_spheres = [0.0517,   0.5220,   1.0895,
			 0.0658,   0.4526,   0.8624,
			 0.0844,   0.7044,   0.9207,
			 0.2083,   0.3075,   1.0208,
			 0.0556,   0.6289,   0.7595,
			 0.2024,   0.2732,   0.8478,
			 0.0267,   0.5535,   0.5983,
			 0.1965,   0.2389,   0.6749,
			-0.0208,   0.3964,   0.5857,
			 0.0546,   0.2951,   0.6132,
			-0.1062,   0.2444,   0.5897,
			-0.0998,   0.3062,   0.5387,
			 0.1908,   0.5290,   1.0016,
			 0.2106,   0.4602,   0.6915];
        
        #sphereId_1 = p.loadURDF("sphere2.urdf", [0,0.5,1.5])
        sphereRadius_arr = [0.511, 0.601, 0.451, 0.451, 0.421, 0.421, 0.421, 0.421, 0.401, 0.401, 0.411, 0.411, 0.461, 0.441] 
        sphereRadius_arr = [0.5*x for x in sphereRadius_arr]
        colSphereId = -1
        visualShapeId_1 = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[0]); 
        visualShapeId_2 = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[1]);
        visualShapeId_3 = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[2]);
        visualShapeId_4 = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[3]);
        visualShapeId_5 = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[4]);
        visualShapeId_6 = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[5]);
        visualShapeId_7 = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[6]);
        visualShapeId_8 = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[7]);
        visualShapeId_9 = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[8]);
        visualShapeId_10 = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[9]);
        visualShapeId_11 = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[10]);
        visualShapeId_12 = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[11]);
        visualShapeId_13 = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[12]);
        visualShapeId_14 = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[13]);
        
        
        random_position = [0,0.5,1.5]
        identity_orientation = [0, 0, 0, 1]
        random_position_1 = random_position; random_position_1[0] = random_position_1[0]+1*0.05;
        random_position_2 = random_position; random_position_2[0] = random_position_2[0]+2*0.05;
        random_position_3 = random_position; random_position_3[0] = random_position_3[0]+1*0.05;
        random_position_4 = random_position; random_position_4[0] = random_position_4[0]+1*0.05;
        random_position_5 = random_position; random_position_5[0] = random_position_5[0]+1*0.05;
        random_position_6 = random_position; random_position_6[0] = random_position_6[0]+1*0.05;
        random_position_7 = random_position; random_position_7[0] = random_position_7[0]+1*0.05;
        random_position_8 = random_position; random_position_8[0] = random_position_8[0]+1*0.05;
        random_position_9 = random_position; random_position_9[0] = random_position_9[0]+1*0.05;
        random_position_10 = random_position; random_position_10[0] = random_position_10[0]+10*0.05;
        random_position_11 = random_position; random_position_11[0] = random_position_11[0]+11*0.05;
        random_position_12 = random_position; random_position_12[0] = random_position_12[0]+12*0.05;
        random_position_13 = random_position; random_position_13[0] = random_position_13[0]+13*0.05;
        random_position_14 = random_position; random_position_14[0] = random_position_14[0]+14*0.05;
        mass = 1; basePosition = [0,0.5,1.5]; baseOrientation = [0,0,0,1]
        self.sphereUid_1 = p.createMultiBody(mass, colSphereId, visualShapeId_1, random_position_1, baseOrientation)
        self.sphereUid_2 = p.createMultiBody(mass, colSphereId, visualShapeId_2, random_position_2, baseOrientation)
        self.sphereUid_3 = p.createMultiBody(mass, colSphereId, visualShapeId_3, random_position_3, baseOrientation)
        self.sphereUid_4 = p.createMultiBody(mass, colSphereId, visualShapeId_4, random_position_4, baseOrientation)
        self.sphereUid_5 = p.createMultiBody(mass, colSphereId, visualShapeId_5, random_position_5, baseOrientation)
        self.sphereUid_6 = p.createMultiBody(mass, colSphereId, visualShapeId_6, random_position_6, baseOrientation)
        self.sphereUid_7 = p.createMultiBody(mass, colSphereId, visualShapeId_7, random_position_7, baseOrientation)
        self.sphereUid_8 = p.createMultiBody(mass, colSphereId, visualShapeId_8, random_position_8, baseOrientation)
        self.sphereUid_9 = p.createMultiBody(mass, colSphereId, visualShapeId_9, random_position_9, baseOrientation)
        self.sphereUid_10 = p.createMultiBody(mass, colSphereId, visualShapeId_10, random_position_10, baseOrientation)
        self.sphereUid_11 = p.createMultiBody(mass, colSphereId, visualShapeId_11, random_position_11, baseOrientation)
        self.sphereUid_12 = p.createMultiBody(mass, colSphereId, visualShapeId_12, random_position_12, baseOrientation)
        self.sphereUid_13 = p.createMultiBody(mass, colSphereId, visualShapeId_13, random_position_13, baseOrientation)
        self.sphereUid_14 = p.createMultiBody(mass, colSphereId, visualShapeId_14, random_position_14, baseOrientation)
        
        #self.sphereUid_1 = p.loadURDF("sphere2.urdf", random_position_1, globalScaling=sphereRadius_arr[0]*2)
        #self.sphereUid_2 = p.loadURDF("sphere2.urdf", random_position_2, globalScaling=sphereRadius_arr[1]*2)
        #self.sphereUid_3 = p.loadURDF("sphere2.urdf", random_position_3, globalScaling=sphereRadius_arr[2]*2)
        #self.sphereUid_4 = p.loadURDF("sphere2.urdf", random_position_4, globalScaling=sphereRadius_arr[3]*2)
        #self.sphereUid_5 = p.loadURDF("sphere2.urdf", random_position_5, globalScaling=sphereRadius_arr[4]*2)
        #self.sphereUid_6 = p.loadURDF("sphere2.urdf", random_position_6, globalScaling=sphereRadius_arr[5]*2)
        #self.sphereUid_7 = p.loadURDF("sphere2.urdf", random_position_7, globalScaling=sphereRadius_arr[6]*2)
        #self.sphereUid_8 = p.loadURDF("sphere2.urdf", random_position_8, globalScaling=sphereRadius_arr[7]*2)
        #self.sphereUid_9 = p.loadURDF("sphere2.urdf", random_position_9, globalScaling=sphereRadius_arr[8]*2)
        #self.sphereUid_10 = p.loadURDF("sphere2.urdf", random_position_10, globalScaling=sphereRadius_arr[9]*2)
        #self.sphereUid_11 = p.loadURDF("sphere2.urdf", random_position_11, globalScaling=sphereRadius_arr[10]*2)
        #self.sphereUid_12 = p.loadURDF("sphere2.urdf", random_position_12, globalScaling=sphereRadius_arr[11]*2)
        #self.sphereUid_13 = p.loadURDF("sphere2.urdf", random_position_13, globalScaling=sphereRadius_arr[12]*2)
        #self.sphereUid_14 = p.loadURDF("sphere2.urdf", random_position_14, globalScaling=sphereRadius_arr[13]*2)
        
        #group = 0#other objects don't collide with me
        #mask=0 # don't collide with any other object
        #p.setCollisionFilterGroupMask(self.sphereUid_1, 0, group, mask)
        #p.setCollisionFilterGroupMask(self.sphereUid_2, 0, group, mask)
        #p.setCollisionFilterGroupMask(self.sphereUid_3, 0, group, mask)
        #p.setCollisionFilterGroupMask(self.sphereUid_4, 0, group, mask)
        #p.setCollisionFilterGroupMask(self.sphereUid_5, 0, group, mask)
        #p.setCollisionFilterGroupMask(self.sphereUid_6, 0, group, mask)
        #p.setCollisionFilterGroupMask(self.sphereUid_7, 0, group, mask)
        #p.setCollisionFilterGroupMask(self.sphereUid_8, 0, group, mask)
        #p.setCollisionFilterGroupMask(self.sphereUid_9, 0, group, mask)
        #p.setCollisionFilterGroupMask(self.sphereUid_10, 0, group, mask)
        #p.setCollisionFilterGroupMask(self.sphereUid_11, 0, group, mask)
        #p.setCollisionFilterGroupMask(self.sphereUid_12, 0, group, mask)
        #p.setCollisionFilterGroupMask(self.sphereUid_13, 0, group, mask)
        #p.setCollisionFilterGroupMask(self.sphereUid_14, 0, group, mask)
        
        self.sphere_pose_counter = 0
        
        p.syncBodyInfo()
        self.get_logger().info("%i" % p.getNumBodies())
        
        #maxForce = [25.0, 25.0, 25.0, 13.0, 13.0, 13.0] # from urdf
        self.maxForce = [40.0, 40.0, 40.0, 13.0, 13.0, 13.0] # from urdf
        self.maxSpeed = [3.15, 3.15, 3.15, 3.15, 3.15, 3.20] # from urdf
        #start_pose = [0.00104, -2.29971, -1.10022, -1.20020, -1.20016, 0.50019]
        start_pose = [0.0, -1.5708, 0.0, -1.5708, 0, 0]
        # p.resetJointStateMultiDof(self.boxId, 1, targetValue=[start_pose[0]], targetVelocity=[0.0])
        # p.resetJointStateMultiDof(self.boxId, 2, targetValue=[start_pose[1]], targetVelocity=[0.0])
        # p.resetJointStateMultiDof(self.boxId, 3, targetValue=[start_pose[2]], targetVelocity=[0.0])
        # p.resetJointStateMultiDof(self.boxId, 4, targetValue=[start_pose[3]], targetVelocity=[0.0])
        # p.resetJointStateMultiDof(self.boxId, 5, targetValue=[start_pose[4]], targetVelocity=[0.0])
        # p.resetJointStateMultiDof(self.boxId, 6, targetValue=[start_pose[5]], targetVelocity=[0.0])
        mode = p.VELOCITY_CONTROL
        # Start simulation
        p.setRealTimeSimulation(True);
        self.get_logger().info("running...")
        
    def joint_state_callback(self, msg):
        joint_map = {
            'shoulder_pan_joint': 1,
            'shoulder_lift_joint': 2,
            'elbow_joint': 3,
            'wrist_1_joint': 4,
            'wrist_2_joint': 5,
            'wrist_3_joint': 6,
        }

        for name, pos in zip(msg.name, msg.position):
            if name in joint_map:
                p.resetJointState(
                    self.boxId,
                    joint_map[name],
                    pos
                )



    # def vels_goal_callback(self, msg):
    #     global current_mode, goal_data
    #     current_mode = 'vels'
    #     goal_data = msg.data
    #     self.get_logger().info('vels recived')

    # def pose_goal_callback(self, msg):
    #     global current_mode, goal_data
    #     current_mode = 'pose'
    #     goal_data = msg.data[0:6]
    #     self.get_logger().info('pose recived')

    def human_data_callback(self, msg):
        self.human_spheres = msg.data[:]

    def timer_callback(self):       
        # simulate for 1/120
        try:
            # if GUI disabled use manual steps
            #p.stepSimulation()
            #time.sleep(1./240.)
            # as real time simulation selected just use ros sleep
            # states = p.getJointStates(self.boxId,[1,2,3,4,5,6]);
            # positions_short = [x[0] for x in states]
            # velocities_short = [x[1] for x in states]
            # if current_mode=='pose':
            #     p.setJointMotorControlArray(self.boxId, [1, 2, 3, 4, 5, 6],
            #         controlMode=p.POSITION_CONTROL, targetPositions = goal_data,
            #         forces = self.maxForce)
            # else:
            #     diff_array = [a - b for a, b in zip(goal_data, velocities_short)]
            #     #diff_array = goal_data - velocities_short
            #     #setpoint = velocities_short + [acceleration_limit*1.0/240.0*copysign(1, x) for x in diff_array]
            #     #setpoint = [a + single_step*copysign(1, b) for a, b in zip(velocities_short, diff_array)]
            #     setpoint = [0, 0, 0, 0, 0, 0]
            #     for i in range(6):
            #         if abs(diff_array[i])<single_step:
            #             setpoint[i] = velocities_short[i] + diff_array[i]
            #         else:
            #             setpoint[i] = velocities_short[i] + single_step*copysign(1, diff_array[i])
            #     p.setJointMotorControlArray(self.boxId, [1, 2, 3, 4, 5, 6],
            #         controlMode=p.VELOCITY_CONTROL, targetVelocities = setpoint,
            #         forces = self.maxForce)
            #
            self.sphere_pose_counter = self.sphere_pose_counter + 1
            if self.sphere_pose_counter%20:
            #if 0:
                random_position = [0,0.5,1.5*((self.sphere_pose_counter%10000)/10000.0)]
                identity_orientation = [0, 0, 0, 1]
                #random_position_1 = [random_position[0]+1*0.05,random_position[1],random_position[2]]
                #random_position_2 = [random_position[0]+2*0.05,random_position[1],random_position[2]]
                #random_position_3 = [random_position[0]+3*0.05,random_position[1],random_position[2]]
                #random_position_4 = [random_position[0]+4*0.05,random_position[1],random_position[2]]
                #random_position_5 = [random_position[0]+5*0.05,random_position[1],random_position[2]]
                #random_position_6 = [random_position[0]+6*0.05,random_position[1],random_position[2]]
                #random_position_7 = [random_position[0]+7*0.05,random_position[1],random_position[2]]
                #random_position_8 = [random_position[0]+8*0.05,random_position[1],random_position[2]]
                #random_position_9 = [random_position[0]+9*0.05,random_position[1],random_position[2]]
                #random_position_10 = [random_position[0]+10*0.05,random_position[1],random_position[2]]
                #random_position_11 = [random_position[0]+11*0.05,random_position[1],random_position[2]]
                #random_position_12 = [random_position[0]+12*0.05,random_position[1],random_position[2]]
                #random_position_13 = [random_position[0]+13*0.05,random_position[1],random_position[2]]
                #random_position_14 = [random_position[0]+14*0.05,random_position[1],random_position[2]]
                random_position_1 = [self.human_spheres[0+3*0],self.human_spheres[1+3*0],self.human_spheres[2+3*0]]
                random_position_2 = [self.human_spheres[0+3*1],self.human_spheres[1+3*1],self.human_spheres[2+3*1]]
                random_position_3 = [self.human_spheres[0+3*2],self.human_spheres[1+3*2],self.human_spheres[2+3*2]]
                random_position_4 = [self.human_spheres[0+3*3],self.human_spheres[1+3*3],self.human_spheres[2+3*3]]
                random_position_5 = [self.human_spheres[0+3*4],self.human_spheres[1+3*4],self.human_spheres[2+3*4]]
                random_position_6 = [self.human_spheres[0+3*5],self.human_spheres[1+3*5],self.human_spheres[2+3*5]]
                random_position_7 = [self.human_spheres[0+3*6],self.human_spheres[1+3*6],self.human_spheres[2+3*6]]
                random_position_8 = [self.human_spheres[0+3*7],self.human_spheres[1+3*7],self.human_spheres[2+3*7]]
                random_position_9 = [self.human_spheres[0+3*8],self.human_spheres[1+3*8],self.human_spheres[2+3*8]]
                random_position_10 = [self.human_spheres[0+3*9],self.human_spheres[1+3*9],self.human_spheres[2+3*9]]
                random_position_11 = [self.human_spheres[0+3*10],self.human_spheres[1+3*10],self.human_spheres[2+3*10]]
                random_position_12 = [self.human_spheres[0+3*11],self.human_spheres[1+3*11],self.human_spheres[2+3*11]]
                random_position_13 = [self.human_spheres[0+3*12],self.human_spheres[1+3*12],self.human_spheres[2+3*12]]
                random_position_14 = [self.human_spheres[0+3*13],self.human_spheres[1+3*13],self.human_spheres[2+3*13]]
                start = time.time()
                p.resetBasePositionAndOrientation(self.sphereUid_1, random_position_1, identity_orientation)
                p.resetBasePositionAndOrientation(self.sphereUid_2, random_position_2, identity_orientation)
                p.resetBasePositionAndOrientation(self.sphereUid_3, random_position_3, identity_orientation)
                p.resetBasePositionAndOrientation(self.sphereUid_4, random_position_4, identity_orientation)
                p.resetBasePositionAndOrientation(self.sphereUid_5, random_position_5, identity_orientation)
                p.resetBasePositionAndOrientation(self.sphereUid_6, random_position_6, identity_orientation)
                p.resetBasePositionAndOrientation(self.sphereUid_7, random_position_7, identity_orientation)
                p.resetBasePositionAndOrientation(self.sphereUid_8, random_position_8, identity_orientation)
                p.resetBasePositionAndOrientation(self.sphereUid_9, random_position_9, identity_orientation)
                p.resetBasePositionAndOrientation(self.sphereUid_10, random_position_10, identity_orientation)
                p.resetBasePositionAndOrientation(self.sphereUid_11, random_position_11, identity_orientation)
                p.resetBasePositionAndOrientation(self.sphereUid_12, random_position_12, identity_orientation)
                p.resetBasePositionAndOrientation(self.sphereUid_13, random_position_13, identity_orientation)
                p.resetBasePositionAndOrientation(self.sphereUid_14, random_position_14, identity_orientation)
                stop = time.time()
                #self.get_logger().info(str(stop-start))
            # hello_str = JointState()
            # hello_str.header = Header()
            # hello_str.header.stamp = self.get_clock().now().to_msg()
            # #hello_str.header.stamp = self.get_clock().now().to_msg()
            # hello_str.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            # hello_str.position = positions_short
            # hello_str.velocity = velocities_short
            # hello_str.effort = []
            # self.publisher_.publish(hello_str)
            #rate.sleep()
            #self.get_logger().info(" ".join([str(x) for x in positions_short]))
        except KeyboardInterrupt:
            print('Stopped')    
            print('simulation stopped')
            p.disconnect()
            #break
            raise SystemExit           # <--- here is we exit the node


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = RobotNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy     .shutdown()


if __name__ == '__main__':
    main()
