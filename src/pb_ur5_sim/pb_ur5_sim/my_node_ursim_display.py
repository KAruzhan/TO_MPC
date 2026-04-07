import rclpy
from rclpy.node import Node

import pybullet as p
import pybullet_data

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class RobotDisplayNode(Node):
    """PyBullet visualization node for URSim mode.

    Mirrors joint states published by ur_ros2_driver (from URSim) and
    displays the human body spheres from sim_utils. Does NOT simulate
    physics or publish joint states — ur_ros2_driver owns /joint_states.
    """

    def __init__(self):
        super().__init__('pb_ursim_display')

        # Joint states come from ur_ros2_driver, which reads them from URSim
        self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 1)
        self.create_subscription(Float64MultiArray, '/Sphere_poses_plotter', self.human_data_callback, 1)

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
                              0.2106,   0.4602,   0.6915]

        # PyBullet setup
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0.0, 0.0, 0.0)
        p.loadURDF("plane.urdf", [0, 0, -1.2])

        startPos = [0, 0, 0]
        startOrientation = p.getQuaternionFromEuler([0, 0, 0])
        self.boxId = p.loadURDF("ur5_urdf/full_UR5.urdf", startPos, startOrientation)

        # Set robot to home pose so it looks right before URSim connects
        start_pose = [0.0, -1.5708, 0.0, -1.5708, 0, 0]
        for i, angle in enumerate(start_pose):
            p.resetJointState(self.boxId, i + 1, angle)

        # Human body spheres (visual only, no collision)
        sphereRadius_arr = [0.511, 0.601, 0.451, 0.451, 0.421, 0.421,
                            0.421, 0.421, 0.401, 0.401, 0.411, 0.411, 0.461, 0.441]
        sphereRadius_arr = [0.5 * r for r in sphereRadius_arr]
        identity_orientation = [0, 0, 0, 1]
        self.sphere_ids = []
        for i in range(14):
            pos = [self.human_spheres[3*i], self.human_spheres[3*i+1], self.human_spheres[3*i+2]]
            vis = p.createVisualShape(p.GEOM_SPHERE, radius=sphereRadius_arr[i])
            uid = p.createMultiBody(1, -1, vis, pos, identity_orientation)
            self.sphere_ids.append(uid)

        p.syncBodyInfo()
        p.setRealTimeSimulation(True)
        self.get_logger().info('URSim display node running — mirroring /joint_states from ur_ros2_driver')

    def joint_states_callback(self, msg: JointState):
        """Teleport PyBullet robot to match URSim joint positions."""
        for i, pos in enumerate(msg.position[:6]):
            p.resetJointState(self.boxId, i + 1, pos)

    def human_data_callback(self, msg: Float64MultiArray):
        identity_orientation = [0, 0, 0, 1]
        for i, uid in enumerate(self.sphere_ids):
            pos = [msg.data[3*i], msg.data[3*i+1], msg.data[3*i+2]]
            p.resetBasePositionAndOrientation(uid, pos, identity_orientation)


def main(args=None):
    rclpy.init(args=args)
    node = RobotDisplayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        p.disconnect()


if __name__ == '__main__':
    main()
