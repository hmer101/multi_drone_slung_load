import rclpy

import numpy as np
import quaternionic as qt

import frame_transforms


def main():
    # Create node
    rclpy.init()

    q = qt.array([1.42, 0.0, 0.0, 1.42])
    q_norm = q.normalized

    a = np.array([0.71, 0.0, 0.0, 0.71])
    #y = frame_transforms.quaternion_get_yaw(np.asarray(q_norm))
    quat_d = frame_transforms.px4_to_ros_orientation(a)

    print(quat_d)

    #rclpy.spin(drone)

    # Destroy node
    rclpy.shutdown()

if __name__ == '__main__':
    main()