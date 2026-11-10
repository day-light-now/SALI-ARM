"""Live TF smoke test for the display-only shoulder installation."""
import time
import numpy as np
import rclpy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


def main():
    rclpy.init()
    node = rclpy.create_node('check_slaihand_shoulder_mount')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    try:
        deadline = time.monotonic() + 8
        while True:
            rclpy.spin_once(node, timeout_sec=.1)
            try:
                transform = buffer.lookup_transform('shoulder_world', 'base_link', Time())
                buffer.lookup_transform('shoulder_world', 'link4', Time())
                break
            except TransformException:
                if time.monotonic() > deadline:
                    raise RuntimeError('No shoulder installation TF received')
        q = transform.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        rotation = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
        ])
        np.testing.assert_allclose(rotation @ [0,1,0], [0,0,-1], atol=1e-10)
        np.testing.assert_allclose(rotation @ [0,0,1], [0,1,0], atol=1e-10)
        np.testing.assert_allclose(rotation @ [1,0,0], [1,0,0], atol=1e-10)
        gravity_base = rotation.T @ [0,0,-9.80665]
        np.testing.assert_allclose(gravity_base, [0,9.80665,0], atol=1e-10)
        p = transform.transform.translation
        np.testing.assert_allclose([p.x,p.y,p.z], [0,0,0], atol=1e-10)
        # Confirm the entire canonical chain is reachable in the new world.
        buffer.lookup_transform('shoulder_world', 'link4', Time())
        print('PASS: base +Y points down; world gravity equals base +Y; link4 TF reachable')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
