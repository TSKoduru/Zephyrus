"""Opt-in ROS-message pipeline smoke test, not a hardware or flight test."""
import os
from pathlib import Path
import tempfile
import unittest
import numpy as np


@unittest.skipUnless(os.environ.get('COURSE_ROS_INTEGRATION'), 'Requires built ROS package')
class RosAdapterTest(unittest.TestCase):
    def test_calibrated_stereo_to_filter_to_recording(self):
        self.run_pipeline(False)

    def test_compact_walk_pipeline_and_inline_display(self):
        self.run_pipeline(True)

    def run_pipeline(self, compact):
        import rclpy
        import cv2
        from rclpy.time import Time
        from sensor_msgs.msg import CameraInfo, Imu, Range
        from nav_msgs.msg import Odometry
        from geometry_msgs.msg import TransformStamped
        from scipy.spatial.transform import Rotation
        from course_localization import node as module
        with tempfile.TemporaryDirectory() as logs:
            old = module.DEFAULTS.copy()
            module.DEFAULTS.update(px4_range_topic='', record_raw_topics=False, log_root=logs,
                                   compact_logs=compact, ascii_live=compact)
            rclpy.init()
            node = module.CourseLocalization()
            try:
                stamp = node.get_clock().now().nanoseconds * 1e-9
                def ros_stamp(t):
                    return Time(nanoseconds=int(round(t * 1e9))).to_msg()
                optical = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
                for child, position, rotation in (
                    ('left_optical', [0., .0375, 0.], optical),
                    ('right_optical', [0., -.0375, 0.], optical),
                    ('imu_frame', [0., 0., 0.], np.eye(3)),
                    ('range_frame', [0., 0., 0.], Rotation.from_euler('y', np.pi / 2).as_matrix()),
                ):
                    tf = TransformStamped()
                    tf.header.frame_id, tf.child_frame_id = 'base_link', child
                    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = position
                    q = Rotation.from_matrix(rotation).as_quat()
                    tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w = map(float, q)
                    node.inputs.tf.set_transform_static(tf, 'test')
                for eye in ('left', 'right'):
                    info = CameraInfo(width=640, height=400, distortion_model='plumb_bob')
                    info.header.frame_id = eye + '_optical'
                    info.k = [360., 0., 319.5, 0., 360., 199.5, 0., 0., 1.]
                    info.d = [0.] * 5
                    node.inputs.info[eye] = info
                for t in (stamp - .02, stamp + .02):
                    odom = Odometry(child_frame_id='base_link')
                    odom.header.frame_id, odom.header.stamp = 'odom', ros_stamp(t)
                    odom.pose.pose.orientation.w = 1.
                    node.inputs.on_odom(odom)
                for t in np.arange(stamp - 2.05, stamp + .011, .005):
                    imu = Imu()
                    imu.header.frame_id, imu.header.stamp = 'imu_frame', ros_stamp(t)
                    imu.linear_acceleration.z = 9.81
                    node.inputs.on_imu(imu)
                reading = Range(range=.116, min_range=.04, max_range=4.)
                reading.header.frame_id, reading.header.stamp = 'range_frame', ros_stamp(stamp)
                node.inputs.on_range(reading)
                left = np.random.default_rng(7).integers(0, 256, (400, 640), dtype=np.uint8)
                # Spatially coherent texture; independent pixel noise is deliberately
                # rejected by the production noise-aware texture gate.
                left = cv2.GaussianBlur(left, (5, 5), 0.8)
                right = np.zeros_like(left)
                right[:, :-8] = left[:, 8:]
                images = []
                for eye, pixels in (('left', left), ('right', right)):
                    msg = node.bridge.cv2_to_imgmsg(pixels, encoding='mono8')
                    msg.header.frame_id, msg.header.stamp = eye + '_optical', ros_stamp(stamp)
                    images.append(msg)
                node.inputs.on_pair(*images)
                node.process()
                self.assertEqual(node.frame_count, 1, node.last_error)
                self.assertTrue(node.last_measurement['range_valid'])
                self.assertTrue(node.last_measurement['imu_valid'])
                node.close()
                files = list((node.recorder.path / 'frames').glob('*.npz'))
                if compact:
                    self.assertEqual(files, [])
                    self.assertFalse((node.recorder.path/'rosbag').exists())
                    self.assertTrue(list((node.recorder.path/'previews').glob('*depth_m.png')))
                    self.assertIn('#',node.live_view.text(stamp))
                    topics=node.get_publisher_names_and_types_by_node(node.get_name(),node.get_namespace())
                    self.assertFalse(any(name.startswith(('/skybeta/','/fmu/in/')) for name,_ in topics))
                else:
                    self.assertEqual(len(files), 1)
                    with np.load(files[0]) as result:
                        self.assertTrue(np.isfinite(result['depth_m']).any())
                        self.assertEqual(result['particles'].shape, (800, 4))
                        self.assertIn('T_map_body_vo', result.files)
                        self.assertIn('T_map_body_pf', result.files)
                self.assertTrue(list((node.recorder.path / 'snapshots').glob('*_vo.ply')))
            finally:
                node.close()
                node.destroy_node()
                rclpy.shutdown()
                module.DEFAULTS.clear()
                module.DEFAULTS.update(old)


if __name__ == '__main__':
    unittest.main()
