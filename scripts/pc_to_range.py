#!/usr/bin/env python3
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2


class Points2Img:
    """Transform point clouds (with dim [N, 3]) to range images.
    Returns dmax-normalized [H, W] array of float
    shape_imgs  -- [H, W] image shape
    dmax        -- max
    hfov        -- horizontal fov
    vfov        -- vertical fov
    """
    def __init__(self, shape_imgs, dmax, hfov, vfov):
        self.shape_imgs = shape_imgs
        self.height = shape_imgs[-2]
        self.width = shape_imgs[-1]
        self.dmax = dmax
        self.hfov = hfov
        self.vfov = vfov

    def __call__(self, pc: np.ndarray) -> np.ndarray:
        img = np.ones(self.shape_imgs, dtype=float)

        r = np.linalg.norm(pc, axis=-1)
        mask = (r > 0) & (r < self.dmax)

        if not mask.any():
            return img

        pc = pc[mask]
        r = r[mask]
    
        azimuth = np.arctan2(pc[:,1], pc[:,0])
        elevation = np.arcsin(pc[:,2] / r)

        ## mask out by range and bearing
        mask = (azimuth >= -self.hfov / 2) & \
            (azimuth <= self.hfov / 2) & \
            (elevation >= -self.vfov / 2) & \
            (elevation <= self.vfov / 2)
        
        if not mask.any():
            return img
    
        pc = pc[mask]
        r = r[mask] / self.dmax
        azimuth = azimuth[mask]
        elevation = elevation[mask]
        
        ## binning
        u = np.rint(0.5 * (-azimuth + self.hfov) / self.hfov * (self.width - 1)).astype(int)
        v = np.rint(0.5 * (-elevation + self.vfov) / self.vfov * (self.height - 1)).astype(int)
        # img[v, u] = np.min(img[u,v], r)
        img[v, u] = r

        return img



class PcToRangeNode(Node):
    def __init__(self):
        super().__init__('pc_to_range')

        ## ros params
        self.declare_parameter('img_width', 512)
        self.declare_parameter('img_height', 128)
        self.declare_parameter('max_range', 10.0)
        self.declare_parameter('hfov', 3.1415)
        self.declare_parameter('vfov', 0.7853)

        img_width = int(self.get_parameter('img_width').value)
        img_height = int(self.get_parameter('img_height').value)
        max_range = float(self.get_parameter('max_range').value)
        hfov = float(self.get_parameter('hfov').value)
        vfov = float(self.get_parameter('vfov').value)

        self.pc2img = Points2Img([img_height, img_width], max_range, hfov, vfov)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub_img = self.create_publisher(Image, '~/img_out', qos)
        self.sub_pc  = self.create_subscription(PointCloud2, '~/pc_in', self.cb_pc, qos)

    def cb_pc(self, msg: PointCloud2):
        ## read pc
        pts_iter = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        pc = np.fromiter(pts_iter, dtype=np.dtype([('x', 'f4'), ('y', 'f4'), ('z', 'f4')])).view(np.float32).reshape(-1, 3)

        if len(pc):
            img = self.pc2img(pc)

            from matplotlib import pyplot as plt
            plt.imshow(img)
            plt.show()

            msg_img = Image()
            msg_img.header = msg.header
            msg_img.height = img.shape[0]
            msg_img.width = img.shape[1]
            msg_img.encoding = '32FC1'
            msg_img.is_bigendian = 0
            msg_img.step = img.shape[1] * 4
            msg_img.data = img.tobytes(order='F')
            self.pub_img.publish(msg_img)


if __name__ == '__main__':
    rclpy.init()
    node = PcToRangeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
