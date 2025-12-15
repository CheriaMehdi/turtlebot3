#!/usr/bin/env python3
# map_cleaner.py
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import cv2
from skimage.morphology import skeletonize
from scipy.signal import savgol_filter

class MapCleaner(Node):
    def __init__(self):
        super().__init__('map_cleaner')
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 1)
        self.pub_clean = self.create_publisher(OccupancyGrid, '/cleaned_map', 1)
        self.pub_paths = self.create_publisher(Path, '/lane_centerline', 1)
        # parameters (tune these)
        self.declare_parameter('obstacle_thresh', 65)
        self.declare_parameter('free_thresh', 25)
        self.declare_parameter('morph_kernel_m', 0.18)   # morphological kernel in meters (approx lane width noise)
        self.declare_parameter('min_blob_area_m2', 0.02) # remove blobs smaller than this (m^2)
        self.get_logger().info('MapCleaner node started')

    def map_cb(self, msg: OccupancyGrid):
        try:
            w = msg.info.width
            h = msg.info.height
            res = msg.info.resolution
            origin = msg.info.origin
            arr = np.array(msg.data, dtype=np.int8).reshape((h, w))
            # thresholds
            occ_t = self.get_parameter('obstacle_thresh').value
            free_t = self.get_parameter('free_thresh').value
            # Build binary image: free -> 255, others -> 0
            img = np.zeros((h, w), dtype=np.uint8)
            img[arr <= free_t] = 255
            # Morphological filtering
            kernel_m = self.get_parameter('morph_kernel_m').value
            ksize = max(1, int(round(kernel_m / res)))
            if ksize % 2 == 0: ksize += 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
            img = cv2.medianBlur(img, 3)
            img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
            img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
            # Remove small connected components
            min_area = int(max(1, round(self.get_parameter('min_blob_area_m2').value / (res * res))))
            num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(img, connectivity=8)
            cleaned = np.zeros_like(img)
            for i in range(1, num_labels):
                area = stats[i, cv2.CC_STAT_AREA]
                if area >= min_area:
                    cleaned[labels == i] = 255
            # Skeletonize (skimage expects boolean)
            bool_img = cleaned > 0
            if bool_img.sum() == 0:
                self.get_logger().warn('No free area after cleaning - adjust parameters')
                return
            skel = skeletonize(bool_img).astype(np.uint8) * 255
            # Break skeleton into connected components to produce polylines
            num_lab, lab_img = cv2.connectedComponents(skel, connectivity=8)
            published_any = False
            for label in range(1, num_lab):
                pts = np.column_stack(np.where(lab_img == label))  # (row, col)
                if pts.shape[0] < 6:
                    continue
                # sort points along principal axis for ordering
                mean = pts.mean(axis=0)
                cov = np.cov(pts.T)
                eigvals, eigvecs = np.linalg.eig(cov)
                principal = eigvecs[:, np.argmax(eigvals)]
                proj = (pts - mean) @ principal
                idx = np.argsort(proj)
                pts_sorted = pts[idx]
                # Convert pixel -> world coords
                # Note: pixel (row, col). Map image origin typically top-left -> convert carefully:
                cols = pts_sorted[:, 1]
                rows = pts_sorted[:, 0]
                xs_world = origin.position.x + (cols * res)
                ys_world = origin.position.y + ((h - rows - 1) * res)
                # smoothing
                if len(xs_world) >= 7:
                    try:
                        xs_s = savgol_filter(xs_world, 7, 3)
                        ys_s = savgol_filter(ys_world, 7, 3)
                    except Exception:
                        xs_s = xs_world; ys_s = ys_world
                else:
                    xs_s = xs_world; ys_s = ys_world
                path = Path()
                path.header.frame_id = msg.header.frame_id
                path.header.stamp = self.get_clock().now().to_msg()
                for xx, yy in zip(xs_s, ys_s):
                    ps = PoseStamped()
                    ps.header = path.header
                    ps.pose.position.x = float(xx)
                    ps.pose.position.y = float(yy)
                    ps.pose.position.z = 0.0
                    ps.pose.orientation.w = 1.0
                    path.poses.append(ps)
                self.pub_paths.publish(path)
                published_any = True
            # Publish cleaned occupancy grid: free where cleaned>0, occupied elsewhere
            grid = OccupancyGrid()
            grid.header = msg.header
            grid.info = msg.info
            grid_data = np.full((h, w), 100, dtype=np.int8)  # occupied
            grid_data[cleaned > 0] = 0                       # free
            grid.data = [int(x) for x in grid_data.flatten()]

            self.pub_clean.publish(grid)
            if not published_any:
                self.get_logger().warn('No skeleton paths published; may be too aggressive cleaning.')
        except Exception as e:
            self.get_logger().error(f'Exception in map_cb: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MapCleaner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()