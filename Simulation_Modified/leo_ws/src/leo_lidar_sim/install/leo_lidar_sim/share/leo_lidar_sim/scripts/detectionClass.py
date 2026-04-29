try:
    import pyrealsense2 as rs
    HAS_REALSENSE = True
except ImportError:
    HAS_REALSENSE = False
    rs = None  

import numpy as np
import cv2
import csv
import ast


class DetectionSystem:
    def __init__(self, color_params_file, sim_mode=False):
        self.col_array = ['r', 'g', 'b', 'p', 'y']
        self.current_index = 0  # 默认红色
        self.min_block_size = 20
        self.sim_mode = sim_mode
        
        self.color_params = self._load_color_params(color_params_file)
        
        # 仿真模式的 HSV 参数（Gazebo 颜色和真实相机不同）
        self.sim_hsv_params = {
            'r': {'lower1': [0, 100, 50],   'upper1': [10, 255, 255],
                   'lower2': [170, 100, 50], 'upper2': [180, 255, 255]},
            'g': {'lower1': [35, 100, 50],   'upper1': [85, 255, 255],
                   'lower2': None,           'upper2': None},
            'b': {'lower1': [100, 100, 50],  'upper1': [130, 255, 255],
                   'lower2': None,           'upper2': None},
            'p': {'lower1': [130, 100, 50],  'upper1': [160, 255, 255],
                   'lower2': None,           'upper2': None},
            'y': {'lower1': [20, 100, 50],   'upper1': [35, 255, 255],
                   'lower2': None,           'upper2': None},
        }
        
        # 只在真实相机模式下初始化硬件
        if not sim_mode and HAS_REALSENSE:
            self.pipeline, self.align = self._initialize_camera()
        else:
            self.pipeline = None
            self.align = None
        
        # 仿真模式下手动设置的图像
        self.color_image = None
        self.depth_image = None
        
        self.goal_point_3d = None
        self._current_result = None
    
    def _load_color_params(self, color_params_file):
        fields = []
        rows = []
        
        with open(color_params_file, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            fields = next(csvreader)
            for row in csvreader:
                rows.append(row)
            print(f"Total No. of rows: {csvreader.line_num}")
        
        print('Field names are: ' + ', '.join(fields))
        
        LCRHSV, UCRHSV = ast.literal_eval(rows[0][1])
        LCGHSV, UCGHSV = ast.literal_eval(rows[0][2])
        LCBHSV, UCBHSV = ast.literal_eval(rows[0][3])
        LCYHSV, UCYHSV = ast.literal_eval(rows[0][4])
        LCPHSV, UCPHSV = ast.literal_eval(rows[0][5])
        
        return {
            'LCRHSV': LCRHSV, 'UCRHSV': UCRHSV,
            'LCGHSV': LCGHSV, 'UCGHSV': UCGHSV,
            'LCBHSV': LCBHSV, 'UCBHSV': UCBHSV,
            'LCYHSV': LCYHSV, 'UCYHSV': UCYHSV,
            'LCPHSV': LCPHSV, 'UCPHSV': UCPHSV
        }
    
    def _initialize_camera(self):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)
        
        align_to = rs.stream.color
        align = rs.align(align_to)
        
        print("Camera started successfully")
        return pipeline, align
    
    def _build_masks_sim(self, hsv_image):
        """仿真模式：使用仿真专用 HSV 参数"""
        masks = []
        for color_key in self.col_array:
            params = self.sim_hsv_params[color_key]
            lower1 = np.array(params['lower1'])
            upper1 = np.array(params['upper1'])
            mask = cv2.inRange(hsv_image, lower1, upper1)
            
            # 红色等需要两个范围的颜色
            if params['lower2'] is not None:
                lower2 = np.array(params['lower2'])
                upper2 = np.array(params['upper2'])
                mask2 = cv2.inRange(hsv_image, lower2, upper2)
                mask = cv2.bitwise_or(mask, mask2)
            
            masks.append(mask)
        return masks
    
    def _build_masks_real(self, hsv_image):
        """真实相机模式：使用 CSV 标定参数"""
        p = self.color_params
        
        # 红色（两个范围）
        mask_r1 = cv2.inRange(hsv_image,
            np.array([0, p['LCRHSV'][1], p['LCRHSV'][2]]),
            np.array([p['UCRHSV'][0], p['UCRHSV'][1], p['UCRHSV'][2]]))
        mask_r2 = cv2.inRange(hsv_image,
            np.array([p['LCRHSV'][0], p['LCRHSV'][1], p['LCRHSV'][2]]),
            np.array([180, p['UCRHSV'][1], p['UCRHSV'][2]]))
        mask_r = cv2.bitwise_or(mask_r1, mask_r2)
        
        # 绿色
        mask_g = cv2.inRange(hsv_image,
            np.array(p['LCGHSV']), np.array(p['UCGHSV']))
        
        # 蓝色
        mask_b = cv2.inRange(hsv_image,
            np.array(p['LCBHSV']), np.array(p['UCBHSV']))
        
        # 紫色
        mask_p = cv2.inRange(hsv_image,
            np.array(p['LCPHSV']), np.array(p['UCPHSV']))
        
        # 黄色
        mask_y = cv2.inRange(hsv_image,
            np.array(p['LCYHSV']), np.array(p['UCYHSV']))
        
        return [mask_r, mask_g, mask_b, mask_p, mask_y]
    
    def _apply_morphology(self, masks):
        """形态学处理：仿真少腐蚀，真实多腐蚀"""
        struc_elem = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        n = 1 if self.sim_mode else 5
        
        for i in range(len(masks)):
            for j in range(n):
                masks[i] = cv2.erode(masks[i], struc_elem)
        return masks
    
    def _update_frame(self):
        if self.sim_mode:
            return self._update_frame_sim()
        else:
            return self._update_frame_real()
    
    def _update_frame_sim(self):
        """仿真模式：使用外部设置的图像"""
        if self.color_image is None or self.depth_image is None:
            return False
        
        color_image = self.color_image
        depth_image = self.depth_image
        
        # 深度过滤（仿真深度单位是米，转毫米）
        depth_clean = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)
        depth_mm = (depth_clean * 1000).astype(np.uint16)
        depth_mask = (depth_mm > 0) & (depth_mm < 4000)
        filtered_color = cv2.bitwise_and(color_image, color_image, 
                                          mask=depth_mask.astype(np.uint8) * 255)
        
        # HSV 转换与 mask 生成
        hsv_image = cv2.cvtColor(filtered_color, cv2.COLOR_BGR2HSV)
        masks = self._build_masks_sim(hsv_image)
        masks = self._apply_morphology(masks)
        
        self._current_result = {
            'color_image': color_image,
            'filtered_color': filtered_color,
            'masks': masks,
            'depth_frame': None,
            'color_frame': None,
            'depth_image': depth_image
        }
        return True
    
    def _update_frame_real(self):
        """真实相机模式"""
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return False
        
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        depth_mask = (depth_image > 0) & (depth_image < 4000)
        filtered_color = cv2.bitwise_and(color_image, color_image,
                                          mask=depth_mask.astype(np.uint8) * 255)
        
        hsv_image = cv2.cvtColor(filtered_color, cv2.COLOR_BGR2HSV)
        masks = self._build_masks_real(hsv_image)
        masks = self._apply_morphology(masks)
        
        self._current_result = {
            'color_image': color_image,
            'filtered_color': filtered_color,
            'masks': masks,
            'depth_frame': depth_frame,
            'color_frame': color_frame,
            'depth_image': depth_image
        }
        return True
    
    def detect_block(self):
        if not self._update_frame():
            return None
        
        mask = self._current_result['masks'][self.current_index]
        color_image = self._current_result['color_image']
        filtered_color = self._current_result['filtered_color']
        depth_image = self._current_result['depth_image']
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blocks = []
        
        # 相机内参
        fx, fy = 277.0, 277.0
        cx, cy = 320.0, 240.0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 50:  # 过滤太小的噪点
                continue
            
            M = cv2.moments(cnt)
            if M["m00"] <= 0:
                continue
            
            px = int(M["m10"] / M["m00"])
            py = int(M["m01"] / M["m00"])
            cv2.circle(color_image, (px, py), 3, (0, 0, 255), -1)
            
            if self.sim_mode:
                depth = depth_image[py, px]  # 单位：米
                if depth <= 0 or np.isnan(depth):
                    continue
                x_3d = (px - cx) * depth / fx
                y_3d = (py - cy) * depth / fy
                z_3d = depth
                point_3d = [x_3d, y_3d, z_3d]
            else:
                depth_frame = self._current_result['depth_frame']
                color_frame = self._current_result['color_frame']
                depth = depth_frame.get_distance(px, py)
                if depth <= 0:
                    continue
                intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
                point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [px, py], depth)
            
            self.goal_point_3d = point_3d
            x, y, w, h = cv2.boundingRect(cnt)
            
            block_info = {
                'centroid': (px, py),
                'position_3d': point_3d,
                'depth': point_3d[2],
                'bounding_box': (x, y, w, h),
                'width': w,
                'height': h
            }
            blocks.append(block_info)
        
        cv2.imshow("Color", color_image)
        cv2.imshow("Mask", mask)
        cv2.imshow('Filtered Color Image', filtered_color)
        
        return blocks
    
    def detect_block_orientation(self):
        if not self._update_frame():
            return None
        
        mask = self._current_result['masks'][self.current_index]
        depth_frame = self._current_result['depth_frame']
        color_frame = self._current_result['color_frame']
        color_image = self._current_result['color_image']
        filtered_color = self._current_result['filtered_color']
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blocks = []
        
        for cnt in contours:
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                cv2.circle(mask, (cx, cy), 1, (0, 0, 255), -1)
                
                depth = depth_frame.get_distance(cx, cy)
                intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
                point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
                
                x, y, w, h = cv2.boundingRect(cnt)
                
                if w > self.min_block_size and h > self.min_block_size:
                    orientation = '90deg' if w / h >= 1 else '0deg'
                    
                    block_info = {
                        'centroid': (cx, cy),
                        'position_3d': point_3d,
                        'depth': point_3d[2],
                        'orientation': orientation
                    }
                    blocks.append(block_info)
        
        cv2.imshow("Color", color_image)
        cv2.imshow("Mask", mask)
        cv2.imshow('Filtered Color Image', filtered_color)
        
        return blocks

    def detect_box_edge(self):
        if not self._update_frame():
            return None
        
        color_image = self._current_result['color_image']
        depth_image = self._current_result['depth_image']
        
        max_depth = 350.0
        area_threshold = 0.015
        
        depth_mask = (depth_image > 0) & (depth_image < max_depth)
        filtered_color = cv2.bitwise_and(color_image, color_image,
                                          mask=depth_mask.astype(np.uint8) * 255)
        
        gray = cv2.cvtColor(filtered_color, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 150, 200)
        
        kernel_big = np.ones((5, 5), np.uint8)
        edges_dilated = cv2.dilate(edges, kernel_big, iterations=2)
        kernel_small = np.ones((3, 3), np.uint8)
        edges_clean = cv2.erode(edges_dilated, kernel_small, iterations=1)
        
        contours, _ = cv2.findContours(edges_clean, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        box_info = None
        visualization = filtered_color.copy()
        
        if len(contours) > 0:
            cnt = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(cnt)
            
            if area > (640 * 480) * area_threshold:
                rect = cv2.minAreaRect(cnt)
                (cx, cy), _, _ = rect
                cx, cy = int(cx), int(cy)
                
                box = cv2.boxPoints(rect).astype(np.int32)
                
                cv2.drawContours(visualization, [box], -1, (0, 255, 0), 2)
                cv2.circle(visualization, (cx, cy), 6, (0, 0, 255), -1)
                
                box_info = {
                    'center': (cx, cy),
                    'area': area,
                    'box_points': box
                }
        
        cv2.imshow("Edges", edges_clean)
        cv2.imshow("Box", visualization)
        
        return box_info
    
    def stop(self):
        if self.pipeline:
            self.pipeline.stop()
        cv2.destroyAllWindows()
        print("Camera stopped")

    def run(self, mode='block'):
        try:
            while True:
                if mode == 'block':
                    blocks = self.detect_block()
                    if blocks:
                        for block in blocks:
                            print(f"Block at {block['centroid']}, depth: {block['depth']:.3f}m")
                elif mode == 'orientation':
                    blocks = self.detect_block_orientation()
                    if blocks:
                        for block in blocks:
                            print(f"Block orientation: {block['orientation']}")
                elif mode == 'edge':
                    box = self.detect_box_edge()
                    if box:
                        print(f"Box center: {box['center']}, area: {box['area']:.2f}")
                
                key = cv2.waitKey(1)
                if key == 27 or key == ord('q'):
                    break
                elif key == ord('r'):
                    self.current_index = 0
                elif key == ord('g'):
                    self.current_index = 1
                elif key == ord('b'):
                    self.current_index = 2
                elif key == ord('p'):
                    self.current_index = 3
                elif key == ord('y'):
                    self.current_index = 4
                elif key == ord('o'):
                    mode = 'orientation'
                    cv2.destroyAllWindows()
                    print("Switched to orientation detection mode")
                elif key == ord('e'):
                    mode = 'edge'
                    cv2.destroyAllWindows()
                    print("Switched to edge detection mode")
                elif key == ord('d'):
                    mode = 'block'
                    cv2.destroyAllWindows()
                    print("Switched to block detection mode")
        
        except KeyboardInterrupt:
            pass
        
        finally:
            self.stop()


def main():
    color_params_file = '/home/student15/rspd_venv/src/colour_params.csv'
    detector = DetectionSystem(color_params_file)
    detector.run(mode='block')

if __name__ == "__main__":
    main()
