import pyrealsense2 as rs
import numpy as np
import cv2
# import time
import csv
import ast


class DetectionSystem:
    def __init__(self, color_params_file):
        self.col_array = ['r', 'g', 'b', 'p', 'y']
        self.current_index = 4
        self.min_block_size = 20
        
        self.color_params = self._load_color_params(color_params_file)
        self.pipeline, self.align = self._initialize_camera()
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
        print('\nFirst row is\n')
        print(f'{rows[0][1][1]}')
        
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
    
    def _update_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return False
        
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        min_depth = 0
        max_depth = 4000.0
        depth_mask = (depth_image > min_depth) & (depth_image < max_depth)
        filtered_color = cv2.bitwise_and(color_image, color_image, mask=depth_mask.astype(np.uint8)*255)
        
        hsv_image = cv2.cvtColor(filtered_color, cv2.COLOR_BGR2HSV)
        lower_color_p = np.array([self.color_params['LCPHSV'][0], self.color_params['LCPHSV'][1], self.color_params['LCPHSV'][2]])
        upper_color_p = np.array([self.color_params['UCPHSV'][0], self.color_params['UCPHSV'][1], self.color_params['UCPHSV'][2]])
        mask_p = cv2.inRange(hsv_image, lower_color_p, upper_color_p)

        lower_color_b = np.array([self.color_params['LCBHSV'][0],self.color_params['LCBHSV'][1],self.color_params['LCBHSV'][2]])
        upper_color_b = np.array([self.color_params['UCBHSV'][0],self.color_params['UCBHSV'][1],self.color_params['UCBHSV'][2]])
        mask_b = cv2.inRange(hsv_image, lower_color_b, upper_color_b)

        lower_r1 = np.array([0,self.color_params['LCRHSV'][1],self.color_params['LCRHSV'][2]])
        upper_r1 = np.array([self.color_params['UCRHSV'][0],self.color_params['UCRHSV'][1],self.color_params['UCRHSV'][2]])
        
        lower_r2 = np.array([self.color_params['LCRHSV'][0],self.color_params['LCRHSV'][1],self.color_params['LCRHSV'][2]])
        upper_r2 = np.array([180,self.color_params['UCRHSV'][1],self.color_params['UCRHSV'][2]])

        mask_r1 = cv2.inRange(hsv_image, lower_r1, upper_r1)
        mask_r2 = cv2.inRange(hsv_image, lower_r2, upper_r2)
        mask_r = cv2.bitwise_or(mask_r1,mask_r2)

        lower_g = np.array([self.color_params['LCGHSV'][0],self.color_params['LCGHSV'][1],self.color_params['LCGHSV'][2]])
        upper_g = np.array([self.color_params['UCGHSV'][0],self.color_params['UCGHSV'][1],self.color_params['UCGHSV'][2]])
        mask_g = cv2.inRange(hsv_image, lower_g, upper_g)

        lower_y = np.array([self.color_params['LCYHSV'][0],self.color_params['LCYHSV'][1],self.color_params['LCYHSV'][2]])
        upper_y = np.array([self.color_params['UCYHSV'][0],self.color_params['UCYHSV'][1],self.color_params['UCYHSV'][2]])
        mask_y = cv2.inRange(hsv_image, lower_y, upper_y)
        masks = [mask_r, mask_g, mask_b, mask_p, mask_y]
        
        struc_elem = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
        n = 5
        for mask in masks:
            for i in range(n):
                cv2.erode(mask, struc_elem)
        
        self._current_result = {
            'color_image': color_image,
            'filtered_color': filtered_color,
            'masks': masks,
            'depth_frame': depth_frame,
            'color_frame': color_frame,
            'depth_image': depth_image
        }
        
        return True
    
    def detect_all_blocks(self):
        if not self._update_frame():
            return None
        
        depth_frame = self._current_result['depth_frame']
        color_frame = self._current_result['color_frame']
        color_image = self._current_result['color_image']
        filtered_color = self._current_result['filtered_color']
        
        all_blocks = []
        combined_mask = np.zeros_like(self._current_result['masks'][0])  # 合并用的mask
        
        for i, color_name in enumerate(self.col_array):
            mask = self._current_result['masks'][i]
            combined_mask = cv2.bitwise_or(combined_mask, mask)  # 合并
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    
                    cv2.circle(mask, (cx, cy), 1, (0, 0, 255), -1)
                    
                    depth = depth_frame.get_distance(cx, cy)
                    
                    intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
                    point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], depth)
                    self.goal_point_3d = point_3d
                    x, y, w, h = cv2.boundingRect(cnt)
                    
                    block_info = {
                        'color': color_name,
                        'centroid': (cx, cy),
                        'position_3d': point_3d,
                        'depth': point_3d[2],
                        'bounding_box': (x, y, w, h),
                        'width': w,
                        'height': h
                    }
                    all_blocks.append(block_info)
        
        cv2.imshow("Color", color_image)
        cv2.imshow("Mask", combined_mask)  # 显示合并后的mask
        cv2.imshow('Filtered Color Image', filtered_color)
        
        return all_blocks

    def detect_block(self):
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
                self.goal_point_3d = point_3d
                x, y, w, h = cv2.boundingRect(cnt)
                
                block_info = {
                    'centroid': (cx, cy),
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
                    if w / h >= 1:
                        orientation = '90deg'
                    else:
                        orientation = '0deg'
                    
                    block_info = {
                        'centroid': (cx, cy),
                        'position_3d': point_3d,
                        'depth': point_3d[2],
                        # 'bounding_box': (x, y, w, h),
                        # 'width': w,
                        # 'height': h,
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
        
        min_depth = 0
        max_depth = 350.0
        area_threshold = 0.015
        
        depth_mask = (depth_image > min_depth) & (depth_image < max_depth)
        filtered_color = cv2.bitwise_and(color_image, color_image, mask=depth_mask.astype(np.uint8)*255)
        
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
                (cx, cy), _, _= rect
                cx, cy = int(cx), int(cy)
                
                box = cv2.boxPoints(rect)
                box = box.astype(np.int32)
                
                cv2.drawContours(visualization, [box], -1, (0, 255, 0), 2)
                cv2.circle(visualization, (cx, cy), 6, (0, 0, 255), -1)
                
                box_info = {
                    'center': (cx, cy),
                    # 'width': rw,
                    # 'height': rh,
                    # 'angle': angle,
                    'area': area,
                    'box_points': box
                }
        
        cv2.imshow("Edges", edges_clean)
        cv2.imshow("Box", visualization)
        # cv2.imshow('Blur', blur)
        
        return box_info
    
    def stop(self):
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

# def main():
#     color_params_file = '/home/student15/rspd_venv/src/colour_params.csv'
#     detector = DetectionSystem(color_params_file)
    
#     mode = 'block'  
    
#     try:
#         while True:
#             if mode == 'block':
#                 blocks = detector.detect_block()
#                 if blocks:
#                     for block in blocks:
#                         print(f"Block at {block['centroid']}, depth: {block['depth']:.3f}m")
#             elif mode == 'orientation':
#                 blocks = detector.detect_block_orientation()
#                 if blocks:
#                     for block in blocks:
#                         print(f"Block orientation: {block['orientation']}")
#             elif mode == 'edge':
#                 box = detector.detect_box_edge()
#                 if box:
#                     print(f"Box center: {box['center']}, area: {box['area']:.2f}")
        
#             key = cv2.waitKey(1)
#             if key == 27 or key == ord('q'):
#                 break
#             elif key == ord('r'):
#                 detector.current_index = 0
#             elif key == ord('g'):
#                 detector.current_index = 1
#             elif key == ord('b'):
#                 detector.current_index = 2
#             elif key == ord('p'):
#                 detector.current_index = 3
#             elif key == ord('y'):
#                 detector.current_index = 4
#             elif key == ord('o'):
#                 mode = 'orientation'
#                 cv2.destroyAllWindows()  
#                 print("Switched to orientation detection mode")
#             elif key == ord('e'):
#                 mode = 'edge'
#                 cv2.destroyAllWindows()
#                 print("Switched to edge detection mode")
#             elif key == ord('d'):
#                 mode = 'block'
#                 cv2.destroyAllWindows()
#                 print("Switched to block detection mode")

#     except KeyboardInterrupt:
#         pass
    
#     except Exception as E:
#         print(E)

#     finally:
#         detector.stop()

def main():
    color_params_file = './colour_params.csv'
    detector = DetectionSystem(color_params_file)
    detector.run(mode='block')

if __name__ == "__main__":
    main()
