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
    def __init__(self, color_params_file, sim_mode=False,
                 min_depth=0.0, max_depth=4.0, box_edge_max_depth=0.35,
                 use_morphology=True):
        """
        min_depth, max_depth: detection range in meters.
            Objects closer than min_depth or farther than max_depth are ignored.
        box_edge_max_depth: separate (closer) depth limit for detect_box_edge(),
            since box edge detection is used when the arm is right next to the box.
        use_morphology: apply closing to fill small holes. Set to False
            if small/distant objects are being removed.
        """
        self.col_array = ['r', 'g', 'b', 'p', 'y']
        self.current_index = 0
        self.min_block_size = 20
        self.sim_mode = sim_mode
        self.use_morphology = use_morphology
        
        # Detection range, in meters. Converted to mm internally for real camera.
        self.min_depth_m = min_depth
        self.max_depth_m = max_depth
        self.box_edge_max_depth_m = box_edge_max_depth
        
        # block/box size classification threshold (meters)
        # block ~2-3cm, box ~30cm, order of magnitude apart, 10cm is a safe cutoff
        self.size_threshold = 0.10
        
        self.color_params = self._load_color_params(color_params_file)
        
        # HSV params for simulation mode
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
        
        if not sim_mode and HAS_REALSENSE:
            self.pipeline, self.align = self._initialize_camera()
        else:
            self.pipeline = None
            self.align = None
        
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
    
    def _get_intrinsics(self):
        """Unified interface: get camera intrinsics (fx, fy, cx, cy)"""
        if self.sim_mode:
            return 277.0, 277.0, 320.0, 240.0
        else:
            color_frame = self._current_result['color_frame']
            intr = color_frame.profile.as_video_stream_profile().intrinsics
            return intr.fx, intr.fy, intr.ppx, intr.ppy
    
    def _get_depth_at_contour(self, cnt, depth_image):
        """Use median depth over contour area for robustness. Returns meters, None if invalid."""
        contour_mask = np.zeros(depth_image.shape[:2], dtype=np.uint8)
        cv2.drawContours(contour_mask, [cnt], -1, 255, -1)
        depths = depth_image[contour_mask > 0]
        
        if self.sim_mode:
            depths = depths[(depths > self.min_depth_m) & 
                            (depths < self.max_depth_m) & 
                            ~np.isnan(depths)]
            if len(depths) == 0:
                return None
            return float(np.median(depths))  # sim depth already in meters
        else:
            min_mm = self.min_depth_m * 1000
            max_mm = self.max_depth_m * 1000
            depths = depths[(depths > min_mm) & (depths < max_mm)]
            if len(depths) == 0:
                return None
            return float(np.median(depths)) / 1000.0  # mm -> m
    
    def _classify_by_size(self, real_w, real_h):
        """Classify as block or box based on real-world size"""
        return 'block' if max(real_w, real_h) < self.size_threshold else 'box'
    
    def _build_masks_sim(self, hsv_image):
        masks = []
        for color_key in self.col_array:
            params = self.sim_hsv_params[color_key]
            lower1 = np.array(params['lower1'])
            upper1 = np.array(params['upper1'])
            mask = cv2.inRange(hsv_image, lower1, upper1)
            
            if params['lower2'] is not None:
                lower2 = np.array(params['lower2'])
                upper2 = np.array(params['upper2'])
                mask2 = cv2.inRange(hsv_image, lower2, upper2)
                mask = cv2.bitwise_or(mask, mask2)
            
            masks.append(mask)
        return masks
    
    def _build_masks_real(self, hsv_image):
        p = self.color_params
        
        mask_r1 = cv2.inRange(hsv_image,
            np.array([0, p['LCRHSV'][1], p['LCRHSV'][2]]),
            np.array([p['UCRHSV'][0], p['UCRHSV'][1], p['UCRHSV'][2]]))
        mask_r2 = cv2.inRange(hsv_image,
            np.array([p['LCRHSV'][0], p['LCRHSV'][1], p['LCRHSV'][2]]),
            np.array([180, p['UCRHSV'][1], p['UCRHSV'][2]]))
        mask_r = cv2.bitwise_or(mask_r1, mask_r2)
        
        mask_g = cv2.inRange(hsv_image,
            np.array(p['LCGHSV']), np.array(p['UCGHSV']))
        mask_b = cv2.inRange(hsv_image,
            np.array(p['LCBHSV']), np.array(p['UCBHSV']))
        mask_p = cv2.inRange(hsv_image,
            np.array(p['LCPHSV']), np.array(p['UCPHSV']))
        mask_y = cv2.inRange(hsv_image,
            np.array(p['LCYHSV']), np.array(p['UCYHSV']))
        
        return [mask_r, mask_g, mask_b, mask_p, mask_y]
    
    def _apply_morphology(self, masks):
        """Only apply closing to fill small holes inside objects.
        Erosion is skipped because it destroys small/distant objects."""
        if not self.use_morphology:
            return masks
        
        struc_elem = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        
        for i in range(len(masks)):
            # Closing = dilate then erode: fills small holes, preserves object size
            masks[i] = cv2.morphologyEx(masks[i], cv2.MORPH_CLOSE, struc_elem, iterations=1)
        return masks
    
    def _update_frame(self):
        if self.sim_mode:
            return self._update_frame_sim()
        else:
            return self._update_frame_real()
    
    def _update_frame_sim(self):
        if self.color_image is None or self.depth_image is None:
            return False
        
        color_image = self.color_image
        depth_image = self.depth_image
        
        depth_clean = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)
        depth_mm = (depth_clean * 1000).astype(np.uint16)
        min_mm = int(self.min_depth_m * 1000)
        max_mm = int(self.max_depth_m * 1000)
        depth_mask = (depth_mm > min_mm) & (depth_mm < max_mm)
        filtered_color = cv2.bitwise_and(color_image, color_image,
                                          mask=depth_mask.astype(np.uint8) * 255)
        
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
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            return False
        
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        min_mm = int(self.min_depth_m * 1000)
        max_mm = int(self.max_depth_m * 1000)
        depth_mask = (depth_image > min_mm) & (depth_image < max_mm)
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
        """Detect objects of the currently selected color only"""
        if not self._update_frame():
            return None
        
        mask = self._current_result['masks'][self.current_index]
        color_image = self._current_result['color_image']
        filtered_color = self._current_result['filtered_color']
        depth_image = self._current_result['depth_image']
        
        fx, fy, cx_cam, cy_cam = self._get_intrinsics()
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blocks = []
        
        for cnt in contours:
            if cv2.contourArea(cnt) < 50:
                continue
            
            M = cv2.moments(cnt)
            if M["m00"] <= 0:
                continue
            
            px = int(M["m10"] / M["m00"])
            py = int(M["m01"] / M["m00"])
            cv2.circle(color_image, (px, py), 3, (0, 0, 255), -1)
            
            depth = self._get_depth_at_contour(cnt, depth_image)
            if depth is None:
                continue
            
            x_3d = (px - cx_cam) * depth / fx
            y_3d = (py - cy_cam) * depth / fy
            point_3d = [x_3d, y_3d, depth]
            self.goal_point_3d = point_3d
            
            x, y, w, h = cv2.boundingRect(cnt)
            real_w = w * depth / fx
            real_h = h * depth / fy
            obj_type = self._classify_by_size(real_w, real_h)
            
            blocks.append({
                'centroid': (px, py),
                'position_3d': point_3d,
                'depth': depth,
                'bounding_box': (x, y, w, h),
                'real_size': (real_w, real_h),
                'real_area': real_w * real_h,
                'type': obj_type,
            })
        
        cv2.imshow("Color", color_image)
        cv2.imshow("Mask", mask)
        cv2.imshow('Filtered Color Image', filtered_color)
        
        return blocks
    
    def detect_all_blocks(self, classify_size=True):
        """
        Detect all colored objects. Use depth + pixel area to get real size,
        then classify as block / box.
        
        Returns a list of dicts. Each object contains:
          color, type ('block'|'box'), centroid, position_3d,
          depth, bounding_box, real_size, real_area
        """
        if not self._update_frame():
            return None
        
        color_image = self._current_result['color_image']
        filtered_color = self._current_result['filtered_color']
        depth_image = self._current_result['depth_image']
        
        fx, fy, cx_cam, cy_cam = self._get_intrinsics()
        
        all_objects = []
        combined_mask = np.zeros_like(self._current_result['masks'][0])
        visualization = color_image.copy()
        
        for i, color_name in enumerate(self.col_array):
            mask = self._current_result['masks'][i]
            combined_mask = cv2.bitwise_or(combined_mask, mask)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for cnt in contours:
                area_px = cv2.contourArea(cnt)
                if area_px < 50:
                    continue
                
                M = cv2.moments(cnt)
                if M["m00"] <= 0:
                    continue
                
                px = int(M["m10"] / M["m00"])
                py = int(M["m01"] / M["m00"])
                
                depth = self._get_depth_at_contour(cnt, depth_image)
                if depth is None:
                    continue
                
                # 3D position
                x_3d = (px - cx_cam) * depth / fx
                y_3d = (py - cy_cam) * depth / fy
                point_3d = [x_3d, y_3d, depth]
                
                # Pixel bounding box -> real-world size
                x, y, w, h = cv2.boundingRect(cnt)
                real_w = w * depth / fx
                real_h = h * depth / fy
                real_area = real_w * real_h
                
                obj_type = self._classify_by_size(real_w, real_h) if classify_size else None
                
                all_objects.append({
                    'color': color_name,
                    'type': obj_type,
                    'centroid': (px, py),
                    'position_3d': point_3d,
                    'depth': depth,
                    'bounding_box': (x, y, w, h),
                    'pixel_size': (w, h),
                    'real_size': (real_w, real_h),
                    'real_area': real_area,
                })
                
                # Visualization
                box_color = (0, 255, 0) if obj_type == 'block' else (0, 0, 255)
                cv2.rectangle(visualization, (x, y), (x+w, y+h), box_color, 2)
                label = f"{color_name}/{obj_type} {real_w*100:.1f}x{real_h*100:.1f}cm"
                cv2.putText(visualization, label, (x, max(y-5, 15)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, box_color, 1)
        
        cv2.imshow("All Objects", visualization)
        cv2.imshow("Combined Mask", combined_mask)
        cv2.imshow("Filtered Color", filtered_color)
        
        return all_objects
    
    def detect_block_orientation(self, gripper_max_width=0.05):
        """
        Assumes blocks are placed only horizontally or vertically.
        Decides which direction the gripper should close from.
        
        gripper_max_width: maximum gripper opening (meters).
                           If the short edge exceeds this, the block is not graspable.
        
        Each block contains:
          long_edge, short_edge: real long/short edge lengths (meters)
          grasp_dir: 'horizontal' or 'vertical' (direction the gripper closes)
          can_grasp: whether the short edge fits within the gripper opening
        """
        if not self._update_frame():
            return None
        
        mask = self._current_result['masks'][self.current_index]
        color_image = self._current_result['color_image']
        filtered_color = self._current_result['filtered_color']
        depth_image = self._current_result['depth_image']
        
        fx, fy, cx_cam, cy_cam = self._get_intrinsics()
        visualization = color_image.copy()
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blocks = []
        
        for cnt in contours:
            if cv2.contourArea(cnt) < 50:
                continue
            
            x, y, w, h = cv2.boundingRect(cnt)
            if w < self.min_block_size or h < self.min_block_size:
                continue
            
            M = cv2.moments(cnt)
            if M["m00"] <= 0:
                continue
            px = int(M["m10"] / M["m00"])
            py = int(M["m01"] / M["m00"])
            
            depth = self._get_depth_at_contour(cnt, depth_image)
            if depth is None:
                continue
            
            # 3D position
            x_3d = (px - cx_cam) * depth / fx
            y_3d = (py - cy_cam) * depth / fy
            point_3d = [x_3d, y_3d, depth]
            
            # Pixel width/height -> real-world size
            real_w = w * depth / fx
            real_h = h * depth / fy
            
            # Determine direction: shorter side is the short edge; gripper closes along it
            if w > h:
                # Horizontal block: short edge is vertical -> gripper closes vertically
                grasp_dir = 'vertical'
                short_real = real_h
                long_real = real_w
            else:
                # Vertical block: short edge is horizontal -> gripper closes horizontally
                grasp_dir = 'horizontal'
                short_real = real_w
                long_real = real_h
            
            can_grasp = short_real < gripper_max_width
            
            blocks.append({
                'centroid': (px, py),
                'position_3d': point_3d,
                'depth': depth,
                'bounding_box': (x, y, w, h),
                'long_edge': long_real,
                'short_edge': short_real,
                'grasp_dir': grasp_dir,
                'can_grasp': can_grasp,
            })
            
            # Visualization
            box_color = (0, 255, 0) if can_grasp else (0, 0, 255)
            cv2.rectangle(visualization, (x, y), (x+w, y+h), box_color, 2)
            cv2.circle(visualization, (px, py), 3, (0, 0, 255), -1)
            
            # Draw gripper direction indicator (two small arrows)
            arrow_len = 20
            if grasp_dir == 'horizontal':
                cv2.arrowedLine(visualization, (px-arrow_len-5, py), (px-5, py), (255, 255, 0), 2)
                cv2.arrowedLine(visualization, (px+arrow_len+5, py), (px+5, py), (255, 255, 0), 2)
            else:
                cv2.arrowedLine(visualization, (px, py-arrow_len-5), (px, py-5), (255, 255, 0), 2)
                cv2.arrowedLine(visualization, (px, py+arrow_len+5), (px, py+5), (255, 255, 0), 2)
            
            label = f"L={long_real*100:.1f} S={short_real*100:.1f}cm {grasp_dir}"
            cv2.putText(visualization, label, (x, max(y-5, 15)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, box_color, 1)
        
        cv2.imshow("Orientation", visualization)
        cv2.imshow("Mask", mask)
        cv2.imshow('Filtered Color Image', filtered_color)
        
        return blocks
    
    def detect_box_edge(self):
        if not self._update_frame():
            return None
        
        color_image = self._current_result['color_image']
        depth_image = self._current_result['depth_image']
        
        area_threshold = 0.015
        
        # depth_image unit differs: sim = meters, real = millimeters
        if self.sim_mode:
            depth_mask = (depth_image > self.min_depth_m) & \
                         (depth_image < self.box_edge_max_depth_m)
        else:
            min_mm = self.min_depth_m * 1000
            max_mm = self.box_edge_max_depth_m * 1000
            depth_mask = (depth_image > min_mm) & (depth_image < max_mm)
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
    
    def _pick_target_demo(self):
        """Triggered by the 't' key: pick the best grasp target of the current color
        and print a grasp command. Simulates how upper-level control code would
        consume the detection results."""
        color_name = self.col_array[self.current_index]
        print(f"\n=== Picking [{color_name}] block as grasp target ===")
        
        blocks = self.detect_block_orientation(gripper_max_width=0.05)
        
        if not blocks:
            print("  -> No block detected")
            return None
        
        # Filter: keep only small (i.e. block, not box) and graspable objects
        candidates = [b for b in blocks 
                      if b['can_grasp'] and max(b['long_edge'], b['short_edge']) < 0.10]
        
        if not candidates:
            too_big = [b for b in blocks if not b['can_grasp']]
            if too_big:
                print(f"  -> Found {len(too_big)} block(s), but short edge exceeds gripper opening")
            else:
                print("  -> No suitable block")
            return None
        
        # Pick the nearest one
        target = min(candidates, key=lambda b: b['depth'])
        
        xyz = target['position_3d']
        print(f"  Position (camera frame): x={xyz[0]:.3f}  y={xyz[1]:.3f}  z={xyz[2]:.3f} m")
        print(f"  Size: {target['long_edge']*100:.1f} x {target['short_edge']*100:.1f} cm")
        print(f"  Gripper direction: {target['grasp_dir']}")
        print(f"  -> Send to arm: goto({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}), "
              f"gripper_{target['grasp_dir']}(), close_gripper()")
        
        return target
    
    def stop(self):
        if self.pipeline:
            self.pipeline.stop()
        cv2.destroyAllWindows()
        print("Camera stopped")
    
    def run(self, mode='block'):
        last_signature = None  # scene "fingerprint"; print only when it changes
        
        try:
            while True:
                if mode == 'block':
                    blocks = self.detect_block()
                    # Fingerprint: sorted tuple of each object's type
                    sig = tuple(sorted(b['type'] for b in blocks)) if blocks else ()
                    if sig != last_signature:
                        last_signature = sig
                        if blocks:
                            print(f"--- {len(blocks)} object(s) ---")
                            for b in blocks:
                                print(f"  [{b['type']}] at {b['centroid']}, "
                                      f"size={b['real_size'][0]*100:.1f}x{b['real_size'][1]*100:.1f}cm, "
                                      f"depth={b['depth']:.3f}m")
                        else:
                            print("--- nothing detected ---")
                
                elif mode == 'all':
                    objs = self.detect_all_blocks()
                    # Fingerprint: sorted tuple of all color/type combinations
                    sig = tuple(sorted(f"{o['color']}/{o['type']}" for o in objs)) if objs else ()
                    if sig != last_signature:
                        last_signature = sig
                        if objs:
                            print(f"--- {len(objs)} object(s) ---")
                            for o in objs:
                                print(f"  [{o['color']}/{o['type']}] "
                                      f"size={o['real_size'][0]*100:.1f}x{o['real_size'][1]*100:.1f}cm, "
                                      f"depth={o['depth']:.3f}m")
                        else:
                            print("--- nothing detected ---")
                
                elif mode == 'orientation':
                    blocks = self.detect_block_orientation()
                    # Fingerprint: grasp direction + graspable flag
                    sig = tuple((b['grasp_dir'], b['can_grasp']) for b in blocks) if blocks else ()
                    if sig != last_signature:
                        last_signature = sig
                        if blocks:
                            for b in blocks:
                                grasp = "OK" if b['can_grasp'] else "TOO BIG"
                                print(f"  L={b['long_edge']*100:.1f}cm S={b['short_edge']*100:.1f}cm "
                                      f"grip={b['grasp_dir']} [{grasp}]")
                
                elif mode == 'edge':
                    box = self.detect_box_edge()
                    sig = 'box' if box else 'none'
                    if sig != last_signature:
                        last_signature = sig
                        if box:
                            print(f"Box center: {box['center']}, area: {box['area']:.2f}")
                
                key = cv2.waitKey(1)
                if key == 27 or key == ord('q'):
                    break
                elif key == ord('r'):
                    self.current_index = 0
                    last_signature = None
                    print(f"Color: red (mode={mode})")
                elif key == ord('g'):
                    self.current_index = 1
                    last_signature = None
                    print(f"Color: green (mode={mode})")
                elif key == ord('b'):
                    self.current_index = 2
                    last_signature = None
                    print(f"Color: blue (mode={mode})")
                elif key == ord('p'):
                    self.current_index = 3
                    last_signature = None
                    print(f"Color: purple (mode={mode})")
                elif key == ord('y'):
                    self.current_index = 4
                    last_signature = None
                    print(f"Color: yellow (mode={mode})")
                elif key == ord('t'):
                    # Trigger a target pick: nearest graspable block
                    self._pick_target_demo()
                elif key == ord('a'):
                    mode = 'all'
                    last_signature = None
                    cv2.destroyAllWindows()
                    print("Switched to detect-all mode")
                elif key == ord('o'):
                    mode = 'orientation'
                    last_signature = None
                    cv2.destroyAllWindows()
                    print("Switched to orientation detection mode")
                elif key == ord('e'):
                    mode = 'edge'
                    last_signature = None
                    cv2.destroyAllWindows()
                    print("Switched to edge detection mode")
                elif key == ord('d'):
                    mode = 'block'
                    last_signature = None
                    cv2.destroyAllWindows()
                    print("Switched to block detection mode")
        
        except KeyboardInterrupt:
            pass
        
        finally:
            self.stop()


def main():
    color_params_file = '/home/student15/rspd_venv/src/colour_params.csv'
    detector = DetectionSystem(color_params_file, sim_mode=False)
    detector.run(mode='block')  # default: single-color detection (color keys work)


if __name__ == "__main__":
    main()
