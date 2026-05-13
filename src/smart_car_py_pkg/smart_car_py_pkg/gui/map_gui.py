# map_gui.py
import os
import yaml
import numpy as np
import cv2
import dearpygui.dearpygui as dpg
import math
import heapq
from collections import deque

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    pass

class MapManager:
    def __init__(self, texture_tag="map_texture", width=500, height=420):
        self.texture_tag = texture_tag
        self.width = width
        self.height = height
        
        self.map_image_base = None
        self.map_gray = None
        self.map_info = None
        self.robot_pose = (0.0, 0.0, 0.0)
        self.lidar_local_points = []
        
        self.nav_target_pose = None
        self.nav_target_name = None
        self.nav_landmarks = {}
        self.last_path_status = "NO_TARGET"
        
        self.drag_start_tex = None
        self.drag_current_tex = None

        self.user_zoom_factor = 1.0   
        self.pan_x = 0.0              
        self.pan_y = 0.0              
        
        self.current_scale = 1.0
        self.current_x_offset = 0
        self.current_y_offset = 0
        
        self.is_dragging = False
        self.last_mouse_pos = None

        with dpg.handler_registry():
            dpg.add_mouse_wheel_handler(callback=self.wheel_cb)

    def wheel_cb(self, sender, app_data):
        if dpg.is_item_hovered("map_image_item"):
            zoom_speed = 1.15
            old_zoom = self.user_zoom_factor
            
            if app_data > 0:
                self.user_zoom_factor *= zoom_speed
            elif app_data < 0:
                self.user_zoom_factor /= zoom_speed
                
            self.user_zoom_factor = max(0.2, min(self.user_zoom_factor, 15.0))
            
            mouse_pos = dpg.get_mouse_pos(local=False)
            item_state = dpg.get_item_state("map_image_item")
            if item_state and 'rect_min' in item_state:
                rect_min = item_state['rect_min']
                win_x = (mouse_pos[0] - rect_min[0]) * (self.width / 480.0)
                win_y = (mouse_pos[1] - rect_min[1]) * (self.height / 400.0)
                
                scale_ratio = self.user_zoom_factor / old_zoom
                
                dx = win_x - (self.width / 2.0 + self.pan_x)
                dy = win_y - (self.height / 2.0 + self.pan_y)
                
                self.pan_x -= dx * (scale_ratio - 1.0)
                self.pan_y -= dy * (scale_ratio - 1.0)

    def load_map_from_yaml(self, yaml_path):
        if not os.path.exists(yaml_path):
            print(f"[MapManager] Error: Map yaml file not found: {yaml_path}")
            return False
            
        try:
            with open(yaml_path, 'r') as f:
                self.map_info = yaml.safe_load(f)
            
            image_path = self.map_info.get('image', '')
            if not os.path.isabs(image_path):
                yaml_dir = os.path.dirname(yaml_path)
                image_path = os.path.join(yaml_dir, image_path)
                
            if not os.path.exists(image_path):
                print(f"[MapManager] Error: Map image file not found: {image_path}")
                return False
                
            img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                print(f"[MapManager] Error: Failed to load map image: {image_path}")
                return False
                
            rgba = cv2.cvtColor(img, cv2.COLOR_GRAY2RGBA)
            rgba[:, :, 3] = 255
            
            self.map_gray = img.copy()
            self.map_image_base = rgba
            
            self.user_zoom_factor = 1.0
            self.pan_x = 0.0
            self.pan_y = 0.0
            
            print(f"[MapManager] Successfully loaded and auto-cropped map from {yaml_path}")
            self.update_display()
            return True
            
        except Exception as e:
            print(f"[MapManager] Exception loading map: {e}")
            return False

    def update_robot_pose(self, x, y, yaw=0.0):
        self.robot_pose = (x, y, yaw)
        self.update_display()

    def set_nav_target(self, x, y, name):
        if x is None or y is None:
            self.nav_target_pose = None
            self.nav_target_name = None
            self.last_path_status = "NO_TARGET"
        else:
            self.nav_target_pose = (x, y)
            self.nav_target_name = name
            self.last_path_status = "PENDING"
        self.update_display()

    def set_nav_landmarks(self, landmarks):
        self.nav_landmarks = {
            name: (float(x), float(y))
            for name, coords in landmarks.items()
            for x, y in [coords]
            if x is not None and y is not None
        }
        self.update_display()

    def get_real_coords(self, win_x, win_y):
        if self.map_info is None or self.map_image_base is None:
            return None, None
            
        orig_h = self.map_image_base.shape[0]
        res = self.map_info.get('resolution', 0.05)
        origin_x = self.map_info['origin'][0]
        origin_y = self.map_info['origin'][1]
        
        map_px = (win_x - self.current_x_offset) / self.current_scale
        map_py = (win_y - self.current_y_offset) / self.current_scale
        
        real_x = origin_x + map_px * res
        real_y = origin_y + (orig_h - map_py) * res
        return real_x, real_y

    def _world_to_pixel(self, x, y):
        if self.map_info is None or self.map_image_base is None:
            return None

        orig_h = self.map_image_base.shape[0]
        res = self.map_info.get('resolution', 0.05)
        origin_x = self.map_info['origin'][0]
        origin_y = self.map_info['origin'][1]

        px = int(round((x - origin_x) / res))
        py = int(round(orig_h - ((y - origin_y) / res)))
        return px, py

    def _build_free_mask(self):
        if self.map_gray is None:
            return None

        # map_saver PGM 기준: free는 254/255, unknown은 205, occupied는 0.
        free_mask = self.map_gray >= 250
        obstacle_mask = ~free_mask
        kernel = np.ones((3, 3), np.uint8)
        inflated_obstacles = cv2.dilate(obstacle_mask.astype(np.uint8), kernel, iterations=1).astype(bool)
        return ~inflated_obstacles

    def _nearest_free_pixel(self, free_mask, start):
        h, w = free_mask.shape
        sx = min(max(int(start[0]), 0), w - 1)
        sy = min(max(int(start[1]), 0), h - 1)

        if free_mask[sy, sx]:
            return sx, sy

        queue = deque([(sx, sy)])
        visited = {(sx, sy)}
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

        while queue:
            x, y = queue.popleft()
            for dx, dy in neighbors:
                nx, ny = x + dx, y + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h or (nx, ny) in visited:
                    continue
                if free_mask[ny, nx]:
                    return nx, ny
                visited.add((nx, ny))
                queue.append((nx, ny))

        return None

    def _plan_path_pixels(self, start, goal):
        free_mask = self._build_free_mask()
        if free_mask is None:
            return None

        start = self._nearest_free_pixel(free_mask, start)
        goal = self._nearest_free_pixel(free_mask, goal)
        if start is None or goal is None:
            return None

        h, w = free_mask.shape
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}
        neighbors = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
        ]

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return self._simplify_path(path)

            cx, cy = current
            for dx, dy, move_cost in neighbors:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h or not free_mask[ny, nx]:
                    continue

                # 대각선이 벽 모서리를 뚫고 지나가지 않도록 인접 직교칸도 확인.
                if dx != 0 and dy != 0:
                    if not free_mask[cy, nx] or not free_mask[ny, cx]:
                        continue

                neighbor = (nx, ny)
                tentative_g = g_score[current] + move_cost
                if tentative_g >= g_score.get(neighbor, float('inf')):
                    continue

                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                heuristic = math.hypot(goal[0] - nx, goal[1] - ny)
                heapq.heappush(open_set, (tentative_g + heuristic, neighbor))

        return None

    def _simplify_path(self, path):
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        last_dx = path[1][0] - path[0][0]
        last_dy = path[1][1] - path[0][1]

        for i in range(2, len(path)):
            dx = path[i][0] - path[i - 1][0]
            dy = path[i][1] - path[i - 1][1]
            if dx != last_dx or dy != last_dy:
                simplified.append(path[i - 1])
                last_dx, last_dy = dx, dy

        simplified.append(path[-1])
        return simplified

    def _draw_dashed_line(self, image, start, end, color, thickness, dash_len):
        x1, y1 = start
        x2, y2 = end
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist <= 0:
            return

        dash_len = max(4, dash_len)
        steps = max(1, int(dist / dash_len))
        for i in range(steps):
            if i % 2 != 0:
                continue
            t1 = i / steps
            t2 = min((i + 1) / steps, 1.0)
            sx = int(x1 + (x2 - x1) * t1)
            sy = int(y1 + (y2 - y1) * t1)
            ex = int(x1 + (x2 - x1) * t2)
            ey = int(y1 + (y2 - y1) * t2)
            cv2.line(image, (sx, sy), (ex, ey), color, thickness, lineType=cv2.LINE_AA)

    def _draw_dashed_path(self, image, points, color, thickness, dash_len):
        if len(points) < 2:
            return

        for i in range(1, len(points)):
            self._draw_dashed_line(image, points[i - 1], points[i], color, thickness, dash_len)

    def _draw_nav_label(self, image, text, x, y, font_scale, color):
        font_paths = [
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
            "/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
            "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
            "/usr/share/fonts/truetype/fonts-nanum/NanumGothic.ttf",
        ]
        font_path = next((p for p in font_paths if os.path.exists(p)), None)

        if font_path:
            try:
                img_pil = Image.fromarray(image)
                draw = ImageDraw.Draw(img_pil)
                font_size = max(11, int(15 * font_scale))
                font = ImageFont.truetype(font_path, font_size)
                bbox = draw.textbbox((0, 0), text, font=font)
                tw = bbox[2] - bbox[0]
                th = bbox[3] - bbox[1]
                text_x = x - tw // 2
                text_y = y + 5
                draw.rectangle(
                    [text_x - 4, text_y - 2, text_x + tw + 4, text_y + th + 4],
                    fill=(255, 255, 255, 210),
                    outline=(0, 0, 0, 255),
                    width=max(1, int(font_scale)),
                )
                draw.text((text_x, text_y), text, font=font, fill=(0, 0, 0, 255))
                return np.array(img_pil)
            except Exception as e:
                print(f"PIL Text fallback: {e}")

        cv2.putText(
            image,
            text,
            (x - 15, y + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55 * font_scale,
            (0, 0, 0, 255),
            max(2, int(2 * font_scale)),
        )
        cv2.putText(
            image,
            text,
            (x - 15, y + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55 * font_scale,
            (255, 255, 255, 255),
            max(1, int(font_scale)),
        )
        return image

    def _draw_nav_marker(self, image, x, y, name, selected=False):
        pin_r = max(5, int((9 if selected else 6) * self.user_zoom_factor))
        color = (0, 0, 255, 255) if selected else (255, 120, 0, 255)
        cv2.circle(image, (x, y - pin_r), pin_r, color, -1)
        cv2.circle(image, (x, y - pin_r), max(2, int(pin_r * 0.4)), (255, 255, 255, 255), -1)
        cv2.line(image, (x, y), (x, y - pin_r), color, max(2, int(2 * self.user_zoom_factor)))
        return self._draw_nav_label(image, name, x, y, self.user_zoom_factor, color)

    def update_lidar(self, ranges, angle_min, angle_increment, range_min, range_max):
        points = []
        if range_min <= 0.0: range_min = 0.05
        if range_max <= 0.0: range_max = 30.0
            
        for i, r in enumerate(ranges):
            if range_min <= r <= range_max and not np.isinf(r) and not np.isnan(r):
                angle = angle_min + i * angle_increment
                points.append((r * math.cos(angle), r * math.sin(angle)))
        self.lidar_local_points = points
        self.update_display()

    def update_display(self):
        if self.map_image_base is None or self.map_info is None:
            return

        if dpg.is_mouse_button_down(dpg.mvMouseButton_Right) or dpg.is_mouse_button_down(dpg.mvMouseButton_Middle):
            if dpg.is_item_hovered("map_image_item") or self.is_dragging:
                self.is_dragging = True
                mouse_pos = dpg.get_mouse_pos(local=False)
                if self.last_mouse_pos is not None:
                    dx = (mouse_pos[0] - self.last_mouse_pos[0]) * (self.width / 480.0)
                    dy = (mouse_pos[1] - self.last_mouse_pos[1]) * (self.height / 400.0)
                    self.pan_x += dx
                    self.pan_y += dy
                self.last_mouse_pos = mouse_pos
        else:
            self.is_dragging = False
            self.last_mouse_pos = None

        # ========================================================
        # [수정됨] 전체 맵이 다 보이면서(min) 여백 없이 최대한 꽉 차게(1.0)
        # ========================================================
        orig_h, orig_w = self.map_image_base.shape[:2]
        target_w, target_h = self.width, self.height
        res = self.map_info.get('resolution', 0.05)
        
        # 90% 제한을 지우고 100% 꽉 차게(1.0 배율) 계산합니다.
        base_scale = min(target_w / orig_w, target_h / orig_h)
        
        final_scale = base_scale * self.user_zoom_factor
        
        new_w = int(orig_w * final_scale)
        new_h = int(orig_h * final_scale)
        
        base_x_offset = (target_w - new_w) / 2.0
        base_y_offset = (target_h - new_h) / 2.0
        
        final_x_offset = int(base_x_offset + self.pan_x)
        final_y_offset = int(base_y_offset + self.pan_y)
        
        self.current_scale = final_scale
        self.current_x_offset = final_x_offset
        self.current_y_offset = final_y_offset
        # ========================================================
        
        scaled_map = cv2.resize(self.map_image_base, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
        
        origin_x = self.map_info['origin'][0]
        origin_y = self.map_info['origin'][1]
        
        px = int(((self.robot_pose[0] - origin_x) / res) * final_scale)
        py = int((orig_h - (self.robot_pose[1] - origin_y) / res) * final_scale)
        yaw = self.robot_pose[2]

        if 0 <= px < new_w and 0 <= py < new_h:
            for lx, ly in self.lidar_local_points:
                map_x = self.robot_pose[0] + lx * math.cos(yaw) - ly * math.sin(yaw)
                map_y = self.robot_pose[1] + lx * math.sin(yaw) + ly * math.cos(yaw)
                
                lpx = int(((map_x - origin_x) / res) * final_scale)
                lpy = int((orig_h - (map_y - origin_y) / res) * final_scale)
                if 0 <= lpx < new_w and 0 <= lpy < new_h:
                    cv2.circle(scaled_map, (lpx, lpy), radius=max(3, int(3.0*self.user_zoom_factor/2.0)), color=(0, 255, 0, 255), thickness=-1)

            robot_size_px = max(4, int((0.15 / res) * final_scale)) 
            head_tip = robot_size_px * 2
            
            arrow_pts_local = np.array([
                [head_tip, 0],              
                [-robot_size_px, -robot_size_px], 
                [-int(robot_size_px * 0.7), 0],                      
                [-robot_size_px, robot_size_px]  
            ], dtype=np.float32)
            
            rotated_pts = []
            for pt in arrow_pts_local:
                rx = pt[0] * math.cos(-yaw) - pt[1] * math.sin(-yaw)
                ry = pt[0] * math.sin(-yaw) + pt[1] * math.cos(-yaw)
                rotated_pts.append([px + rx, py + ry])
            
            rotated_pts = np.array(rotated_pts, dtype=np.int32).reshape((-1, 1, 2))
            cv2.fillPoly(scaled_map, [rotated_pts], color=(0, 0, 255, 255))
            cv2.polylines(scaled_map, [rotated_pts], isClosed=True, color=(255, 255, 255, 255), thickness=max(1, int(1 * self.user_zoom_factor)))

        selected_pose_key = None
        if self.nav_target_pose is not None:
            selected_pose_key = (
                round(self.nav_target_pose[0], 4),
                round(self.nav_target_pose[1], 4),
            )

        for landmark_name, (landmark_x, landmark_y) in self.nav_landmarks.items():
            landmark_key = (round(landmark_x, 4), round(landmark_y, 4))
            if selected_pose_key is not None and landmark_key == selected_pose_key:
                continue

            ltx = int(((landmark_x - origin_x) / res) * final_scale)
            lty = int((orig_h - (landmark_y - origin_y) / res) * final_scale)
            if 0 <= ltx < new_w and 0 <= lty < new_h:
                scaled_map = self._draw_nav_marker(
                    scaled_map,
                    ltx,
                    lty,
                    landmark_name,
                    selected=False,
                )

        if self.nav_target_pose is not None:
            tx = int(((self.nav_target_pose[0] - origin_x) / res) * final_scale)
            ty = int((orig_h - (self.nav_target_pose[1] - origin_y) / res) * final_scale)

            robot_px = self._world_to_pixel(self.robot_pose[0], self.robot_pose[1])
            target_px = self._world_to_pixel(self.nav_target_pose[0], self.nav_target_pose[1])
            planned_path = self._plan_path_pixels(robot_px, target_px) if robot_px and target_px else None

            if planned_path and len(planned_path) >= 2:
                path_points = [(int(x * final_scale), int(y * final_scale)) for x, y in planned_path]
                self._draw_dashed_path(
                    scaled_map,
                    path_points,
                    color=(50, 200, 50, 255),
                    thickness=max(2, int(3 * self.user_zoom_factor)),
                    dash_len=max(7, int(9 * self.user_zoom_factor)),
                )
                self.last_path_status = f"PATH_READY:{len(planned_path)}"
            elif 0 <= px < new_w and 0 <= py < new_h:
                self._draw_dashed_line(
                    scaled_map,
                    (px, py),
                    (tx, ty),
                    (50, 200, 50, 255),
                    max(2, int(2 * self.user_zoom_factor)),
                    max(7, int(9 * self.user_zoom_factor)),
                )
                self.last_path_status = "PATH_FALLBACK_STRAIGHT"
            else:
                self.last_path_status = "PATH_UNAVAILABLE"

            if self.nav_target_name:
                scaled_map = self._draw_nav_marker(
                    scaled_map,
                    tx,
                    ty,
                    self.nav_target_name,
                    selected=True,
                )

        # 빈 공간을 투명(Glass 테마)하게 생성
        final_canvas = np.zeros((target_h, target_w, 4), dtype=np.uint8)

        out_x_min = max(0, final_x_offset)
        out_y_min = max(0, final_y_offset)
        out_x_max = min(target_w, final_x_offset + new_w)
        out_y_max = min(target_h, final_y_offset + new_h)

        in_x_min = max(0, -final_x_offset)
        in_y_min = max(0, -final_y_offset)
        in_x_max = in_x_min + (out_x_max - out_x_min)
        in_y_max = in_y_min + (out_y_max - out_y_min)

        if out_x_min < out_x_max and out_y_min < out_y_max:
            final_canvas[out_y_min:out_y_max, out_x_min:out_x_max] = scaled_map[in_y_min:in_y_max, in_x_min:in_x_max]

        if self.drag_start_tex is not None:
            sx, sy = int(self.drag_start_tex[0]), int(self.drag_start_tex[1])
            cv2.circle(final_canvas, (sx, sy), 6, (255, 0, 0, 255), -1)
            if self.drag_current_tex is not None:
                cx, cy = int(self.drag_current_tex[0]), int(self.drag_current_tex[1])
                if (sx, sy) != (cx, cy):
                    cv2.arrowedLine(final_canvas, (sx, sy), (cx, cy), (255, 0, 0, 255), 3, tipLength=0.3)

        texture_data = final_canvas.astype(np.float32) / 255.0
        
        if dpg.does_item_exist(self.texture_tag):
            dpg.set_value(self.texture_tag, texture_data.flatten())
