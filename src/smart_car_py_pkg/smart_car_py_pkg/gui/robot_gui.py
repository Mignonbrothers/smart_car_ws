# robot_gui.py

import os
import platform
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage, BatteryState, LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import cv2
import dearpygui.dearpygui as dpg
import threading
from datetime import datetime
import time

from smart_car_py_pkg.gui.button_manager import (
    create_command_buttons,
    create_navigation_buttons,
    update_overlay_text,
)
from smart_car_py_pkg.gui.map_gui import MapManager

def get_wifi_strength():
    if platform.system() == "Linux":
        try:
            with open("/proc/net/wireless", "r") as f:
                lines = f.readlines()
                for line in lines[2:]:
                    data = line.split()
                    if len(data) >= 4:
                        level_str = data[3].replace('.', '')
                        try:
                            dbm = int(level_str)
                            if dbm <= -100:
                                return 0
                            elif dbm >= -50:
                                return 100
                            else:
                                return 2 * (dbm + 100)
                        except ValueError:
                            pass
        except Exception:
            pass
        return 0
    return 100

class RobotGuiNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        
        self.image_sub = self.create_subscription(
            CompressedImage, '/pan_tilt/debug_image/compressed', self.image_callback, 10)
        self.status_sub = self.create_subscription(
            String, '/robot_status', self.status_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, '/battery_state', self.battery_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_timer = self.create_timer(0.1, self.tf_timer_callback)

        self.command_pub = self.create_publisher(String, '/gui_command', 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        self.is_pose_estimating = False
        self.pose_start_pos = None

        self.get_logger().info("Robot Node Initialized. UI Ready.")

        self.map_manager = MapManager(texture_tag="map_texture", width=1000, height=740)

        try:
            package_share_dir = get_package_share_directory('smart_car_py_pkg')
        except Exception:
            package_share_dir = None
        
        possible_map_paths = [
            os.path.join(package_share_dir, 'gui', 'maps', 'map.yaml') if package_share_dir else None,
            os.path.join(os.path.dirname(os.path.abspath(__file__)), 'maps', 'map.yaml'),
            os.path.join(os.path.expanduser('~'), 'smart_car_ws', 'src', 'smart_car_py_pkg', 'smart_car_py_pkg', 'gui', 'maps', 'map.yaml'),
            os.path.join(os.path.expanduser('~'), 'smartcar_ws', 'src', 'smart_car_py_pkg', 'smart_car_py_pkg', 'gui', 'maps', 'map.yaml'),
        ]
        
        map_yaml_path = None
        for path in possible_map_paths:
            if path and os.path.exists(path):
                map_yaml_path = path
                break

        if map_yaml_path:
            self.map_manager.load_map_from_yaml(map_yaml_path)
        else:
            self.get_logger().error("Map yaml file not found in any of the expected paths.")

        self.last_battery_update_time = 0
        self.last_pose_log_time = 0
        
    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if img_bgr is not None:
                rgba = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGBA)
                rgba_resized = cv2.resize(rgba, (480, 270))
                texture_data = rgba_resized.astype(np.float32) / 255.0

                if dpg.does_item_exist("camera_texture"):
                    dpg.set_value("camera_texture", texture_data.flatten())
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')

    def scan_callback(self, msg):
        if hasattr(self, 'map_manager') and self.map_manager:
            self.map_manager.update_lidar(
                msg.ranges, 
                msg.angle_min, 
                msg.angle_increment, 
                msg.range_min, 
                msg.range_max
            )

    def status_callback(self, msg):
        if dpg.does_item_exist("status_val"):
            dpg.set_value("status_val", f"MODE: {msg.data}")
            self.add_log(f"[Status] {msg.data}")

    def tf_timer_callback(self):
        target_frames = [
            ('map', 'base_link'), 
            ('map', 'base_footprint'), 
            ('odom', 'base_link'), 
            ('odom', 'base_footprint')
        ]
        t = None
        
        for parent, child in target_frames:
            try:
                t = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                break  
            except TransformException:
                continue
                
        if t is None:
            return  
            
        x = round(t.transform.translation.x, 2)
        y = round(t.transform.translation.y, 2)
        
        q = t.transform.rotation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if dpg.does_item_exist("pose_val"):
            yaw_deg = round(math.degrees(yaw), 1)
            dpg.set_value("pose_val", f"X: {x}, Y: {y}")
            if dpg.does_item_exist("theta_val"):
                dpg.set_value("theta_val", f"{yaw_deg}°")

        current_time = time.time()
        if current_time - self.last_pose_log_time >= 5.0:
            self.last_pose_log_time = current_time
            self.add_log(f"[POSE] X:{x}, Y:{y}, Theta:{round(math.degrees(yaw), 1)}°")
        
        self.map_manager.update_robot_pose(t.transform.translation.x, t.transform.translation.y, yaw)

    def battery_callback(self, msg):
        current_time = time.time()
        if current_time - self.last_battery_update_time >= 30.0:
            self.last_battery_update_time = current_time
            if dpg.does_item_exist("battery_val"):
                pct = msg.percentage
                if pct <= 2.0:
                    pct *= 100
                
                pct = max(0.0, min(100.0, pct))
                
                dpg.set_value("battery_val", f"{int(pct)}%")
                self.add_log(f"[BATT] Battery {int(pct)}%")
                
                if pct > 50:
                    color = (0, 150, 50, 255) 
                elif pct > 20:
                    color = (255, 140, 0, 255)
                else:
                    color = (220, 20, 20, 255)
                    
                dpg.configure_item("battery_val", color=color)

    def send_command(self, cmd_str):
        msg = String()
        msg.data = cmd_str
        self.command_pub.publish(msg)
        self.add_log(f"[CMD] {cmd_str}")

    def add_log(self, text):
        if dpg.does_item_exist("log_listbox"):
            logs = dpg.get_value("log_listbox") or[]
            if isinstance(logs, str):
                logs = [logs] if logs else []
            current_time = datetime.now().strftime("%p %I:%M:%S")
            logs.append(f"[{current_time}] {text}")
            if len(logs) > 30:
                logs.pop(0)
            dpg.configure_item("log_listbox", items=logs)

# --- GUI 이벤트 콜백 ---
def map_mouse_down_cb(sender, app_data, user_data):
    ros_node = user_data
    if not getattr(ros_node, "is_pose_estimating", False): return
    if getattr(ros_node, "pose_start_pos", None) is not None: return
    if dpg.is_item_hovered("map_image_item"):
        ros_node.pose_start_pos = dpg.get_mouse_pos(local=False)
        
        rect_min = dpg.get_item_state("map_image_item")['rect_min']
        start_img_x = ros_node.pose_start_pos[0] - rect_min[0]
        start_img_y = ros_node.pose_start_pos[1] - rect_min[1]
        
        img_w_dpg, img_h_dpg = 960.0, 700.0
        map_tex_w, map_tex_h = 1000.0, 740.0
        click_x_tex = start_img_x * (map_tex_w / img_w_dpg)
        click_y_tex = start_img_y * (map_tex_h / img_h_dpg)
        
        if hasattr(ros_node, "map_manager"):
            ros_node.map_manager.drag_start_tex = (click_x_tex, click_y_tex)
            ros_node.map_manager.drag_current_tex = (click_x_tex, click_y_tex)
            ros_node.map_manager.update_display()

def map_mouse_drag_cb(sender, app_data, user_data):
    ros_node = user_data
    if not getattr(ros_node, "is_pose_estimating", False) or getattr(ros_node, "pose_start_pos", None) is None: return
    
    current_pos = dpg.get_mouse_pos(local=False)
    rect_min = dpg.get_item_state("map_image_item")['rect_min']
    curr_img_x = current_pos[0] - rect_min[0]
    curr_img_y = current_pos[1] - rect_min[1]
    
    img_w_dpg, img_h_dpg = 960.0, 700.0
    map_tex_w, map_tex_h = 1000.0, 740.0
    click_x_tex = curr_img_x * (map_tex_w / img_w_dpg)
    click_y_tex = curr_img_y * (map_tex_h / img_h_dpg)
    
    if hasattr(ros_node, "map_manager"):
        ros_node.map_manager.drag_current_tex = (click_x_tex, click_y_tex)
        ros_node.map_manager.update_display()

def map_mouse_release_cb(sender, app_data, user_data):
    ros_node = user_data
    if not getattr(ros_node, "is_pose_estimating", False) or getattr(ros_node, "pose_start_pos", None) is None: return
    
    end_pos = dpg.get_mouse_pos(local=False)
    
    rect_min = dpg.get_item_state("map_image_item")['rect_min']
    start_img_x = ros_node.pose_start_pos[0] - rect_min[0]
    start_img_y = ros_node.pose_start_pos[1] - rect_min[1]
    end_img_x = end_pos[0] - rect_min[0]
    end_img_y = end_pos[1] - rect_min[1]
    
    img_w_dpg, img_h_dpg = 960.0, 700.0 
    map_tex_w, map_tex_h = 1000.0, 740.0 
    
    click_x_tex = start_img_x * (map_tex_w / img_w_dpg)
    click_y_tex = start_img_y * (map_tex_h / img_h_dpg)
    
    manager = getattr(ros_node, "map_manager", None)
    if manager is None or manager.map_image_base is None or manager.map_info is None:
        ros_node.is_pose_estimating = False
        ros_node.pose_start_pos = None
        if manager:
            manager.drag_start_tex = None
            manager.drag_current_tex = None
            manager.update_display()
        return
    
    real_x, real_y = manager.get_real_coords(click_x_tex, click_y_tex)
    
    if real_x is not None and real_y is not None:
        dx = end_img_x - start_img_x
        dy = -(end_img_y - start_img_y)
        
        if dx == 0 and dy == 0:
            yaw = ros_node.map_manager.robot_pose[2] if hasattr(ros_node.map_manager, 'robot_pose') else 0.0
        else:
            yaw = math.atan2(dy, dx)
        
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = ros_node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = float(real_x)
        msg.pose.pose.position.y = float(real_y)
        half_yaw = yaw * 0.5
        msg.pose.pose.orientation.z = math.sin(half_yaw)
        msg.pose.pose.orientation.w = math.cos(half_yaw)
        
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        
        ros_node.initial_pose_pub.publish(msg)
        ros_node.add_log(f"[POSE] X:{real_x:.2f}, Y:{real_y:.2f}, Yaw:{math.degrees(yaw):.1f}°")
        update_overlay_text("위치 설정 완료", (0, 150, 50, 255))
        ros_node.add_log("[GUI] 위치 설정 완료")
        
        manager.update_robot_pose(real_x, real_y, yaw)
        
    ros_node.is_pose_estimating = False
    ros_node.pose_start_pos = None
    manager.drag_start_tex = None
    manager.drag_current_tex = None
    manager.update_display()

def main(args=None):
    rclpy.init(args=args)

    dpg.create_context()
    
    with dpg.texture_registry(show=False):
        blank_cam = np.ones((270, 480, 4), dtype=np.float32) * 0.9
        blank_cam[:, :, 3] = 0.5
        dpg.add_dynamic_texture(width=480, height=270, default_value=blank_cam.flatten(), tag="camera_texture")
        
        blank_map = np.zeros((740, 1000, 4), dtype=np.float32)
        blank_map[:, :, 3] = 0.0
        dpg.add_dynamic_texture(width=1000, height=740, default_value=blank_map.flatten(), tag="map_texture")

    ros_node = RobotGuiNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()

    global_title_font = None
    global_header_font = None
    global_button_font = None
    
    with dpg.font_registry():
        font_candidates =[
            "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
            "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",
            "/usr/share/fonts/truetype/nanum/NanumGothicBold.ttf",
            "/usr/share/fonts/truetype/ubuntu/Ubuntu-R.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "C:/Windows/Fonts/malgun.ttf",
            "C:/Windows/Fonts/malgunbd.ttf"
        ]
        
        selected_font = None
        for path in font_candidates:
            if os.path.exists(path):
                selected_font = path
                break
        
        if selected_font:
            with dpg.font(selected_font, 32) as default_font:
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Korean)
            with dpg.font(selected_font, 72) as global_title_font:
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Korean)
            with dpg.font(selected_font, 44) as global_header_font:
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Korean)
            with dpg.font(selected_font, 48) as global_button_font:
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Korean)

            dpg.bind_font(default_font)
        else:
            print("[Warning] 시스템 폰트를 찾을 수 없어 크기 조절이 무시됩니다.")

    with dpg.theme() as global_no_scroll_theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 0, 0)
            dpg.add_theme_style(dpg.mvStyleVar_ScrollbarSize, 0) 

    with dpg.theme() as glass_theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (255, 255, 255, 180))
            dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (255, 255, 255, 0))
            dpg.add_theme_color(dpg.mvThemeCol_Border, (190, 195, 200, 255))
            dpg.add_theme_style(dpg.mvStyleVar_WindowBorderSize, 4)
            dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, 32)
            dpg.add_theme_color(dpg.mvThemeCol_Text, (40, 45, 50, 255))

    with dpg.theme() as pill_button_theme:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (225, 230, 235, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (210, 215, 220, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (195, 200, 205, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (0, 0, 0, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 40)

    with dpg.theme() as theme_learning:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (210, 230, 255, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (190, 215, 255, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (170, 200, 255, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (0, 0, 0, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 40)

    with dpg.theme() as theme_driving:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (215, 240, 215, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (195, 230, 195, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (175, 220, 175, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (0, 0, 0, 255))
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 40)

    with dpg.theme() as home_button_theme:
        with dpg.theme_component(dpg.mvButton):
            dpg.add_theme_color(dpg.mvThemeCol_Button, (245, 235, 225, 255)) 
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (235, 220, 210, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (220, 205, 195, 255))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (50, 50, 55, 255)) 
            dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 40)

    with dpg.theme() as transparent_list_theme:
        with dpg.theme_component(dpg.mvListbox):
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (0, 0, 0, 0))
            dpg.add_theme_color(dpg.mvThemeCol_Text, (100, 105, 110, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarBg, (0, 0, 0, 0))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrab, (180, 190, 200, 200))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrabHovered, (150, 160, 170, 255))
            dpg.add_theme_color(dpg.mvThemeCol_ScrollbarGrabActive, (120, 130, 140, 255))
            dpg.add_theme_style(dpg.mvStyleVar_ScrollbarRounding, 12) 
            dpg.add_theme_style(dpg.mvStyleVar_ScrollbarSize, 24) 
            dpg.add_theme_color(dpg.mvThemeCol_Header, (0, 0, 0, 0))
            dpg.add_theme_color(dpg.mvThemeCol_HeaderHovered, (0, 0, 0, 0))
            dpg.add_theme_color(dpg.mvThemeCol_HeaderActive, (0, 0, 0, 0))

    with dpg.window(tag="base_window", width=1720, height=1500, pos=[0,0], 
                    no_title_bar=True, no_move=True, no_resize=True, no_scrollbar=True, no_bring_to_front_on_focus=True):
        
        dpg.bind_item_theme("base_window", global_no_scroll_theme)
        
        with dpg.drawlist(width=1720, height=1500):
            dpg.draw_rectangle([0,0],[1720,1500], color=(240, 244, 248, 255), fill=(240, 244, 248, 255))

        t_main = dpg.add_text("ROBOT CONTROL CENTER", pos=[60, 16], color=(40, 50, 60))
        dpg.add_text("SIGNAL", pos=[1200, 30], color=(120, 120, 120))
        dpg.add_text("BATTERY", pos=[1340, 30], color=(120, 120, 120))
        dpg.add_text("TIME", pos=[1500, 30], color=(120, 120, 120)) 

        t_sig = dpg.add_text("100%", tag="wifi_val", pos=[1200, 70], color=(0, 0, 0))
        t_bat = dpg.add_text("100%", tag="battery_val", pos=[1340, 70], color = (0, 150, 50, 255))
        t_time = dpg.add_text("00:00:00", tag="live_clock", pos=[1500, 70], color=(0, 0, 0))

        if global_title_font: dpg.bind_item_font(t_main, global_title_font)
        if global_header_font:
            dpg.bind_item_font(t_sig, global_header_font)
            dpg.bind_item_font(t_bat, global_header_font)
            dpg.bind_item_font(t_time, global_header_font)

    map_win = dpg.add_window(no_title_bar=True, no_resize=True, no_move=True, no_scrollbar=True, width=1000, height=740, pos=[60, 140])
    dpg.bind_item_theme(map_win, glass_theme)
    with dpg.group(parent=map_win):
        dpg.add_image("map_texture", width=960, height=700, tag="map_image_item") 

    with dpg.handler_registry():
        dpg.add_mouse_down_handler(button=dpg.mvMouseButton_Left, callback=map_mouse_down_cb, user_data=ros_node)
        dpg.add_mouse_drag_handler(button=dpg.mvMouseButton_Left, callback=map_mouse_drag_cb, user_data=ros_node)
        dpg.add_mouse_release_handler(button=dpg.mvMouseButton_Left, callback=map_mouse_release_cb, user_data=ros_node)

    overlay_win = dpg.add_window(no_title_bar=True, no_resize=True, no_move=True, no_scrollbar=True, 
                                 width=1000, height=110, pos=[60, 170], no_background=True)
    with dpg.group(parent=overlay_win, horizontal=True):
        dpg.add_spacer(width=320)
        t_ov = dpg.add_text("", tag="overlay_status_text")
        if global_title_font: dpg.bind_item_font(t_ov, global_title_font)

    log_win = dpg.add_window(no_title_bar=True, no_resize=True, no_move=True, no_scrollbar=True, width=1000, height=240, pos=[60, 910])
    dpg.bind_item_theme(log_win, glass_theme)
    with dpg.group(parent=log_win):
        dpg.add_spacer(height=10)
        with dpg.group(horizontal=True):
            dpg.add_spacer(width=14)
            t_l = dpg.add_text("SYSTEM LOGS", color=(0, 122, 255, 255))
            if global_header_font: dpg.bind_item_font(t_l, global_header_font)
        
        dpg.add_spacer(height=10)
        lb = dpg.add_listbox(items=["[SYSTEM] Awaiting input..."], tag="log_listbox", width=960, num_items=5)
        dpg.bind_item_theme(lb, transparent_list_theme)

    dash_win = dpg.add_window(no_title_bar=True, no_resize=True, no_move=True, no_scrollbar=True, width=560, height=1020, pos=[1100, 140])
    dpg.bind_item_theme(dash_win, glass_theme)
    
    with dpg.group(parent=dash_win):
        dpg.add_spacer(height=14) 
        
        with dpg.group(horizontal=True):
            dpg.add_spacer(width=14)
            with dpg.group():
                dpg.add_text("CAM_01 LIVE", color=(0, 0, 0, 255))
                dpg.add_spacer(height=6)
                dpg.add_image("camera_texture", width=480, height=270)
                
        dpg.add_spacer(height=50) 
        
        with dpg.group(horizontal=True):
            dpg.add_spacer(width=14)
            with dpg.group():
                with dpg.group(horizontal=True):
                    dpg.add_text("STATUS :")
                    dpg.add_text("MODE: NAVIGATING", tag="status_val", color=(0, 150, 50, 255))
                
                dpg.add_spacer(height=16)
                with dpg.group(horizontal=True):
                    dpg.add_text("POS    :") 
                    dpg.add_text("X: 0.0, Y: 0.0", tag="pose_val")

                dpg.add_spacer(height=16)
                with dpg.group(horizontal=True):
                    dpg.add_text("THETA  :")
                    dpg.add_text("0.0°", tag="theta_val")

                dpg.add_spacer(height=16)
                with dpg.group(horizontal=True):
                    dpg.add_text("TARGET :")
                    # [해결됨] 대시보드 상태창 텍스트 업데이트를 위한 태그 추가
                    dpg.add_text("STANDBY", tag="target_val", color=(0, 122, 255, 255))
        
        dpg.add_spacer(height=50) 
        
        with dpg.group(horizontal=True):
            dpg.add_spacer(width=20)
            with dpg.group():
                t_c = dpg.add_text("COMMAND", color=(0, 122, 255, 255))
                if global_header_font: dpg.bind_item_font(t_c, global_header_font)
                
                dpg.add_spacer(height=20)
                
                create_command_buttons(
                    ros_node,
                    theme_driving,
                    theme_learning,
                    global_button_font,
                )

    nav_win = dpg.add_window(
        tag="navigation_window",
        no_title_bar=True,
        no_resize=True,
        no_move=True,
        no_scrollbar=True,
        no_background=True,
        width=1600,
        height=260,
        pos=[60, 1190],
    )
    with dpg.group(parent=nav_win):
        t_n = dpg.add_text("GUIDED NAVIGATION", pos=[0, 0], color=(0, 122, 255, 255))
        if global_header_font:
            dpg.bind_item_font(t_n, global_header_font)

        create_navigation_buttons(
            ros_node,
            pill_button_theme,
            home_button_theme,
            global_button_font,
            parent=nav_win,
        )

    dpg.create_viewport(title='Robot Control Center', width=1720, height=1500, clear_color=[0, 0, 0, 0])
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("base_window", True)

    last_clock_update = 0
    while dpg.is_dearpygui_running():
        current_time = time.time()
        
        if current_time - last_clock_update >= 1.0:
            clock_str = datetime.now().strftime("%H:%M:%S")
            if dpg.does_item_exist("live_clock"):
                dpg.set_value("live_clock", clock_str)
            last_clock_update = current_time
            
        dpg.render_dearpygui_frame()

    dpg.destroy_context()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == '__main__':
    main()
