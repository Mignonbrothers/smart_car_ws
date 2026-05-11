# button_manager.py

import dearpygui.dearpygui as dpg

NAV_TARGET_LABELS = {
    "toilet": "화장실",
    "home": "홈",
    "stationery": "문구류",
    "sunscreen": "선크림",
    "wet_tissue": "물티슈",
}

NAV_TARGET_COORDS = {
    "toilet": (1.9163875579833984, -1.4372011423110962),
    "home": (-0.2187747061252594, 0.0520884245634079),
    "stationery": (2.522963523864746, -2.9560720920562744),
    "sunscreen": (-0.3035605251789093, -0.7663712501525879),
    "wet_tissue": (0.7415836453437805, -2.8868584632873535),
}

ACTIVE_NAV_THEME = None

def get_active_nav_theme():
    global ACTIVE_NAV_THEME
    if ACTIVE_NAV_THEME is None:
        with dpg.theme() as ACTIVE_NAV_THEME:
            with dpg.theme_component(dpg.mvButton):
                dpg.add_theme_color(dpg.mvThemeCol_Button, (0, 122, 255, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (40, 150, 255, 255))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (0, 100, 220, 255))
                dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255, 255))
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 40)
    return ACTIVE_NAV_THEME


def update_overlay_text(text, color):
    if dpg.does_item_exist("overlay_status_text"):
        dpg.set_value("overlay_status_text", text)
        dpg.configure_item("overlay_status_text", color=color)


def btn_cb_auto_driving(sender, app_data, user_data):
    person_mode = not getattr(user_data, "person_following_mode", False)
    user_data.person_following_mode = person_mode

    if person_mode:
        user_data.send_command("CMD_SET_MODE_PERSON_FOLLOWING")
        update_overlay_text("사람추종 모드", (0, 180, 50, 255))
        if dpg.does_item_exist("status_val"):
            dpg.set_value("status_val", "MODE: PERSON FOLLOWING")
        if dpg.does_item_exist("target_val"):
            dpg.set_value("target_val", "PERSON")
    else:
        user_data.send_command("CMD_SET_MODE_NAVIGATION")
        update_overlay_text("목적지 이동 모드", (0, 122, 255, 255))
        if dpg.does_item_exist("status_val"):
            dpg.set_value("status_val", "MODE: NAVIGATION")
        if dpg.does_item_exist("target_val"):
            dpg.set_value("target_val", "STANDBY")

    if hasattr(user_data, "add_log"):
        mode_text = "사람추종" if person_mode else "목적지 이동"
        user_data.add_log(f"[MODE] {mode_text} 모드 전환")


def btn_cb_pose_estimate(sender, app_data, user_data):
    ros_node = user_data
    if getattr(ros_node, "is_pose_estimating", False):
        ros_node.is_pose_estimating = False
        ros_node.pose_start_pos = None
        if hasattr(ros_node, "map_manager"):
            ros_node.map_manager.drag_start_tex = None
            ros_node.map_manager.drag_current_tex = None
            ros_node.map_manager.update_display()
        update_overlay_text("위치추정 취소됨", (255, 120, 0, 255))
        if hasattr(ros_node, "add_log"):
            ros_node.add_log("[POSE] 위치추정 모드 취소")
    else:
        ros_node.is_pose_estimating = True
        ros_node.pose_start_pos = None
        if hasattr(ros_node, "map_manager"):
            ros_node.map_manager.drag_start_tex = None
            ros_node.map_manager.drag_current_tex = None
        update_overlay_text("위치추정 (드래그 지정)", (255, 120, 0, 255))
        if hasattr(ros_node, "add_log"):
            ros_node.add_log("[POSE] 맵에서 시작 위치와 방향을 드래그하세요")


def btn_cb_nav_target(sender, app_data, user_data):
    node, target = user_data

    if getattr(node, "person_following_mode", False):
        update_overlay_text("사람추종 모드에서는 목적지 이동 비활성", (255, 120, 0, 255))
        if hasattr(node, "add_log"):
            node.add_log("[NAV] 사람추종 모드에서는 목적지 이동 명령을 무시합니다")
        return
    
    # [해결 1] 클릭된 버튼만 파란색(Active) 테마로 즉각 변경
    if hasattr(node, 'nav_button_registry'):
        for btn, orig_theme in node.nav_button_registry:
            if dpg.does_item_exist(btn):
                dpg.bind_item_theme(btn, orig_theme)
        
        if sender and dpg.does_item_exist(sender):
            dpg.bind_item_theme(sender, get_active_nav_theme())

    node.send_command(f"CMD_NAV_TO_{target}")
    target_kr = NAV_TARGET_LABELS.get(target, target)
    
    if target == "home":
        msg = "홈으로 복귀 시작"
    elif target == "toilet":
        msg = "화장실로 안내 시작"
    else:
        msg = f"{target_kr} 코너로 안내 시작"
        
    update_overlay_text(msg, (0, 180, 50, 255))
    if hasattr(node, "add_log"):
        coords = NAV_TARGET_COORDS.get(target)
        if coords:
            node.add_log(f"[NAV] {target_kr} 안내 시작 -> X:{coords[0]:.2f}, Y:{coords[1]:.2f}")
        else:
            node.add_log(f"[NAV] {target_kr} 안내 시작")
    
    # [해결 2] 대시보드의 STANDBY 텍스트를 목적지로 즉각 갱신
    if dpg.does_item_exist("target_val"):
        dpg.set_value("target_val", target_kr)
    
    # [해결 3] set_nav_target 함수가 맵 매니저에 없더라도 프로그램이 뻗지 않도록 방어 코드 적용
    if hasattr(node, "map_manager") and node.map_manager:
        coords = NAV_TARGET_COORDS.get(target)
        if coords:
            if hasattr(node.map_manager, "set_nav_target"):
                node.map_manager.set_nav_target(coords[0], coords[1], target_kr)
                if hasattr(node, "add_log"):
                    status = getattr(node.map_manager, "last_path_status", "")
                    if status.startswith("PATH_READY"):
                        node.add_log(f"[PATH] {target_kr} 우회 경로 계산 완료")
                    elif status == "PATH_FALLBACK_STRAIGHT":
                        node.add_log(f"[PATH] {target_kr} 경로 계산 실패, 직선 점선 표시")
                    elif status == "PATH_UNAVAILABLE":
                        node.add_log(f"[PATH] {target_kr} 경로 표시 불가")
            else:
                node.add_log("[경고] map_gui.py에 set_nav_target 함수가 없습니다.")


def create_navigation_buttons(
    ros_node,
    pill_button_theme,
    home_button_theme,
    button_font=None,
    parent=None,
):
    # [해결 4] 콜백 전에 미리 활성화 테마를 로드하여 렌더링 딜레이 방지
    get_active_nav_theme()
    
    buttons = [
        dpg.add_button(
            label="화장실",
            width=500,
            height=80,
            pos=[0, 80],
            parent=parent,
            callback=btn_cb_nav_target,
            user_data=(ros_node, "toilet"),
        ),
        dpg.add_button(
            label="HOME",
            width=520,
            height=180,
            pos=[540, 80],
            parent=parent,
            callback=btn_cb_nav_target,
            user_data=(ros_node, "home"),
        ),
        dpg.add_button(
            label="문구류",
            width=500,
            height=80,
            pos=[1100, 80],
            parent=parent,
            callback=btn_cb_nav_target,
            user_data=(ros_node, "stationery"),
        ),
        dpg.add_button(
            label="선크림",
            width=500,
            height=80,
            pos=[0, 180],
            parent=parent,
            callback=btn_cb_nav_target,
            user_data=(ros_node, "sunscreen"),
        ),
        dpg.add_button(
            label="물티슈",
            width=500,
            height=80,
            pos=[1100, 180],
            parent=parent,
            callback=btn_cb_nav_target,
            user_data=(ros_node, "wet_tissue"),
        ),
    ]

    ros_node.nav_button_registry = [
        (buttons[0], pill_button_theme),
        (buttons[1], home_button_theme),
        (buttons[2], pill_button_theme),
        (buttons[3], pill_button_theme),
        (buttons[4], pill_button_theme),
    ]

    for button in [buttons[0], buttons[2], buttons[3], buttons[4]]:
        dpg.bind_item_theme(button, pill_button_theme)
        if button_font:
            dpg.bind_item_font(button, button_font)

    dpg.bind_item_theme(buttons[1], home_button_theme)
    if button_font:
        dpg.bind_item_font(buttons[1], button_font)

    return buttons


def create_command_buttons(ros_node, theme_driving, theme_learning, button_font=None):
    auto_button = dpg.add_button(
        label="AUTO DRIVING",
        width=440,
        height=112,
        callback=btn_cb_auto_driving,
        user_data=ros_node,
    )
    dpg.add_spacer(height=30)
    pose_button = dpg.add_button(
        label="POSE ESTIMATE",
        width=440,
        height=112,
        callback=btn_cb_pose_estimate,
        user_data=ros_node,
    )

    dpg.bind_item_theme(auto_button, theme_driving)
    dpg.bind_item_theme(pose_button, theme_learning)
    if button_font:
        dpg.bind_item_font(auto_button, button_font)
        dpg.bind_item_font(pose_button, button_font)

    return auto_button, pose_button
