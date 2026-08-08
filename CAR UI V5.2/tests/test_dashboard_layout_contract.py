from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _text(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_home_uses_four_mode_control_hub_and_vehicle_art():
    page = _text("qml/pages/HomePage.qml")

    assert '"../../assets/vehicle.png"' not in page
    assert 'objectName: "homeControlHub"' in page
    assert 'objectName: "homeInformationPanel"' in page
    assert 'objectName: "homeControlPanel"' in page
    assert 'objectName: "systemBrandFrame"' in page
    assert 'I18n.t("深圳文思汇通有限公司AMR操作系统")' in page
    assert 'objectName: "primaryControlRow"' in page
    assert 'objectName: "secondaryControlRow"' in page
    assert 'objectName: "navigationControlCard"' in page
    assert 'objectName: "voiceControlCard"' in page
    assert 'objectName: "gamepadControlCard"' in page
    assert 'objectName: "followControlCard"' in page
    assert 'I18n.t("控制模式")' in page
    assert 'I18n.t("选择一种方式控制车辆")' not in page
    assert 'horizontalAlignment: Text.AlignHCenter' in page
    assert "height: (parent.height - root.gap) / 2" in page
    assert 'I18n.t("车辆状态")' in page
    assert 'I18n.t("上位机状态")' in page
    assert 'I18n.t("下位机状态")' in page
    assert 'I18n.t("导航控制")' in page
    assert 'I18n.t("语音控制")' in page
    assert 'I18n.t("手柄控制")' in page
    assert 'I18n.t("视觉控制")' in page
    assert "camera-lineart.png" in page
    assert "gamepad-lineart.png" in page
    assert "camera-lineart-dark.png" in page
    assert "gamepad-lineart-dark.png" in page
    assert 'window.pushPage("RobotStatusPage.qml")' in page
    assert 'window.pushPage("SettingsPage.qml")' in page


def test_gamepad_handoff_is_local_one_way_placeholder():
    page = _text("qml/pages/HomePage.qml")
    backend = _text("backend/ui_backend.py")
    base = _text("robot_api/base.py")

    assert 'property bool gamepadControlActive: false' in page
    assert 'objectName: "releaseControlToGamepadButton"' in page
    assert "backend.releaseControlToGamepad()" in page
    assert "root.gamepadControlActive = true" in page
    assert "function hostTakesControl()" in page
    assert "def releaseControlToGamepad" in backend
    assert "def release_control_to_gamepad" in base


def test_home_preserves_map_click_and_recent_three_contract():
    page = _text("qml/pages/HomePage.qml")
    backend = _text("backend/ui_backend.py")

    assert "interactiveGoalSelection: true" in page
    assert "backend.selectMapGoal(worldX, worldY)" in page
    assert "backend.selectPoint(modelData.id)" in page
    assert "def recentPoints" in backend
    assert "[:3]" in backend


def test_fullscreen_map_preserves_goal_selection_and_single_navigation_button():
    page = _text("qml/pages/RvizFullscreenPage.qml")
    component = _text("qml/components/RvizPlaceholder.qml")

    assert "interactiveGoalSelection: true" in page
    assert "backend.selectMapGoal(worldX, worldY)" in page
    assert "navigationToggleVisible: true" in page
    assert "onResetRequested: backend.clearNavigationSelection()" in page
    assert 'objectName: "fullscreenNavigationButton"' in component
    assert 'root.navigationActive ? "结束导航" : "开始导航"' in component
    assert "root.navigationActive ? Theme.danger : Theme.primary" in component
    assert "backend.startSelectedNavigation()" in page
    assert "backend.cancelNavigation()" in page
    assert "PinchHandler" in component
    assert "onWheel:" in component
    assert "DragHandler" in component
    assert "root.panX = startingPanX + activeTranslation.x" in component
    assert 'I18n.t("车头朝上并自动居中")' in component
    assert "updateHeadingTransform" in component


def test_navigation_manager_is_split_list_map_and_route_layout():
    page = _text("qml/pages/PointManagerPage.qml")

    assert "property int pageSize: 5" in page
    assert "RvizPlaceholder" in page
    assert 'I18n.t("多路径点导航")' in page
    assert "backend.addRoutePoint(modelData.id)" in page
    assert "backend.removeRoutePoint(modelData.id)" in page
    assert "backend.clearRoute()" in page
    assert "backend.startRoute(orderedSwitch.checked)" in page


def test_follow_distance_is_editable_on_home_and_embedded_control():
    home = _text("qml/pages/HomePage.qml")
    follow = _text("qml/pages/FollowPage.qml")
    settings = _text("qml/pages/SettingsPage.qml")

    assert 'objectName: "homeFollowDistanceSlider"' in home
    assert 'objectName: "homeStartFollowingButton"' in home
    assert "backend.startFollowing(root.followSelectedActor)" in home
    assert 'objectName: "followControlDistanceSlider"' in follow
    assert '"follow_distance", Number(value.toFixed(1))' in follow
    assert 'setParameter("follow_distance"' not in settings
    assert 'I18n.t("换一个人")' in follow
    assert 'I18n.t("开始跟随")' in follow
    assert 'I18n.t("结束跟随")' in follow


def test_home_follow_control_replaces_only_right_panel_and_is_destroyed_on_exit():
    home = _text("qml/pages/HomePage.qml")
    follow = _text("qml/pages/FollowPage.qml")

    assert 'property bool followControlExpanded: false' in home
    assert 'visible: !root.controlDetailExpanded' in home
    assert 'objectName: "embeddedFollowLoader"' in home
    assert 'active: root.followControlExpanded' in home
    assert 'FollowPage {' in home and 'embedded: true' in home
    assert 'root.followControlExpanded = true' in home
    assert 'window.pushPage("FollowPage.qml")' not in home
    assert 'if (followControlExpanded)' in home
    assert 'property bool embedded: false' in follow
    assert 'objectName: "exitEmbeddedFollowButton"' in follow
    assert 'onClicked: root.exitRequested()' in follow
    assert 'enabled: backend.snapshot.visual_follow_enabled ?? false' in follow


def test_settings_volume_uses_unbound_touch_slider():
    settings = _text("qml/pages/SettingsPage.qml")

    assert 'I18n.t("音量设置")' in settings
    assert 'objectName: "systemVolumeSlider"' in settings
    assert "AppSlider {" in settings
    assert "value: backend.settings.volume" not in settings
    assert "backend.refreshSystemVolume()" in settings


def test_settings_keeps_all_theme_choices_and_light_dark_toggle():
    settings = _text("qml/pages/SettingsPage.qml")

    assert 'I18n.t("外观设置")' in settings
    for name in ("工业蓝", "石墨青", "深空紫", "钛金橙"):
        assert f'I18n.t("{name}")' in settings
    assert 'options: [I18n.t("亮色"), I18n.t("暗色")]' in settings
    assert "window.setColorScheme(index)" in settings
    assert "window.setDarkMode(index === 1)" in settings
    assert 'objectName: "performanceModeControl"' in settings
    assert 'I18n.t("低性能模式")' in settings
    assert 'I18n.t("普通模式")' in settings
    assert 'I18n.t("流畅模式")' in settings
    assert 'I18n.t("流畅模式可能占用更多算力和内存")' in settings
    assert "window.setPerformanceMode(index)" in settings
    assert 'objectName: "fontSizeModeControl"' in settings
    assert 'options: [I18n.t("小"), I18n.t("标准"), I18n.t("大")]' in settings
    assert "window.setFontSizeMode(index)" in settings
    assert 'objectName: "borderModeControl"' in settings
    assert 'options: [I18n.t("细"), I18n.t("中"), I18n.t("粗")]' in settings
    assert "window.setBorderMode(index)" in settings


def test_branding_vehicle_info_and_read_only_developer_views_are_integrated():
    home = _text("qml/pages/HomePage.qml")
    settings = _text("qml/pages/SettingsPage.qml")
    my_robot = _text("qml/pages/MyRobotPanel.qml")
    developer = _text("qml/pages/DeveloperPanel.qml")
    password = _text("qml/dialogs/DeveloperPasswordDialog.qml")
    header = _text("qml/components/PageHeader.qml")

    assert "wensihuitong-light.png" in home
    assert "hongxindeli-light.png" in home
    assert "currentVoiceIcon" in home and "currentVoiceLabel" in home
    assert 'I18n.t("我的小车")' in settings
    assert 'I18n.t("开发者模式")' in settings
    assert "NVIDIA Jetson Orin Nano" in _text("backend/ui_backend.py")
    assert "assets/vehicle.png" in my_robot
    assert "developerUiLog" in developer and "developerRosLog" in developer
    assert 'objectName: "rosDomainIdField"' in developer
    assert "backend.setRosDomainId" in developer
    assert "def setRosDomainId" in _text("backend/ui_backend.py")
    assert 'passwordField.text === "123"' in password
    assert 'text: "×"' in header


def test_ros_domain_and_single_instance_are_applied_before_ros_startup():
    main = _text("main.py")
    launcher = _text("run.sh")

    assert "configure_ros_domain_id(data_dir)" in main
    assert main.index("configure_ros_domain_id(data_dir)") < main.index(
        "create_robot_api(data_dir)"
    )
    assert "robot-touch-ui-single-instance.lock" in main
    assert "is_project_ui_pid" in launcher
    assert 'candidate_cwd" == "$PWD' in launcher
    assert 'kill -TERM "${!OLD_UI_PIDS[@]}"' in launcher


def test_startup_branding_and_controller_tutorial_are_integrated():
    main = _text("qml/Main.qml")
    splash = _text("qml/components/StartupSplash.qml")
    home = _text("qml/pages/HomePage.qml")
    selector = _text("qml/pages/MapSelectorPage.qml")
    mapping_dialog = _text("qml/dialogs/MappingConfirmDialog.qml")
    tutorial = _text("qml/pages/GamepadTutorialPage.qml")

    assert "Settings {" in main
    assert "property bool startupVisible: true" in main
    assert "1000 + Math.floor(Math.random() * 2001)" in splash
    assert 'text: "&"' in splash
    assert 'text: "AMR 操作系统"' in splash
    assert "wensihuitong-dark.png" in splash
    assert "hongxindeli-dark.png" in splash
    assert 'I18n.t("手柄教程")' in home
    assert 'I18n.t("创建新地图")' in selector
    assert 'I18n.t("建议观看手柄教程，使用手柄切换至建图档位")' in mapping_dialog
    assert 'objectName: "gamepadTutorialPage"' in tutorial
    assert "GamepadFocusView" in tutorial
    assert "VehicleDemo" in tutorial

    model = _text("qml/components/GamepadTutorialModel.qml")
    focus = _text("qml/components/GamepadFocusView.qml")
    demo = _text("qml/components/VehicleDemo.qml")
    package = _text("robot_ui.pyproject")
    assert model.count("id: \"" ) == 7
    assert "mapping_one" not in model and "mapping_two" not in model
    assert "right_stick" not in model
    assert "manualNavigation" in tutorial
    assert "interval: 1000" in tutorial
    assert "autoAdvanceTimer.restart()" in tutorial
    assert "focusSpec" in focus
    for overlay in ("dpad-up", "dpad-down", "dpad-left", "dpad-right",
                    "estop-button", "gear-button", "left-stick", "right-stick"):
        assert overlay in focus
        assert f"assets/tutorial/highlights/{overlay}.png" in package
    assert "Math.min(5, Math.floor(phase / 2))" in demo
    assert "model: [\"#32B968\", \"#358AF3\", \"#E8B72E\", \"#E65757\"]" in demo


def test_home_create_map_button_sits_left_of_map_management():
    home = _text("qml/pages/HomePage.qml")
    placeholder = _text("qml/components/RvizPlaceholder.qml")

    assert "createMapVisible: true" in home
    assert "onCreateMapRequested: homeMappingConfirmDialog.open()" in home
    assert "MappingConfirmDialog {" in home
    assert "backend.startMapping()" in home
    assert 'window.pushPage("MappingFullscreenPage.qml")' in home
    assert 'I18n.t("创建新地图")' in placeholder
    assert "property bool createMapVisible: false" in placeholder
    assert "signal createMapRequested()" in placeholder
    assert placeholder.index('I18n.t("创建新地图")') < placeholder.index('I18n.t("地图管理")')


def test_voice_control_opens_as_destroyable_right_half_detail():
    home = _text("qml/pages/HomePage.qml")
    voice = _text("qml/pages/VoiceControlPage.qml")

    assert "property bool voiceControlExpanded: false" in home
    assert 'objectName: "embeddedVoiceLoader"' in home
    assert "active: root.voiceControlExpanded" in home
    assert "VoiceControlPage {" in home and "embedded: true" in home
    assert "root.voiceControlExpanded = true" in home
    assert 'property bool embedded: false' in voice
    assert 'closeObjectName: root.embedded ? "exitEmbeddedVoiceButton"' in voice
    assert "showCloseButton: true" in voice


def test_vehicle_art_is_in_both_packaging_paths():
    arm64 = _text("build_arm64_deb.sh")
    amd64 = _text("build_deb.sh")
    deploy = _text("robot_ui.pyproject")

    assert '"$PROJECT_ROOT/assets"' in arm64
    assert '"$PROJECT_ROOT/assets"' in amd64
    assert '"assets/vehicle.png"' in deploy


def test_map_selector_uses_three_item_lightweight_carousel_and_existing_components():
    home = _text("qml/pages/HomePage.qml")
    page = _text("qml/pages/MapSelectorPage.qml")
    carousel = _text("qml/components/MapCarousel.qml")
    card = _text("qml/components/MapCard.qml")

    assert 'window.pushPage("MapSelectorPage.qml")' in home
    assert "DangerButton" in page
    assert "SecondaryButton" in page
    assert "PrimaryButton" in page
    assert "ConfirmDialog" in page
    assert "RenameMapDialog" in page
    assert "pathItemCount: Math.min(3" in carousel
    assert "cacheItemCount: 0" in carousel
    assert "highlightMoveDuration: Performance.mediumDuration" in carousel
    assert "transitionLock" in carousel
    assert "circularDistance" in carousel
    assert "% maps.length" in carousel
    assert "sourceSize.width" in card
    assert "cache_pgm_url" in card
    assert "ShaderEffect" not in carousel


def test_status_bar_does_not_repeat_joint_company_credit():
    status_bar = _text("qml/components/StatusBar.qml")

    assert "深圳文思汇通科技有限公司" not in status_bar
    assert "深圳市洪昕德立科技有限公司" not in status_bar
    assert "companyLine" not in status_bar
