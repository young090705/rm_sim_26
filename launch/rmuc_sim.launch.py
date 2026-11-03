"""Launch the robot simulation using map and sensor settings from YAML."""

import os
from pathlib import Path
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def find_rgl_plugin_paths(root_hint):
    """Accept a workspace, RGL source directory, or RGL install directory."""
    root = Path(root_hint).expanduser().resolve()
    for prefix in (root / "external/RGLGazeboPlugin/install", root / "install", root):
        system = prefix / "RGLServerPlugin"
        if all((system / name).is_file() for name in (
            "libRGLServerPluginManager.so",
            "libRGLServerPluginInstance.so",
            "libRobotecGPULidar.so",
        )):
            return str(system), str(prefix / "RGLVisualize")
    return "", "" #返回两个空字符串


def _join_env_path(variable_name, *extra_paths):
    paths = [*extra_paths, os.environ.get(variable_name, "")]
    return ":".join(path for path in paths if path)


def _launch_setup(context):
    def _has_nvidia_ml():
        candidates = [
            "/usr/lib/x86_64-linux-gnu/libnvidia-ml.so.1",
            "/usr/lib64/libnvidia-ml.so.1",
            "/usr/lib/libnvidia-ml.so.1",
        ]
        return any(os.path.exists(path) for path in candidates)

    def _read_text(path):
        with open(path, "r", encoding="utf-8") as file_obj:
            return file_obj.read()

    def _write_temp_file(content, suffix, prefix):
        with tempfile.NamedTemporaryFile(
            "w",
            delete=False,
            encoding="utf-8",
            suffix=suffix,
            prefix=prefix,
        ) as temp_file:
            temp_file.write(content)
            return temp_file.name

    def _load_sim_config(config_path):
        default_config = {
            "map_year": 2026,
            "maps": {},
            "robot_model": "sentry",
            "publish_tf": False,
            "lidar_mode": "auto",
            "rgl_plugin_root": "",
            "livox_mid360": {
                "horizontal_samples": 1875,
                "horizontal_min_angle": 0.0,
                "horizontal_max_angle": 6.2831852,
                "vertical_samples": 32,
                "vertical_min_angle": -0.12217304764,
                "vertical_max_angle": 0.90757121104,
                "range_min": 0.1,
                "range_max": 40.0,
                "update_rate": 10,
                "noise_type": "gaussian",
                "noise_mean": 0.0,
                "noise_stddev": 0.01,
            },
        }
        if os.path.exists(config_path):
            with open(config_path, "r", encoding="utf-8") as file_obj:
                user_config = yaml.safe_load(file_obj) or {}
            for key, default_value in default_config.items():
                if key not in user_config:
                    continue
                if isinstance(default_value, dict):
                    default_value.update(user_config[key])
                else:
                    default_config[key] = user_config[key]
        return default_config

    def _determine_use_rgl(lidar_mode, has_nvidia):
        if lidar_mode == "rgl":
            if not has_nvidia:
                raise RuntimeError("RGL模式需要可用的NVIDIA驱动，未检测到libnvidia-ml.so.1")
            return True
        if lidar_mode == "gpu_lidar":
            return False
        return has_nvidia

    def _write_fallback_world(src_path):
        content = _read_text(src_path)
        plugin_start = content.find(
            "<plugin name='rgl::RGLServerPluginManager' filename='RGLServerPluginManager'>"
        )
        if plugin_start != -1:
            plugin_end = content.find("</plugin>", plugin_start)
            if plugin_end != -1:
                content = (
                    content[:plugin_start]
                    + content[plugin_end + len("</plugin>") :]
                )
        return _write_temp_file(content, ".world", "rm_sim_26_world_no_rgl_")

    def _build_gpu_lidar_sensor(lidar_config):
        return (
            '<sensor name="livox_lidar" type="gpu_lidar">\n'
            "  <pose>0 0 0 0 0 0</pose>\n"
            "  <always_on>true</always_on>\n"
            "  <visualize>true</visualize>\n"
            f'  <update_rate>{lidar_config["update_rate"]}</update_rate>\n'
            "  <topic>gz_lidar_points</topic>\n"
            "  <gz_frame_id>livox_lidar</gz_frame_id>\n"
            "  <ray>\n"
            "    <scan>\n"
            "      <horizontal>\n"
            f'        <samples>{lidar_config["horizontal_samples"]}</samples>\n'
            "        <resolution>1.0</resolution>\n"
            f'        <min_angle>{lidar_config["horizontal_min_angle"]}</min_angle>\n'
            f'        <max_angle>{lidar_config["horizontal_max_angle"]}</max_angle>\n'
            "      </horizontal>\n"
            "      <vertical>\n"
            f'        <samples>{lidar_config["vertical_samples"]}</samples>\n'
            f'        <min_angle>{lidar_config["vertical_min_angle"]}</min_angle>\n'
            f'        <max_angle>{lidar_config["vertical_max_angle"]}</max_angle>\n'
            "      </vertical>\n"
            "    </scan>\n"
            "    <range>\n"
            f'      <min>{lidar_config["range_min"]}</min>\n'
            f'      <max>{lidar_config["range_max"]}</max>\n'
            "    </range>\n"
            "    <noise>\n"
            f'      <type>{lidar_config["noise_type"]}</type>\n'
            f'      <mean>{lidar_config["noise_mean"]}</mean>\n'
            f'      <stddev>{lidar_config["noise_stddev"]}</stddev>\n'
            "    </noise>\n"
            "  </ray>\n"
            "</sensor>\n"
        )

    def _write_runtime_model(src_path, lidar_config, use_rgl):
        content = _read_text(src_path)
        if use_rgl:
            return _write_temp_file(content, ".sdf", "rm_sim_26_model_rgl_")

        sensor_start = content.find('<sensor name="RGLLidar" type="custom">')
        if sensor_start != -1:
            sensor_end = content.find("</sensor>", sensor_start)
            if sensor_end != -1:
                content = (
                    content[:sensor_start]
                    + _build_gpu_lidar_sensor(lidar_config)
                    + content[sensor_end + len("</sensor>") :]
                )
        return _write_temp_file(content, ".sdf", "rm_sim_26_model_gpu_lidar_")

    package_name = "rm_sim_26"
    pkg_share_path = get_package_share_directory(package_name)
    pattern_file_path = os.path.join(
        pkg_share_path, "plugin", "LivoxMid360.mat3x4f"
    )

    config_path = LaunchConfiguration("config_file").perform(context)
    sim_config = _load_sim_config(config_path)
    publish_tf = sim_config["publish_tf"]
    if not isinstance(publish_tf, bool):
        raise ValueError("publish_tf 必须是 YAML 布尔值 true 或 false（不要加引号）")
    print(f"[INFO] 机器人内部 TF 发布: {publish_tf}")
    map_year = str(sim_config["map_year"])
    maps = sim_config["maps"]
    if map_year not in maps:
        available = ", ".join(maps) or "无"
        raise ValueError(f"未知 map_year '{map_year}'，YAML 中可选地图: {available}")
    map_config = maps[map_year]
    world_file = os.path.join(pkg_share_path, "worlds", map_config["world"])
    if not os.path.isfile(world_file):
        raise FileNotFoundError(f"地图文件不存在: {world_file}")
    spawn_position = {
        axis: LaunchConfiguration(f"spawn_{axis}").perform(context)
        or str(map_config["spawn"][axis])
        for axis in ("x", "y", "z")
    }
    print(f"[INFO] 场地: RMUC {map_year} ({world_file})")
    print(f"[INFO] 出生点: {spawn_position}")
    lidar_override = LaunchConfiguration("lidar_mode").perform(context)
    if lidar_override:
        if lidar_override not in ("auto", "rgl", "gpu_lidar"):
            raise ValueError("lidar_mode must be auto, rgl or gpu_lidar")
        sim_config["lidar_mode"] = lidar_override
    if sim_config["lidar_mode"] not in ("auto", "rgl", "gpu_lidar"):
        raise ValueError("lidar_mode must be auto, rgl or gpu_lidar")

    robot_model = sim_config.get("robot_model", "sentry")
    if robot_model not in ("sentry", "mecanum_car"):
        print(f"[WARN] 未知robot_model '{robot_model}'，回退为sentry")
        robot_model = "sentry"
    robot_name = "sentry_bot" if robot_model == "sentry" else "mecanum_bot"
    print(f"[INFO] 机器人模型: {robot_model} (spawn name: {robot_name})")

    robot_model_path = os.path.join(
        pkg_share_path, "models", robot_model, "model.sdf"
    )

    has_nvidia = _has_nvidia_ml()
    use_rgl = _determine_use_rgl(sim_config["lidar_mode"], has_nvidia)

    rgl_system_plugin, rgl_gui_plugin = "", ""
    if use_rgl:
        root_hint = os.environ.get("RGL_GZ_PLUGIN_ROOT") or sim_config.get("rgl_plugin_root")
        roots = [root_hint] if root_hint else Path(__file__).resolve().parents
        for root in roots:
            rgl_system_plugin, rgl_gui_plugin = find_rgl_plugin_paths(root)
            if rgl_system_plugin:
                break

    if use_rgl and not rgl_system_plugin:
        message = "未找到完整RGL插件安装；请编译 external/RGLGazeboPlugin，或设置 RGL_GZ_PLUGIN_ROOT"
        if sim_config["lidar_mode"] == "rgl":
            raise RuntimeError(message)
        print(f"[WARN] {message}，自动切换为gpu_lidar")
        use_rgl = False

    rgl_patterns_dir = os.environ.get("RGL_PATTERNS_DIR") or os.path.dirname(pattern_file_path)
    preset_file = os.path.join(rgl_patterns_dir, "LivoxMid360.mat3x4f")
    if use_rgl and (not os.path.isfile(preset_file) or
                    os.path.getsize(preset_file) == 0 or
                    os.path.getsize(preset_file) % (48 * 40) != 0):
        message = f"MID360交替扫描pattern缺失或格式错误: {preset_file}"
        if sim_config["lidar_mode"] == "rgl":
            raise RuntimeError(message)
        print(f"[WARN] {message}，自动切换为gpu_lidar")
        use_rgl = False

    print(f"[INFO] 配置文件: {config_path}")
    print(f"[INFO] lidar_mode设置: {sim_config['lidar_mode']}")
    print(f"[INFO] 检测到NVIDIA显卡: {has_nvidia}")
    print(f"[INFO] 雷达仿真模式: {'RGL (NVIDIA)' if use_rgl else 'gpu_lidar (通用)'}")
    if use_rgl:
        print(f"[INFO] RGL插件: {rgl_system_plugin}")
        print(f"[INFO] Livox Mid360: 40帧交替pattern，{os.path.getsize(preset_file) // (48 * 40)}条射线/帧，10 Hz")

    set_gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=_join_env_path(
            "GZ_SIM_RESOURCE_PATH",
            os.path.join(pkg_share_path, "models"),
        ),
    )
    set_gz_plugin_path = SetEnvironmentVariable(
        name="GZ_SIM_SYSTEM_PLUGIN_PATH",
        value=_join_env_path(
            "GZ_SIM_SYSTEM_PLUGIN_PATH",
            rgl_system_plugin if use_rgl else "",
            "/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins",
        ),
    )
    set_gz_gui_plugin_path = SetEnvironmentVariable(
        name="GZ_GUI_PLUGIN_PATH",
        value=_join_env_path(
            "GZ_GUI_PLUGIN_PATH",
            rgl_gui_plugin if use_rgl and os.path.isdir(rgl_gui_plugin) else "",
        ),
    )

    set_rgl_patterns_dir = (
        SetEnvironmentVariable(name="RGL_PATTERNS_DIR", value=rgl_patterns_dir)
        if use_rgl
        else None
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    if not use_rgl:
        world_file = _write_fallback_world(world_file)
    robot_model_path = _write_runtime_model(
        robot_model_path,
        sim_config["livox_mid360"],
        use_rgl,
    )

    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    gazebo_options = "-s --headless-rendering " if headless else ""
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={
            "gz_args": f"{gazebo_options}-r -v 3 {world_file}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    # Use the selected map's spawn position, with optional launch overrides.
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-file",
            robot_model_path,
            "-name",
            robot_name,
            "-x",
            spawn_position["x"],
            "-y",
            spawn_position["y"],
            "-z",
            spawn_position["z"],
            "-R",
            "0.0",
            "-P",
            "0.0",
            "-Y",
            "0.0",
        ],
        output="screen",
    )

    # Bridge for clock (required for use_sim_time)
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    if use_rgl:
        lidar_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/livox/lidar@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
            ],
            remappings=[("/livox/lidar", "/livox/lidar_raw")],
            output="screen",
        )
    else:
        lidar_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="lidar_bridge_node",
            arguments=[
                "/gz_lidar_points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
            ],
            remappings=[("/gz_lidar_points/points", "/livox/lidar_raw")],
            output="screen",
        )

    livox_custom_bridge = Node(
        package="rm_sim_26",
        executable="livox_custom_bridge.py",
        name="livox_custom_bridge",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/livox/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"],
        output="screen",
    )

    chassis_imu_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/chassis/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"],
        output="screen",
    )

    if robot_model == "sentry":
        gimbal_topics = [
            "/gimbal/big_yaw/cmd_pos",
            "/gimbal/small_yaw/cmd_pos",
            "/gimbal/pitch/cmd_pos",
        ]
    else:
        gimbal_topics = ["/gimbal/yaw/cmd_pos", "/gimbal/pitch/cmd_pos"]

    gimbal_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gimbal_cmd_bridge",
        arguments=[
            f"{topic}@std_msgs/msg/Float64]gz.msgs.Double" for topic in gimbal_topics
        ],
        output="screen",
    )

    # Both chassis types use joint velocity controllers, without odometry/TF publishers.
    wheel_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="wheel_cmd_bridge",
        parameters=[{"config_file": os.path.join(pkg_share_path, "config", "os_gz_omni.yaml")}],
        output="screen",
    )
    omni_drive = Node(
        package="rm_sim_26",
        executable="omni_drive.py",
        name="omni_drive",
        parameters=[{
            "use_sim_time": use_sim_time,
            "drive_type": "omni" if robot_model == "sentry" else "mecanum",
        }],
        output="screen",
    )

    gimbal_imu_bridge = None
    if robot_model == "sentry":
        gimbal_imu_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gimbal_imu_bridge",
            arguments=["/gimbal/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"],
            output="screen",
        )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/camera@sensor_msgs/msg/Image[gz.msgs.Image"],
        output="screen",
    )

    joint_state_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model"],
        output="screen",
    )

    delayed_spawn = TimerAction(period=3.0, actions=[spawn_robot])

    launch_actions = [
        set_gazebo_resource_path,
        set_gz_plugin_path,
        set_gz_gui_plugin_path,
        gazebo_launch,
        delayed_spawn,
        clock_bridge,
        wheel_bridge,
        omni_drive,
        lidar_bridge,
        livox_custom_bridge,
        imu_bridge,
        chassis_imu_bridge,
        gimbal_bridge,
        camera_bridge,
        joint_state_bridge,
    ]

    if set_rgl_patterns_dir is not None:
        launch_actions.insert(3, set_rgl_patterns_dir)
    if gimbal_imu_bridge is not None:
        launch_actions.append(gimbal_imu_bridge)
    if publish_tf:
        robot_urdf_path = os.path.join(
            pkg_share_path, "models", robot_model, "robot.urdf"
        )
        launch_actions.append(Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(
                    _read_text(robot_urdf_path), value_type=str
                ),
            }],
            output="screen",
        ))

    return launch_actions


def generate_launch_description():
    pkg_share = get_package_share_directory("rm_sim_26")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "config_file",
            default_value=os.path.join(pkg_share, "config", "sim_config.yaml"),
        ),
        DeclareLaunchArgument(
            "lidar_mode", default_value="",
            description="Override YAML: auto, rgl or gpu_lidar",
        ),
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Run server with headless rendering and no GUI",
        ),
        DeclareLaunchArgument(
            "spawn_x", default_value="",
            description="Override the selected map's YAML spawn x coordinate",
        ),
        DeclareLaunchArgument(
            "spawn_y", default_value="",
            description="Override the selected map's YAML spawn y coordinate",
        ),
        DeclareLaunchArgument(
            "spawn_z", default_value="",
            description="Override the selected map's YAML spawn z coordinate",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
