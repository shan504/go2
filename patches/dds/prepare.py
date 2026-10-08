#!/usr/bin/env python3
"""Prepare copies of installed SDK files. No source/image/system files are edited."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml


def set_real_time(obj):
    if isinstance(obj, dict):
        for key, value in obj.items():
            if key == "use_sim_time":
                obj[key] = False
            else:
                set_real_time(value)
    elif isinstance(obj, list):
        for item in obj:
            set_real_time(item)


def prepare(sdk, output):
    output.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(sdk / "urdf/go2.urdf")
    root = tree.getroot()
    removed = []
    for joint in list(root.findall("joint")):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        if (parent, child) in (("map", "odom"), ("odom", "base_link")):
            if joint.get("type") != "fixed":
                raise RuntimeError("Unexpected dynamic world joint in SDK URDF")
            removed.append(joint.get("name"))
            root.remove(joint)
    for joint in root.findall("joint"):
        if any(joint.find(part).get("link") in ("map", "odom") for part in ("parent", "child")):
            raise RuntimeError("Other map/odom joints exist; refusing an ambiguous URDF edit")
    for link in list(root.findall("link")):
        if link.get("name") in ("map", "odom"):
            root.remove(link)
    if not any(link.get("name") == "base_link" for link in root.findall("link")):
        raise RuntimeError("SDK URDF has no base_link")
    # The original SDK may use relative mesh paths; resolve only file paths,
    # while preserving package:// references for installed SDK assets.
    for mesh in root.iter("mesh"):
        filename = mesh.get("filename", "")
        if filename and not "://" in filename and not filename.startswith("/"):
            mesh.set("filename", str((sdk / "urdf" / filename).resolve()))
    tree.write(output / "go2_edu.urdf", encoding="utf-8", xml_declaration=True)
    nav = yaml.safe_load((sdk / "config/nav2_params.yaml").read_text())
    set_real_time(nav)
    global_map = nav["global_costmap"]["global_costmap"]["ros__parameters"]
    global_map.update(width=60, height=60, origin_x=-30.0, origin_y=-30.0)
    # Do not treat unobserved space as an open passage.
    global_map["track_unknown_space"] = True
    for section in ("local_costmap", "global_costmap"):
        params = nav[section][section]["ros__parameters"]
        params["voxel_layer"]["publish_voxel_map"] = False
        params["voxel_layer"]["scan"]["inf_is_valid"] = True
    follow = nav["controller_server"]["ros__parameters"]["FollowPath"]
    follow.update(max_vel_x=0.15, max_vel_y=0.0, max_vel_theta=0.3,
                  max_speed_xy=0.15, trans_stopped_velocity=0.02,
                  acc_lim_x=0.4, acc_lim_y=0.4, acc_lim_theta=0.6,
                  decel_lim_x=-0.4, decel_lim_y=-0.4, decel_lim_theta=-0.6,
                  vy_samples=1, debug_trajectory_details=False)
    behaviors = nav["behavior_server"]["ros__parameters"]
    behaviors.update(max_rotational_vel=0.3, min_rotational_vel=0.1, rotational_acc_lim=0.6)
    nav["planner_server"]["ros__parameters"]["GridBased"]["tolerance"] = 0.25
    nav["velocity_smoother"] = {"ros__parameters": {
        "use_sim_time": False, "smoothing_frequency": 20.0,
        "scale_velocities": False, "feedback": "OPEN_LOOP",
        "max_velocity": [0.15, 0.0, 0.3], "min_velocity": [-0.15, 0.0, -0.3],
        "max_accel": [0.4, 0.4, 0.6], "max_decel": [-0.4, -0.4, -0.6],
        "odom_topic": "/odom", "odom_duration": 0.1,
        "deadband_velocity": [0.0, 0.0, 0.0], "velocity_timeout": 0.5}}
    (output / "nav2_edu.yaml").write_text(yaml.safe_dump(nav, sort_keys=False))
    slam = yaml.safe_load((sdk / "config/mapper_params_online_async.yaml").read_text())
    set_real_time(slam)
    slam["slam_toolbox"]["ros__parameters"].update(
        use_sim_time=False, odom_frame="odom", map_frame="map", base_frame="base_link",
        scan_topic="/scan", mode="mapping", map_update_interval=2.0)
    (output / "slam_edu.yaml").write_text(yaml.safe_dump(slam, sort_keys=False))
    print("Prepared configuration copies; removed fixed joints: " + ", ".join(removed))


if __name__ == "__main__":
    from ament_index_python.packages import get_package_share_directory
    prepare(Path(get_package_share_directory("go2_robot_sdk")), Path(__file__).resolve().parent / "config")
