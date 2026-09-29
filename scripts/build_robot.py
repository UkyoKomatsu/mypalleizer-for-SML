"""Turn Elephant Robotics' Galactic Pi URDF into a Gazebo Classic model.

The vendor meshes and source URDF remain unmodified in this repository.
"""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "urdf" / "robot.urdf"
tree = ET.parse(ROOT / "urdf" / "vendor_robot.urdf")
robot = tree.getroot()
robot.set("name", "mypalletizer_260_pi_sim")
base_anchor = ET.SubElement(robot, "link", name="world")
anchor_joint = ET.SubElement(robot, "joint", name="world_to_base", type="fixed")
ET.SubElement(anchor_joint, "parent", link="world")
ET.SubElement(anchor_joint, "child", link="base")
ET.SubElement(anchor_joint, "origin", xyz="0 0 0", rpy="0 0 0")

for mesh in robot.iter("mesh"):
    name = mesh.get("filename", "")
    if "/mypalletizer_260_pi/" in name:
        mesh.set("filename", "file:///opt/sim/meshes/arm/" + name.rsplit("/", 1)[-1])
    elif "/adaptive_gripper/" in name:
        mesh.set("filename", "file:///opt/sim/meshes/gripper/" + name.rsplit("/", 1)[-1])

# The vendor description is for RViz and has no inertial data. These conservative
# placeholder values are sufficient for initial motion tests, not force studies.
for link in robot.findall("link"):
    if link.get("name") == "world":
        continue
    # The vendor collision meshes overlap and destabilize Gazebo with the
    # placeholder masses. Stage one checks motion and sensing, not grasp forces.
    for collision in list(link.findall("collision")):
        link.remove(collision)
    ET.SubElement(ET.SubElement(robot, "gazebo", reference=link.get("name")), "gravity").text = "false"
    mass = 0.35 if link.get("name") in {"link1", "link2", "link3", "link4"} else 0.08
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=str(mass))
    ET.SubElement(inertial, "inertia", ixx="0.001", ixy="0", ixz="0", iyy="0.001", iyz="0", izz="0.001")

controlled = [
    "joint1_to_base", "joint2_to_joint1", "joint3_to_joint2",
    "joint5_to_joint4", "gripper_controller",
]
for joint in robot.findall("joint"):
    limit = joint.find("limit")
    if limit is not None and joint.get("type") == "revolute":
        limit.set("velocity", "1.5")
        limit.set("effort", "12")

control = ET.SubElement(robot, "ros2_control", name="GazeboSystem", type="system")
ET.SubElement(ET.SubElement(control, "hardware"), "plugin").text = "gazebo_ros2_control/GazeboSystem"
for name in controlled:
    element = ET.SubElement(control, "joint", name=name)
    ET.SubElement(element, "command_interface", name="position")
    ET.SubElement(element, "state_interface", name="position")
    ET.SubElement(element, "state_interface", name="velocity")
for joint in robot.findall("joint"):
    if joint.find("mimic") is not None:
        element = ET.SubElement(control, "joint", name=joint.get("name"))
        ET.SubElement(element, "state_interface", name="position")
        ET.SubElement(element, "state_interface", name="velocity")

plugin = ET.SubElement(ET.SubElement(robot, "gazebo"), "plugin", name="gazebo_ros2_control", filename="libgazebo_ros2_control.so")
ET.SubElement(plugin, "parameters").text = "/opt/sim/config/controllers.yaml"

camera = ET.SubElement(robot, "link", name="d435_link")
visual = ET.SubElement(camera, "visual")
ET.SubElement(ET.SubElement(visual, "geometry"), "box", size="0.09 0.025 0.025")
inertial = ET.SubElement(camera, "inertial")
ET.SubElement(inertial, "mass", value="0.08")
ET.SubElement(inertial, "inertia", ixx="0.0001", ixy="0", ixz="0", iyy="0.0001", iyz="0", izz="0.0001")
mount = ET.SubElement(robot, "joint", name="d435_mount", type="fixed")
ET.SubElement(mount, "parent", link="gripper_base")
ET.SubElement(mount, "child", link="d435_link")
# Values follow the offsets in legacy/config.py (millimetres), pending measurement.
ET.SubElement(mount, "origin", xyz="0.075 0.038 0.040", rpy="0 0.139626 1.570796")
sensor = ET.SubElement(ET.SubElement(robot, "gazebo", reference="d435_link"), "sensor", name="d435", type="depth")
ET.SubElement(sensor, "always_on").text = "true"
ET.SubElement(sensor, "update_rate").text = "15"
camera_spec = ET.SubElement(sensor, "camera")
ET.SubElement(camera_spec, "horizontal_fov").text = "1.204"
image = ET.SubElement(camera_spec, "image")
for key, value in {"width": "640", "height": "480", "format": "R8G8B8"}.items():
    ET.SubElement(image, key).text = value
clip = ET.SubElement(camera_spec, "clip")
ET.SubElement(clip, "near").text = "0.05"
ET.SubElement(clip, "far").text = "2.0"
cam_plugin = ET.SubElement(sensor, "plugin", name="d435_ros", filename="libgazebo_ros_camera.so")
ET.SubElement(cam_plugin, "frame_name").text = "d435_link"
ET.SubElement(cam_plugin, "camera_name").text = "d435"
ET.SubElement(cam_plugin, "min_depth").text = "0.05"
ET.SubElement(cam_plugin, "max_depth").text = "2.0"

tree.write(MODEL, encoding="unicode", xml_declaration=False)
print(MODEL)
