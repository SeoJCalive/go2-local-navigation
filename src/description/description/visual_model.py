"""Simulation RViz model derived from canonical collision primitives.

The official URDF and all joint/TF geometry stay unchanged on disk. Simulation
uses its collision shapes as visuals, so no external go2_description DAE package
is needed. This representation shows articulation, not the cosmetic Go2 shell.
"""
from copy import deepcopy
import xml.etree.ElementTree as ET


def collision_visual_model(urdf):
    robot = ET.fromstring(urdf)
    for link in robot.findall("link"):
        for visual in link.findall("visual"):
            link.remove(visual)
        for collision in link.findall("collision"):
            visual = deepcopy(collision)
            visual.tag = "visual"
            material = ET.SubElement(visual, "material", name="simulation_collision")
            ET.SubElement(material, "color", rgba="0.7 0.75 0.8 1")
            link.append(visual)
    return ET.tostring(robot, encoding="unicode")
