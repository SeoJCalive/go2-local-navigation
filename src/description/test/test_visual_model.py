"""The mesh-free visual must preserve every canonical joint and collision."""
from pathlib import Path
import xml.etree.ElementTree as ET

from description.visual_model import collision_visual_model


def test_collision_visuals_preserve_model_contract():
    source = (Path(__file__).parents[1] / "urdf/go2_description.urdf").read_text()
    original = ET.fromstring(source)
    result = ET.fromstring(collision_visual_model(source))
    assert not result.findall(".//mesh")
    assert [ET.tostring(j) for j in original.findall("joint")] == [
        ET.tostring(j) for j in result.findall("joint")]
    for before, after in zip(original.findall("link"), result.findall("link")):
        assert [ET.tostring(c) for c in before.findall("collision")] == [
            ET.tostring(c) for c in after.findall("collision")]
        assert len(after.findall("visual")) == len(before.findall("collision"))
        for collision, visual in zip(before.findall("collision"), after.findall("visual")):
            assert ET.tostring(collision.find("geometry")) == ET.tostring(visual.find("geometry"))
