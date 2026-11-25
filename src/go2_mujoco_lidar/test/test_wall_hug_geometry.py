"""G02 behavioral regressions: numerical controls and OS signals, no robot or GUI.

Inputs are independent synthetic poses/contacts/maps; pytest reports failures.
See wall_hug/README.md and G02_WALL_HUG.md for thresholds and provenance.
These checks establish harness contracts; physical validity is unconfirmed.
"""
from pathlib import Path
import sys
import pytest
ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT/"wall_hug"))
import json
import tempfile
import numpy as np
from PIL import Image
from geometry import evaluate, map_criteria

def test_independent_maps_and_gates():
    with tempfile.TemporaryDirectory() as temp:
        root=Path(temp)
        world=root/"world.xml"
        world.write_text('<mujoco><worldbody><geom name="wall" type="box" '
                         'pos="0 0 .75" size="1 .06 .75"/></worldbody></mujoco>')
        (root/"map.yaml").write_text("image: map.pgm\nresolution: 0.05\n"
            "origin: [-2.0, -2.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
        image=np.full((80,80),254,dtype=np.uint8)
        # Physical wall x [-1,1], y [-.06,.06]. Cell centers +/- .025.
        image[39:41,20:60]=0
        Image.fromarray(image).save(root/"map.pgm")
        exact=evaluate(world,root/"map.yaml",[0,0,0],root/"exact.png")
        assert all(map_criteria(exact).values())
        assert exact["wall_offset_m"]["max"]==0
        assert exact["doubled_wall_components"]==0
        assert exact["ghost_wall_components"]==0
        assert exact["normal_profile_offset_m"]["max"]<=.025
        shifted=np.full_like(image,254)
        shifted[35:37,20:60]=0  # Exactly .20m shifted in +y.
        Image.fromarray(shifted).save(root/"map.pgm")
        bad=evaluate(world,root/"map.yaml",[0,0,0],root/"shifted.png")
        assert bad["wall_offset_m"]["p95"]>.10
        assert not map_criteria(bad)["wall_offset_p95"]
        double=image.copy()
        double[35:37,20:60]=0
        Image.fromarray(double).save(root/"map.pgm")
        extra=evaluate(world,root/"map.yaml",[0,0,0],root/"double.png")
        assert extra["doubled_wall_components"]>=1
        assert not map_criteria(extra)["doubled_walls"]
        ghost=image.copy(); ghost[10:15,10:15]=0
        Image.fromarray(ghost).save(root/"map.pgm")
        phantom=evaluate(world,root/"map.yaml",[0,0,0],root/"ghost.png")
        assert phantom["ghost_wall_components"] == 1
        assert not map_criteria(phantom)["ghost_walls"]
        thick=image.copy(); thick[34:46,20:60]=0
        Image.fromarray(thick).save(root/"map.pgm")
        broad=evaluate(world,root/"map.yaml",[0,0,0],root/"thick.png")
        assert not map_criteria(broad)["wall_thickness"]
        print(json.dumps({"exact":exact["wall_offset_m"],"shifted":bad["wall_offset_m"],
                          "doubled_components":extra["doubled_wall_components"]}))
