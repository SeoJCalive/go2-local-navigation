"""Independent raster controls: exact wall, shifted wall, and an extra wall."""
import json
from pathlib import Path
import tempfile

import numpy as np
from PIL import Image

from geometry import evaluate


def main():
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
        assert exact["wall_offset_m"]["max"]==0
        assert exact["doubled_wall_components"]==0
        assert exact["ghost_wall_components"]==0
        assert exact["normal_profile_offset_m"]["max"]<=.025
        shifted=np.full_like(image,254)
        shifted[35:37,20:60]=0  # Exactly .20m shifted in +y.
        Image.fromarray(shifted).save(root/"map.pgm")
        bad=evaluate(world,root/"map.yaml",[0,0,0],root/"shifted.png")
        assert bad["wall_offset_m"]["p95"]>.10
        double=image.copy()
        double[35:37,20:60]=0
        Image.fromarray(double).save(root/"map.pgm")
        extra=evaluate(world,root/"map.yaml",[0,0,0],root/"double.png")
        assert extra["doubled_wall_components"]>=1
        print(json.dumps({"exact":exact["wall_offset_m"],"shifted":bad["wall_offset_m"],
                          "doubled_components":extra["doubled_wall_components"]}))


if __name__=="__main__":
    main()
