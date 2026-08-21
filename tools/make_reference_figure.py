"""Turn the simulator's G1 render into a HUD-ready reference figure.

The source (download.png) is a front-on capture of the G1 from the simulator on
a white background. Dropped into the align console as-is it reads as a white
rectangle, because the console is near-black. This keys the background out to
alpha and trims to the figure, so the robot floats on the panel.

The key is on brightness, not on an exact white match: the render has a soft
antialiased edge and a faint drop shadow, and a hard == 255 test leaves a white
halo one pixel wide around the whole silhouette, which is more visible against
a dark panel than the background was.

Run once; the result is committed. Regenerate only if the render changes:

    python tools/make_reference_figure.py
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from PIL import Image

REPO = Path(__file__).resolve().parent.parent
SOURCE = Path("C:/G1_VR/download.png")
DEST = REPO / "quest_app" / "Assets" / "Resources" / "g1_reference.png"

# Anything at or above WHITE is fully transparent; at or below KEEP is fully
# opaque; between the two, alpha ramps. The gap is what dissolves the
# antialiased edge instead of cutting it.
WHITE = 246.0
KEEP = 205.0

# The figure is mostly mid-grey, so the console's green tint has something to
# work with. Widening the range keeps the panel from flattening it to a blob.
CONTRAST = 1.16


def main() -> int:
    if not SOURCE.exists():
        print(f"no source render at {SOURCE}", file=sys.stderr)
        return 1

    img = Image.open(SOURCE).convert("RGBA")
    rgb = np.asarray(img, dtype=np.float32)[..., :3]

    # Brightness of the lightest channel: a pixel is background only if it is
    # bright in all three, which keeps the robot's own light-grey panels.
    lightest = rgb.max(axis=-1)
    alpha = np.clip((WHITE - lightest) / (WHITE - KEEP), 0.0, 1.0)

    # Drop specks left by JPEG-ish ringing in the shadow: anything under 8%
    # alpha is not part of the figure.
    alpha[alpha < 0.08] = 0.0

    mid = rgb.mean()
    out = np.clip((rgb - mid) * CONTRAST + mid, 0.0, 255.0)

    rgba = np.dstack([out, alpha * 255.0]).astype(np.uint8)
    keyed = Image.fromarray(rgba, mode="RGBA")

    bbox = keyed.getbbox()
    if bbox:
        keyed = keyed.crop(bbox)

    # Height-limited: the HUD draws it about 40mm tall. Anything larger is
    # texture memory spent on detail no one can resolve through a headset.
    target_h = 512
    if keyed.height > target_h:
        w = round(keyed.width * target_h / keyed.height)
        keyed = keyed.resize((w, target_h), Image.LANCZOS)

    DEST.parent.mkdir(parents=True, exist_ok=True)
    keyed.save(DEST)
    print(f"wrote {DEST}  {keyed.width}x{keyed.height}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
