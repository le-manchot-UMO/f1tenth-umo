#!/bin/bash
# Turn everything this car recorded into demo material, with no network needed.
#
#   make_demo.sh [output_dir]        default ~/f1tenth_demo
#
# Encodes every captured frame directory into an MP4, copies the saved maps and
# renders them as PNGs, and writes an index of what came from where. Rosbags are
# NOT copied (they are hundreds of MB); their paths are listed instead.
#
# ffmpeg is installed locally, so this works entirely offline.
set -o pipefail
OUT="${1:-$HOME/f1tenth_demo}"
FPS="${FPS:-8}"
mkdir -p "$OUT/videos" "$OUT/maps" "$OUT/snapshots"

echo "=== encoding run videos (fps ${FPS}) ==="
for d in "$HOME"/f1tenth_video/*/; do
  [[ -d "$d" ]] || continue
  name=$(basename "$d")
  n=$(ls "$d"/frame_*.png 2>/dev/null | wc -l)
  if [[ "$n" -lt 5 ]]; then
    echo "  skip ${name} (only ${n} frames)"; continue
  fi
  echo "  ${name}: ${n} frames -> ${name}.mp4"
  ffmpeg -y -loglevel error -framerate "${FPS}" -i "${d}frame_%05d.png" \
     -c:v libx264 -pix_fmt yuv420p \
     -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" \
     "$OUT/videos/${name}.mp4" 2>&1 | tail -2
done

echo "=== maps ==="
cp -v "$HOME"/maps/*.pgm "$HOME"/maps/*.yaml "$OUT/maps/" 2>/dev/null | tail -4
# render each saved map to PNG so it can be dropped straight into a slide
python3 - "$OUT/maps" <<'PY' 2>/dev/null || echo "  (PNG render skipped)"
import glob, os, sys
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
d = sys.argv[1]
for f in sorted(glob.glob(os.path.join(d, "*.pgm"))):
    with open(f, "rb") as fh:
        assert fh.readline().strip() == b"P5"
        line = fh.readline()
        while line.startswith(b"#"): line = fh.readline()
        w, h = map(int, line.split()); int(fh.readline())
        a = np.frombuffer(fh.read(w*h), dtype=np.uint8).reshape(h, w)
    plt.figure(figsize=(10, 10*h/w), dpi=140)
    plt.imshow(a, cmap="gray", origin="upper"); plt.axis("off")
    plt.title(f"{os.path.basename(f)} — {w}x{h} @ 0.02 m/cell "
              f"({w*0.02:.1f} x {h*0.02:.1f} m)", fontsize=10)
    out = f.replace(".pgm", ".png")
    plt.savefig(out, bbox_inches="tight"); plt.close()
    print("  rendered", os.path.basename(out))
PY

echo "=== snapshots ==="
cp "$HOME"/f1tenth_maps/*.png "$OUT/snapshots/" 2>/dev/null
echo "  $(ls "$OUT/snapshots" 2>/dev/null | wc -l) snapshot PNGs"

{
  echo "F1TENTH exploration — demo material"
  echo "generated: $(date)"
  echo
  echo "videos/     one MP4 per run: SLAM map | global costmap | local costmap,"
  echo "            with the car's track drawn in red"
  echo "maps/       saved maps (.pgm + .yaml, and .png renders)"
  echo "snapshots/  individual map/costmap/laserscan captures"
  echo
  echo "ROSBAGS (not copied — large; replay from these paths):"
  du -sh "$HOME"/f1tenth_runs/* 2>/dev/null
  echo
  echo "Biggest run: 20260821_142122 — car drove ~15 m autonomously,"
  echo "mapped 12.1 x 18.8 m. That is the one to lead the demo with."
} > "$OUT/README.txt"

echo
echo "=== done -> ${OUT} ==="
du -sh "$OUT"/* 2>/dev/null
