#!/usr/bin/env bash
# Baut und startet den UI-Simulator. Benoetigt: git, gcc/g++, python3 (+ Pillow fuer PNG, optional)
# Ergebnis: tools/sim/build/out_<seite>.png (bzw. .ppm) und eine Liste der Elemente ausserhalb des Kreises.
set -e
SIM="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$SIM/../.." && pwd)"
LVGL_VER="${LVGL_VER:-v9.5.0}"  # wie in der Arduino IDE installiert
B="$SIM/build/$LVGL_VER"
mkdir -p "$B"

# 1) LVGL holen (einmalig)
if [ ! -d "$B/lvgl" ]; then
  git -c advice.detachedHead=false clone -q --depth 1 --branch "$LVGL_VER" https://github.com/lvgl/lvgl "$B/lvgl"
fi

# 2) lv_conf.h aus der Vorlage erzeugen: aktivieren, Fonts wie im Sketch, grosser Heap
if [ ! -f "$B/lv_conf.h" ]; then
  sed -e '0,/^#if 0/s//#if 1/' \
      -E -e 's/#define LV_FONT_MONTSERRAT_(12|14|16|20|24|32|40|48) +0/#define LV_FONT_MONTSERRAT_\1 1/' \
      -e 's/#define LV_MEM_SIZE \(64 \* 1024U\)/#define LV_MEM_SIZE (1024 * 1024U)/' \
      "$B/lvgl/lv_conf_template.h" > "$B/lv_conf.h"
fi

# 3) LVGL als Bibliothek bauen (einmalig, parallel)
if [ ! -f "$B/liblvgl.a" ]; then
  echo "Baue LVGL $LVGL_VER (einmalig, dauert etwas)..."
  mkdir -p "$B/obj"
  i=0
  while read -r f; do
    i=$((i+1))
    gcc -c -O1 -w -DLV_CONF_INCLUDE_SIMPLE -I"$B" -I"$B/lvgl" "$f" -o "$B/obj/$i.o" &
    [ $((i % 16)) -eq 0 ] && wait
  done < <(find "$B/lvgl/src" -name '*.c')
  wait
  ar rcs "$B/liblvgl.a" "$B"/obj/*.o
fi

# 4) Sketch + Simulator bauen und ausfuehren
cp "$ROOT/OBD.ino" "$B/sketch.cpp"
g++ -std=gnu++17 -Wall -Wno-unused -DLV_CONF_INCLUDE_SIMPLE \
    -I"$B" -I"$SIM/stub" -I"$B/lvgl" -idirafter "$ROOT" -include Arduino.h \
    "$SIM/sim_main.cpp" "$B/liblvgl.a" -lm -o "$B/sim"
cd "$B"
rm -f out_*.ppm out_*.png
set +e
./sim
RC=$?
set -e

# 5) PPM -> PNG, Bereich ausserhalb des runden Displays dunkelrot
python3 - <<'PY' || echo "(Pillow nicht installiert: Bilder bleiben als .ppm)"
from PIL import Image, ImageDraw
import glob, os
for f in sorted(glob.glob('out_*.ppm')):
    im = Image.open(f).convert('RGB')
    mask = Image.new('L', (480, 480), 0)
    ImageDraw.Draw(mask).ellipse((0, 0, 479, 479), fill=255)
    Image.composite(im, Image.new('RGB', (480, 480), (60, 0, 0)), mask).save(f[:-4] + '.png')
    os.remove(f)
PY
echo "Bilder: $B/out_*.png"
exit $RC
