# MP2 — collection protocol

**40 sessions x 50 frames = 2000 frames.** Roughly three sessions a week to
30 November. A finished 40 is worth more than an abandoned 90.

---

## 0. Three things that cannot be recovered afterwards

| | Why it is gone |
|---|---|
| **True distance** | The image does not contain it. An unlabelled frame is a picture, not a data point |
| **Session identity** | Which frames share a camera setup, a room and a lighting condition. Lose it and the set cannot be split into train and test without leaking |
| **Deliberately hard frames** | A frame that was hard on purpose and a frame that failed by accident look identical later |

Everything else — crops, resizes, re-detection, derived features — can be
redone from the file. These three cannot. Write them down as you shoot.

---

## 1. Print

Two PDFs, and they do different jobs.

| File | What it is for |
|---|---|
| `print/aruco_4x4_100mm.pdf` | 4 pages, one ArUco marker each, IDs 0–3, `DICT_4X4_50`, 100.0000 mm nominal. **These appear in the frames** |
| `print/charuco_a4_landscape.pdf` | 1 page, ChArUco board, `DICT_5X5_100`, 7x5 squares of 35.0000 mm. **This measures the focal length** |

Different dictionaries on purpose: a frame containing both targets would
otherwise report the same marker ID twice with no way to say which target it
came from.

**Printing:**

- **100%. No "fit to page", no "shrink to fit", no borderless.** This is the
  single most common way the whole dataset ends up with a scale error nothing
  downstream can detect.
- **Matte paper.** Gloss produces a specular highlight that blows out part of
  the marker at exactly the angles that matter.
- Flat and unbent. Do not laminate — most laminate is glossy.
- Print at least two copies of each. A creased marker is not repairable and
  reprinting mid-campaign means re-measuring.

**Then verify, before shooting anything:**

1. The 150.0000 mm scale bar at the foot of each marker page measures
   150.0000 mm with a rule. If it does not, the page did not print at 100%.
2. **Measure the black square with calipers, outer edge to outer edge.**
   Write it on the sheet. It will not be exactly 100 mm and it does not need
   to be — it needs to be *known*.
3. On the calibration board, measure across **five** squares and divide by
   five. One square carries the printer's error in full; five carries a fifth
   of it.

Every range estimate divides by the measured marker width. A printer that came
out at 99.6 mm and a manifest that says 100.0 give a 0.4% distance error on
every frame in the set, silently, forever.

---

## 2. Camera

The camera is the laptop's built-in one for now.

**Before session 1, find out what it actually is.** Not what the spec sheet
says:

```powershell
python -c "import cv2; c=cv2.VideoCapture(0); print(c.read()[1].shape); c.release()"
```

Then **calibrate with the ChArUco board.** 20–30 shots of the board at
varied angles and distances, filling different parts of the frame, including
the corners. Calibration returns the focal length in pixels, `f_px`, and the
lens distortion.

**A field of view off a spec sheet is not a measurement and must not be used
for anything but rough planning.** Laptop webcams are routinely 10–15° away
from their quoted figure, and the range estimate is linear in `f_px`.

### What the numbers will look like

Range from a marker is

```
    Z  =  f_px * W_mm / w_px
```

where `W_mm` is the **measured** marker width and `w_px` is its observed width
in the image. Detection of a 4x4 marker needs roughly 30 px of marker width to
be reliable and falls apart below about 20.

Planning figures only — replace with the calibrated `f_px`:

| Sensor | Assumed horizontal FOV | `f_px` | Reliable to (30 px) | Absolute limit (20 px) |
|---|---|---|---|---|
| 640 x 480 | 55° | ~615 | ~2.0 m | ~3.1 m |
| 1280 x 720 | 60° | ~1108 | ~3.7 m | ~5.5 m |
| 1920 x 1080 | 60° | ~1663 | ~5.5 m | ~8.3 m |

Near limit is set by focus, not geometry — most webcams are fixed-focus and
go soft below about 300 mm. Find yours in session 1 and record it.

**If the camera turns out softer than nominal, the fallback is a 120 mm
marker**, which is still inside A4 with its quiet zone. Do not go to 150 mm:
it does not print safely inside normal margins.

---

## 3. A session

One session = one camera setup, one place, one lighting condition, 50 frames.
Change any of those and it is a new session.

**Vary across the 40 sessions, deliberately and on a plan:**

| Axis | Spread across sessions |
|---|---|
| Distance | every bin the checker prints, including the far one |
| Angle | marker square-on through about 60° of yaw |
| Lighting | bright, normal, dim, mixed, backlit |
| Background | plain, cluttered, and at least a few with texture resembling the marker |
| Blur | mostly none; some motion; some defocus |
| Occlusion | mostly none; some partial, and record the percentage |

**Roughly 15–20% of frames should be deliberately hard** and marked `hard=1`.
A dataset with no hard frames reports an accuracy that does not survive
contact with a real robot; a dataset that is all hard frames does not train.

Diversity designed in advance beats diversity accumulated. Decide before
session 1 which sessions cover which conditions, or the last ten sessions will
all be the same easy setup because that one is convenient.

---

## 4. Recording

Copy `data/manifest_template.csv` and fill one row per frame as you shoot.
Filenames as `S007_0031.jpg` — session and index, zero-padded, so sorting and
grouping never depend on a separate index.

Measure distance from the **camera's sensor plane** to the **marker's plane**,
not to the front of the laptop lid and not to the wall behind. Pick a method
and stay with it: `tape`, `laser` or `jig`. A tape at 3 m held by one person
is worth about ±20 mm, which is 0.7% — acceptable, but only if it is the same
±20 mm every time.

---

## 5. Check it, monthly

```powershell
python -m tools.check_manifest --manifest data/manifest.csv --frames data/frames
```

It fails on the things that cannot be repaired later — a frame with no
distance, a frame with no session, a file on disk that no row mentions — and
warns on the things that are probably wrong but are your call, including a
`marker_width_mm` of exactly 100.0, which is the nominal value rather than a
measurement.

Run it after every session for the first three, then monthly. A systematic
recording mistake found in week two costs one session; found in November it
costs the campaign.
