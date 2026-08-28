# WWCutter

Cross-platform four-axis hot-wire CAM and machine controller. It imports a
STEP solid or shell, finds (or lets the user select) its two end sections,
verifies that straight wire rulings reproduce the CAD surface, projects those
rulings to the XY and UV towers, and previews the complete machine motion.

## Setup

Python 3.11 or newer is recommended on Windows, macOS, and Linux.

```bash
conda activate foamcut
python -m pip install -r requirements.txt
python WWCutterV01.py
```

CadQuery includes Open Cascade and reads STEP units as millimetres. No native
compiler or platform-specific application changes are required when its wheel
is available for the selected Python version.

## STEP workflow

1. Select **Import STEP**. Ordinary two-section lofts are detected
   automatically.
2. If necessary, choose **Pick Root** or **Pick Tip** and click a highlighted
   planar face in the preview. Open-shell boundary loops are also available in
   the two section lists.
3. Enter the tower spacing, distance from the left tower to the root face, and
   horizontal/vertical travel. Choose whether internal contours are ignored or
   cut through an entry slit.
4. Choose **Resulting cut surface** to inspect the complete surface swept by
   the physical wire, or **Complete machine** to see it in the tower workspace.
   Move the slider to inspect every individual wire position. Green and blue
   mark the two foam sections or tower paths; yellow is the current wire.
   Dense models are shown at reduced detail while rotating; this affects only
   the display, not the calculated cut.
5. With the heater off, use **Move to Cut Start** to perform the dashed
   positioning move shown in the machine preview.
6. A STEP job can start only when its measured ruled-surface deviation is no
   greater than the configured tolerance and all projected moves fit the
   workspace.

Deviation is checked in both directions at intermediate span sections. This
catches STEP skins that bow between otherwise valid root and tip profiles—a
straight hot wire can create only a ruled surface. Ignored internal contours
do not contribute motion or deviation, and the preview labels the measurement
as outer-only. Through contours can be cut with an entry slit; blind contours
that do not reach both foam end faces remain blocked.

The older paired-DXF workflow remains available in the **Legacy two-DXF
input** section.
