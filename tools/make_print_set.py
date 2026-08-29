"""Generate the MP2 print set: range markers and a camera calibration board.

Two different jobs, two different targets, and they must not be confused:

  MARKER SHEETS      one ArUco marker per A4 page, DICT_4X4_50, 100.0000 mm.
                     These are what appears in the collected frames. The range
                     estimate divides by this marker's TRUE printed width, so
                     the number that matters is the one measured off the paper
                     with calipers, not the one written here.

  CALIBRATION BOARD  a ChArUco board, DICT_5X5_100. This is what actually
                     MEASURES the focal length. A field of view taken off a
                     spec sheet is not a measurement and must not be used to
                     size anything real.

DIFFERENT DICTIONARIES ON PURPOSE. A ChArUco board built from 4X4_50 would put
IDs 0..16 on the calibration board and IDs 0..3 on the range markers, and a
frame containing both would report the same ID twice with no way to say which
target it came from.

Vector where the layout is simple enough to verify, raster where it is not:
the single markers are drawn as filled rectangles straight from the
dictionary's own bit matrix, and check_marker_roundtrip() renders that same
rectangle list and detects it back. The ChArUco layout is OpenCV's, kept as
OpenCV's, rendered at 1200 dpi and detected back before it is embedded.
Hand-placing 17 markers and their parity would be a layout I could get subtly
wrong in a way nothing here would catch.

    python -m tools.make_print_set --out print/
"""

import argparse
from pathlib import Path

import cv2
import numpy as np
from reportlab.lib.pagesizes import A4, landscape
from reportlab.lib.units import mm
from reportlab.lib.utils import ImageReader
from reportlab.pdfgen import canvas

# --- what gets printed -----------------------------------------------------

MARKER_DICT = cv2.aruco.DICT_4X4_50
MARKER_IDS = (0, 1, 2, 3)
MARKER_MM = 100.0000          # nominal. The measured value is what counts.
MARKER_BORDER_CELLS = 1       # ArUco's own black border, part of the 100 mm

BOARD_DICT = cv2.aruco.DICT_5X5_100
BOARD_COLS, BOARD_ROWS = 7, 5
BOARD_SQUARE_MM = 32.0000
BOARD_MARKER_MM = 24.0000
BOARD_DPI = 600


# --------------------------------------------------------------------------
# Marker geometry
# --------------------------------------------------------------------------

def marker_cells(dictionary, marker_id):
    """The 6x6 cell grid for one marker: 4x4 of payload inside a 1-cell border.

    generateImageMarker with sidePixels equal to the cell count returns exactly
    one pixel per cell, so no thresholding or resampling stands between the
    dictionary and the rectangles that get drawn.
    """
    n = dictionary.markerSize + 2 * MARKER_BORDER_CELLS
    img = dictionary.generateImageMarker(marker_id, n, borderBits=MARKER_BORDER_CELLS)
    if img.shape != (n, n):
        raise RuntimeError("expected a %dx%d cell grid, got %s" % (n, n, img.shape))
    return (img == 0)          # True where the cell is black


def black_runs(cells):
    """Merge horizontally adjacent black cells into runs.

    Rectangles that share an edge can leave a hairline seam once a printer has
    had its way with them, and a seam inside a black region is a false corner
    to a detector. One rectangle per run rather than one per cell.
    """
    runs = []
    n = cells.shape[0]
    for r in range(n):
        c = 0
        while c < n:
            if cells[r, c]:
                start = c
                while c < n and cells[r, c]:
                    c += 1
                runs.append((r, start, c - start))
            else:
                c += 1
    return runs


def check_marker_roundtrip(dictionary, marker_id, px_per_cell=40):
    """Render the rectangle list this file will draw, then detect it back.

    Verifies the run-merging and the row orientation, not the dictionary.
    A vertical flip here is invisible on the page and fatal in the data.
    """
    cells = marker_cells(dictionary, marker_id)
    n = cells.shape[0]
    quiet = 3 * px_per_cell
    size = n * px_per_cell
    img = np.full((size + 2 * quiet, size + 2 * quiet), 255, np.uint8)
    for r, c0, width in black_runs(cells):
        y = quiet + r * px_per_cell
        x = quiet + c0 * px_per_cell
        img[y:y + px_per_cell, x:x + width * px_per_cell] = 0

    detector = cv2.aruco.ArucoDetector(dictionary)
    corners, ids, _rejected = detector.detectMarkers(img)
    if ids is None or int(ids.flatten()[0]) != marker_id:
        raise RuntimeError(
            "marker %d did not survive the render: detected %s"
            % (marker_id, None if ids is None else ids.flatten().tolist()))
    return True


# --------------------------------------------------------------------------
# Marker sheets
# --------------------------------------------------------------------------

def draw_scale_bar(c, x_mm, y_mm, length_mm=150.0, tick_mm=10.0):
    """An independent check that the page printed at 100 percent.

    The marker cannot check its own scale: if the page came out at 96 percent
    the marker is 96 mm and looks exactly like a 100 mm marker. This bar is
    a second thing to lay a rule against, and it disagrees with nothing.
    """
    c.setLineWidth(0.5)
    c.line(x_mm * mm, y_mm * mm, (x_mm + length_mm) * mm, y_mm * mm)
    n = int(round(length_mm / tick_mm))
    for i in range(n + 1):
        x = (x_mm + i * tick_mm) * mm
        h = (5.0 if i % 5 == 0 else 2.5) * mm
        c.line(x, y_mm * mm, x, y_mm * mm + h)
        if i % 5 == 0:
            c.setFont("Helvetica", 6)
            c.drawCentredString(x, y_mm * mm + h + 1.5 * mm, "%d" % (i * tick_mm))


def marker_sheets(path, dictionary):
    # invariant=1 fixes the embedded timestamp and document ID. Without it
    # every regeneration produces a different file for identical geometry, the
    # checksum in print/CHECKSUMS.txt means nothing, and there is no way to
    # ask "is the committed PDF the one this code produces?"
    c = canvas.Canvas(str(path), pagesize=A4, invariant=1)
    page_w = A4[0] / mm
    cell = MARKER_MM / (dictionary.markerSize + 2 * MARKER_BORDER_CELLS)

    for marker_id in MARKER_IDS:
        check_marker_roundtrip(dictionary, marker_id)
        cells = marker_cells(dictionary, marker_id)

        x0 = (page_w - MARKER_MM) / 2.0
        y0 = 150.0                      # from the foot of the page
        n = cells.shape[0]

        c.setFillGray(0.0)
        for r, c0, width in black_runs(cells):
            # row 0 of the grid is the TOP row; PDF y grows upward
            c.rect((x0 + c0 * cell) * mm,
                   (y0 + (n - 1 - r) * cell) * mm,
                   width * cell * mm, cell * mm, stroke=0, fill=1)

        c.setFont("Helvetica-Bold", 13)
        c.drawCentredString(page_w / 2 * mm, (y0 + MARKER_MM + 30) * mm,
                            "PRINT AT 100%  -  NO SCALING, NO FIT TO PAGE")
        c.setFont("Helvetica", 9)
        c.drawCentredString(page_w / 2 * mm, (y0 + MARKER_MM + 23) * mm,
                            "Matte paper. Flat and unbent. Do not laminate.")

        c.setFont("Helvetica", 9)
        c.drawCentredString(page_w / 2 * mm, (y0 - 12) * mm,
                            "ArUco DICT_4X4_50   ID %d   nominal %.4f mm outer edge to outer edge"
                            % (marker_id, MARKER_MM))
        c.setFont("Helvetica-Bold", 10)
        c.drawCentredString(page_w / 2 * mm, (y0 - 22) * mm,
                            "MEASURED WIDTH (calipers, mm):  ______________")
        c.setFont("Helvetica", 7.5)
        c.drawCentredString(page_w / 2 * mm, (y0 - 30) * mm,
                            "Measure the black square, outer edge to outer edge. "
                            "Every range estimate divides by this number, not by 100.")

        draw_scale_bar(c, (page_w - 150.0) / 2.0, 55.0)
        c.setFont("Helvetica", 7.5)
        c.drawCentredString(page_w / 2 * mm, 45 * mm,
                            "Scale check: the bar above is 150.0000 mm end to end. "
                            "If it is not, the page did not print at 100%.")
        c.showPage()
    c.save()
    return len(MARKER_IDS)


# --------------------------------------------------------------------------
# ChArUco calibration board
# --------------------------------------------------------------------------

def charuco_board(path, dictionary):
    board = cv2.aruco.CharucoBoard(
        (BOARD_COLS, BOARD_ROWS), BOARD_SQUARE_MM, BOARD_MARKER_MM, dictionary)

    w_mm = BOARD_COLS * BOARD_SQUARE_MM
    h_mm = BOARD_ROWS * BOARD_SQUARE_MM
    px = lambda v: int(round(v / 25.4 * BOARD_DPI))
    img = board.generateImage((px(w_mm), px(h_mm)))

    # Detect it back before committing it to the page. A board that does not
    # detect at 1200 dpi on a clean render will not detect on paper either,
    # and this is the last point at which that is cheap to find out.
    detector = cv2.aruco.CharucoDetector(board)
    corners, ids, _mc, _mi = detector.detectBoard(img)
    expected = (BOARD_COLS - 1) * (BOARD_ROWS - 1)
    if ids is None or len(ids) != expected:
        raise RuntimeError("board render found %s of %d interior corners"
                           % (0 if ids is None else len(ids), expected))

    page = landscape(A4)
    c = canvas.Canvas(str(path), pagesize=page, invariant=1)
    pw, ph = page[0] / mm, page[1] / mm
    # 7 x 5 at 35 mm is 175 mm tall on a 210 mm page: 17.5 mm of margin, and the
    # header sat 3.5 mm from the paper edge where most printers clip it. 32 mm
    # gives 224 x 160 and 25 mm of margin all round. A board that prints intact
    # beats a marginally larger one that does not.
    x0, y0 = (pw - w_mm) / 2.0, 36.0

    png = Path(path).with_suffix(".board.png")
    cv2.imwrite(str(png), img)
    c.drawImage(ImageReader(str(png)), x0 * mm, y0 * mm, w_mm * mm, h_mm * mm)

    c.setFont("Helvetica-Bold", 11)
    c.drawCentredString(pw / 2 * mm, (y0 + h_mm + 3) * mm,
                        "PRINT AT 100%  -  NO SCALING, NO FIT TO PAGE")
    c.setFont("Helvetica", 8)
    c.drawCentredString(pw / 2 * mm, (y0 - 9) * mm,
                        "ChArUco  DICT_5X5_100   %d x %d squares   square %.4f mm   marker %.4f mm"
                        % (BOARD_COLS, BOARD_ROWS, BOARD_SQUARE_MM, BOARD_MARKER_MM))
    c.setFont("Helvetica-Bold", 9)
    c.drawCentredString(pw / 2 * mm, (y0 - 17) * mm,
                        "MEASURED SQUARE PITCH (calipers, mm, over 5 squares / 5):  ______________")
    c.setFont("Helvetica", 7.5)
    c.drawCentredString(pw / 2 * mm, (y0 - 24) * mm,
                        "Measure across five squares and divide. One square carries the "
                        "printer's error in full; five carries a fifth of it.")
    c.showPage()
    c.save()
    png.unlink()
    return expected


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="print")
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    n = marker_sheets(out / "aruco_4x4_100mm.pdf",
                      cv2.aruco.getPredefinedDictionary(MARKER_DICT))
    print("marker sheets     : %d pages, IDs %s, %.4f mm nominal"
          % (n, list(MARKER_IDS), MARKER_MM))

    k = charuco_board(out / "charuco_a4_landscape.pdf",
                      cv2.aruco.getPredefinedDictionary(BOARD_DICT))
    print("calibration board : %dx%d squares, %d interior corners, detected clean"
          % (BOARD_COLS, BOARD_ROWS, k))
    print("out               : %s" % out)


if __name__ == "__main__":
    main()
