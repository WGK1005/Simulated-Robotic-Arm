#!/usr/bin/env python3
"""
make_checkerboard.py - generate a print-ready camera calibration checkerboard.

The image is rendered at a fixed DPI so that, printed at 100% scale (no
"fit to page"), the squares come out at the requested size in mm.

Printers rarely print at exactly 100%: a nominal 100 mm line often comes out
96 mm, and the X and Y axes are usually scaled by slightly different amounts.
Pass the measured print scale back in with --printer-scale-x / -y and the
generator pre-compensates, so the printed squares really are square.

Workflow
  1. generate with defaults, print at 100%, measure the ruler AND the board
     edges (measure across all squares, not one - far more accurate)
  2. printer_scale_x = measured_width_mm / nominal_width_mm
     printer_scale_y = measured_height_mm / nominal_height_mm
  3. regenerate with those two values and print again
  4. measure once more and pass the real square size to the calibrator

Usage
  python3 make_checkerboard.py                                   # plain A4
  python3 make_checkerboard.py --printer-scale-x 0.96 \
                               --printer-scale-y 0.95            # compensated
  python3 make_checkerboard.py --cols 7 --rows 5 --square-mm 25 --paper a3

Printing
  * print at 100% / "actual size" - never "fit to page"
  * glue onto a FLAT rigid board (glass, acrylic, MDF); a wavy sheet ruins it
  * finally measure the real square size and pass THAT to the calibrator:
      --square 0.01997   (not 0.02)
"""

import argparse

from PIL import Image, ImageDraw, ImageFont

PAPERS_MM = {"a4": (210.0, 297.0), "a3": (297.0, 420.0), "letter": (215.9, 279.4)}

FONT_CANDIDATES = [
    r"C:\Windows\Fonts\arial.ttf",
    r"C:\Windows\Fonts\segoeui.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
]


def load_font(size):
    for path in FONT_CANDIDATES:
        try:
            return ImageFont.truetype(path, size)
        except Exception:
            continue
    return ImageFont.load_default()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cols", type=int, default=8,
                    help="inner corners along X (default 8 -> 9 squares)")
    ap.add_argument("--rows", type=int, default=6,
                    help="inner corners along Y (default 6 -> 7 squares)")
    ap.add_argument("--square-mm", type=float, default=20.0,
                    help="wanted printed square size in mm (default 20)")
    ap.add_argument("--paper", default="a4", choices=sorted(PAPERS_MM))
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--margin-mm", type=float, default=10.0,
                    help="minimum margin around the board area (default 10)")
    ap.add_argument("--printer-scale-x", type=float, default=1.0,
                    help="measured/nominal for the X axis (0.96 means the "
                         "printer shrinks X by 4%%)")
    ap.add_argument("--printer-scale-y", type=float, default=1.0,
                    help="measured/nominal for the Y axis")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    sx, sy = args.printer_scale_x, args.printer_scale_y
    if sx <= 0 or sy <= 0:
        raise SystemExit("printer scales must be positive")

    base_pxmm = args.dpi / 25.4
    # pixels per mm *after* compensation: divide by the printer scale so the
    # shrink on paper cancels out
    pxmm_x = base_pxmm / sx
    pxmm_y = base_pxmm / sy

    paper_w_mm, paper_h_mm = PAPERS_MM[args.paper]
    W = int(round(paper_w_mm * base_pxmm))
    H = int(round(paper_h_mm * base_pxmm))

    sq = args.square_mm
    sq_x_mm = sq / sx          # rendered size on paper
    sq_y_mm = sq / sy
    n_x = args.cols + 1
    n_y = args.rows + 1
    board_w_mm = n_x * sq_x_mm
    board_h_mm = n_y * sq_y_mm

    if board_w_mm + 2 * args.margin_mm > paper_w_mm:
        need = (paper_w_mm - 2 * args.margin_mm) / n_x * sx
        raise SystemExit(
            "board would be %.1f mm wide but %s is only %.1f mm.\n"
            "Reduce --square-mm to about %.2f or use --paper a3"
            % (board_w_mm, args.paper.upper(), paper_w_mm, need))
    if board_h_mm + 2 * args.margin_mm + 35 > paper_h_mm:
        raise SystemExit("board would be too tall for %s" % args.paper.upper())

    img = Image.new("L", (W, H), 255)
    draw = ImageDraw.Draw(img)

    board_w_px = int(round(board_w_mm * base_pxmm))
    board_h_px = int(round(board_h_mm * base_pxmm))
    x0 = (W - board_w_px) // 2
    y0 = int(round(35 * base_pxmm))

    for r in range(n_y):
        for c in range(n_x):
            if (r + c) % 2 == 0:
                xa = x0 + int(round(c * sq_x_mm * base_pxmm))
                ya = y0 + int(round(r * sq_y_mm * base_pxmm))
                xb = x0 + int(round((c + 1) * sq_x_mm * base_pxmm))
                yb = y0 + int(round((r + 1) * sq_y_mm * base_pxmm))
                draw.rectangle([xa, ya, xb - 1, yb - 1], fill=0)

    # ---- 100 mm scale check (also compensated) -----------------------------
    ruler_y = y0 + board_h_px + int(round(15 * base_pxmm))
    ruler_x0 = (W - int(round(100 * base_pxmm))) // 2
    tick = int(round(4 * base_pxmm))
    lw = max(2, int(base_pxmm))
    draw.line([ruler_x0, ruler_y, ruler_x0 + int(round(100 * base_pxmm)), ruler_y],
              fill=0, width=lw)
    font = load_font(int(round(3.4 * base_pxmm)))
    for i in range(0, 11):
        tx = ruler_x0 + int(round(i * 10 * base_pxmm))
        long_tick = tick if i % 5 == 0 else tick // 2
        draw.line([tx, ruler_y - long_tick, tx, ruler_y], fill=0, width=lw)
        if i % 5 == 0:
            draw.text((tx + 4, ruler_y + 4), str(i * 10), fill=0, font=font)
    draw.text((ruler_x0, ruler_y + int(round(9 * base_pxmm))),
              "MEASURE THIS LINE - must be exactly 100.0 mm", fill=0, font=font)

    # ---- info block --------------------------------------------------------
    info_y = ruler_y + int(round(18 * base_pxmm))
    lines = [
        "Camera calibration checkerboard  -  print at 100% (actual size)",
        "inner corners %dx%d   target square %.2f mm" % (args.cols, args.rows, sq),
        "compensation  x / %.4f   y / %.4f" % (sx, sy),
        "sheet %s   %d dpi" % (args.paper.upper(), args.dpi),
        "",
        "1) verify the 100 mm ruler with calipers",
        "2) glue onto a FLAT rigid board",
        "3) measure the REAL square size and give it to the calibrator:",
        "   --size %dx%d --square <measured_m>" % (args.cols, args.rows),
    ]
    for i, text in enumerate(lines):
        draw.text((ruler_x0, info_y + i * int(round(4.6 * base_pxmm))), text,
                  fill=0, font=font)

    out = args.out or ("checkerboard_%s_%dx%d_%gmm%s.png"
                       % (args.paper, args.cols, args.rows, sq,
                          "" if (sx == 1.0 and sy == 1.0) else "_comp"))
    img.save(out, dpi=(args.dpi, args.dpi))

    print("written:", out)
    print("  image         : %d x %d px @ %d dpi (%.0f x %.0f mm)"
          % (W, H, args.dpi, paper_w_mm, paper_h_mm))
    print("  pattern       : %dx%d inner corners" % (args.cols, args.rows))
    print("  rendered grid : %.3f x %.3f mm per square on the sheet"
          % (sq_x_mm, sq_y_mm))
    print("  expected      : %.2f x %.2f mm per square after printing"
          % (sq, sq))
    print("  board on sheet: %.1f x %.1f mm" % (board_w_mm, board_h_mm))
    print("  verify        : the 100 mm ruler and the board edges")
    print("  use with      : --size %dx%d --square %.5f (re-measure first!)"
          % (args.cols, args.rows, sq / 1000.0))


if __name__ == "__main__":
    main()
