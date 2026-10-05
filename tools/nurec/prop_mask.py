"""Propeller sweep mask (static, one PNG beside every image, white = keep). Usage: prop_mask.py <images dir>.
Propeller sweep mask: thin dark marks against the sky (the lower part of these upside-down frames), union over
all frames, grown by a few pixels. Textured ground is left alone by restricting the search to the sky side."""
import os, sys, numpy as np
from PIL import Image, ImageFilter
d = sys.argv[1]; names = sorted(n for n in os.listdir(d) if n.endswith(".png") and not n.endswith("_mask.png"))
W, H = 640, 400
union = np.zeros((H, W), bool); hits = np.zeros((H, W), np.int32)
for n in names:
    im = Image.open(os.path.join(d, n)).convert("L").resize((W, H), Image.BILINEAR)
    a = np.asarray(im, np.float32); bl = np.asarray(im.filter(ImageFilter.MedianFilter(21)), np.float32)
    dark = (bl - a) > 45                               # thin dark marks against a smooth background
    sky = bl > 110                                     # bright smooth background (sky side)
    sel = dark & sky; sel[: int(0.5 * H)] = False     # only the sky half (bottom half of these frames)
    hits += sel
union = hits >= 2                                      # seen in at least two frames
mask_small = Image.fromarray(((~union) * 255).astype(np.uint8)).filter(ImageFilter.MinFilter(9))
mask = mask_small.resize((1280, 800), Image.NEAREST).filter(ImageFilter.MinFilter(21))
m = np.asarray(mask); print("frames %d; excluded %.1f%% of pixels" % (len(names), 100 * (m < 128).mean()))
rows = (m < 128).mean(1); print("excluded by row band (8, top->bottom):", [round(float(rows[i*100:(i+1)*100].mean()), 2) for i in range(8)])
Image.fromarray(m).save("/tmp/prop_mask.png")
for n in names: Image.fromarray(m).save(os.path.join(d, os.path.splitext(n)[0] + "_mask.png"))
for k, idx in enumerate((len(names)//3, len(names)//2)):
    frame = np.asarray(Image.open(os.path.join(d, names[idx])).convert("RGB")).copy(); frame[m < 128] = (frame[m < 128] * 0.35 + np.array([255, 0, 0]) * 0.65).astype(np.uint8)
    Image.fromarray(frame).resize((640, 400)).save("/tmp/prop_mask_preview%d.jpg" % k, quality=85)
