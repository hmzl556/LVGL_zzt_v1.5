from PIL import Image, ImageDraw, ImageFont
import os
import math

OUT_DIR = r"F:\A_ZZT_Project\Xiaoya_Xiyiji\LVGL_zzt_v1.2\lv_port_pc_vscode-master\image\AI_create_image"
W = H = 306
DPI = 150

labels = [
    "快洗","速洗","羽绒服","羊毛","冷水洗","大件","单脱水",
    "筒自洁","漂脱","节能","棉麻","混合洗","AI智洗"
]

# 13 example color pairs for ring gradients (start, end)
color_pairs = [
    ((255, 99, 71), (255, 160, 122)),     # tomato
    ((255, 140, 0), (255, 206, 140)),     # darkorange
    ((255, 182, 193), (255,240,245)),     # pink
    ((135,206,235), (176,226,255)),       # skyblue
    ((30,144,255), (135,206,250)),        # dodgerblue
    ((34,139,34), (144,238,144)),         # forestgreen
    ((147,112,219),(216,191,216)),        # mediumpurple
    ((255,215,0),(255,239,140)),          # gold
    ((72,61,139),(123,104,238)),          # darkslateblue
    ((0,206,209),(64,224,208)),           # turquoise
    ((199,21,133),(255,182,193)),         # mediumvioletred
    ((244,164,96),(255,218,185)),         # sandybrown
    ((70,130,180),(176,196,222))          # steelblue
]

os.makedirs(OUT_DIR, exist_ok=True)

# Try to load a Chinese-capable font from common Windows locations
font_paths = [
    r"C:\Windows\Fonts\msyh.ttc",
    r"C:\Windows\Fonts\msyh.ttf",
    r"C:\Windows\Fonts\simhei.ttf",
    r"C:\Windows\Fonts\simsun.ttc",
    r"/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
]
font = None
for p in font_paths:
    try:
        font = ImageFont.truetype(p, 36)
        break
    except Exception:
        font = None

if font is None:
    # fallback to default (may not render Chinese correctly)
    font = ImageFont.load_default()


def lerp(a, b, t):
    return tuple(int(a[i] + (b[i] - a[i]) * t) for i in range(3))


def draw_gradient_ring(draw, center, inner_r, outer_r, start_col, end_col):
    # Draw by concentric ellipses from outer_r down to inner_r
    cx, cy = center
    steps = outer_r - inner_r
    if steps <= 0:
        steps = 1
    for i in range(steps):
        t = i / (steps - 1) if steps > 1 else 0
        col = lerp(start_col, end_col, t)
        bbox = [cx - (inner_r + i), cy - (inner_r + i), cx + (inner_r + i), cy + (inner_r + i)]
        draw.ellipse(bbox, outline=col + (255,))


def draw_icon(draw, center, label, scale=1.0):
    cx, cy = center
    # Draw simple stylized icons depending on keyword
    # Icons are intentionally minimal to keep consistent style
    if "羽" in label or "羽绒" in label:
        # Feather: a curved line with barbs
        for s in range(6):
            angle = math.pi/2 - 0.6 + s*0.24
            x = cx + math.cos(angle)*50*scale
            y = cy - 10 + math.sin(angle)*50*scale
            draw.line([cx, cy, x, y], fill=(255,255,255,220), width=4)
        draw.arc([cx-40*scale,cy-60*scale,cx+40*scale,cy+60*scale], 20, 200, fill=(255,255,255,230), width=4)
    elif "羊毛" in label or "棉" in label:
        # Wool/cotton: circle with small arcs
        draw.ellipse([cx-36*scale, cy-36*scale, cx+36*scale, cy+36*scale], outline=(255,255,255,230), width=4)
        for a in range(6):
            ang = a * math.pi/3
            x1 = cx + math.cos(ang)*36*scale
            y1 = cy + math.sin(ang)*36*scale
            x2 = cx + math.cos(ang)*50*scale
            y2 = cy + math.sin(ang)*50*scale
            draw.line([x1,y1,x2,y2], fill=(255,255,255,200), width=3)
    elif "冷" in label or "冷水" in label:
        # Snowflake
        for a in range(6):
            ang = a * math.pi/3
            x1 = cx + math.cos(ang)*10*scale
            y1 = cy + math.sin(ang)*10*scale
            x2 = cx + math.cos(ang)*40*scale
            y2 = cy + math.sin(ang)*40*scale
            draw.line([x1,y1,x2,y2], fill=(255,255,255,230), width=4)
    elif "大件" in label:
        # Box
        draw.rectangle([cx-36*scale, cy-28*scale, cx+36*scale, cy+28*scale], outline=(255,255,255,230), width=4)
        draw.line([cx-36*scale,cy-12*scale,cx+36*scale,cy-12*scale], fill=(255,255,255,200), width=3)
    elif "脱水" in label or "单脱水" in label or "漂脱" in label:
        # Spinner/arrow
        draw.pieslice([cx-44*scale,cy-44*scale,cx+44*scale,cy+44*scale], -30, 90, outline=(255,255,255,230), width=4)
        draw.polygon([ (cx+35*scale,cy-5*scale),(cx+50*scale,cy-5*scale),(cx+35*scale,cy+10*scale) ], fill=(255,255,255,230))
    elif "筒自洁" in label:
        # Tub with sparkles
        draw.ellipse([cx-40*scale, cy-18*scale, cx+40*scale, cy+18*scale], outline=(255,255,255,230), width=4)
        # sparkles
        for dx,dy in [(-20,-20),(10,-15),(22,-5)]:
            draw.line([cx+dx,cy+dy-6, cx+dx, cy+dy+6], fill=(255,255,255,230), width=2)
            draw.line([cx+dx-6,cy+dy, cx+dx+6, cy+dy], fill=(255,255,255,230), width=2)
    elif "节能" in label:
        # Leaf
        draw.polygon([ (cx,cy-36*scale),(cx+22*scale,cy),(cx,cy+36*scale),(cx-22*scale,cy) ], outline=(255,255,255,230), fill=(255,255,255,40))
    elif "混合" in label:
        # overlapping circles
        draw.ellipse([cx-40*scale-10,cy-36*scale, cx-10,cy+36*scale], outline=(255,255,255,230), width=4)
        draw.ellipse([cx+10,cy-36*scale, cx+40*scale+10,cy+36*scale], outline=(255,255,255,230), width=4)
    elif "AI" in label:
        # AI letters
        f = ImageFont.truetype(font.path, 48) if hasattr(font, 'path') else font
        draw.text((cx-28, cy-28), "AI", font=f, fill=(255,255,255,240))
    else:
        # generic washing symbol: circle and wave
        draw.ellipse([cx-36*scale, cy-36*scale, cx+36*scale, cy+36*scale], outline=(255,255,255,230), width=4)
        draw.line([cx-30*scale,cy+10*scale,cx-10*scale,cy+10*scale,cx+10*scale,cy+6*scale,cx+30*scale,cy+6*scale], fill=(255,255,255,220), width=4)


for idx, label in enumerate(labels):
    img = Image.new('RGBA', (W, H), (0,0,0,0))
    draw = ImageDraw.Draw(img)
    cx = W//2
    cy = H//2 - 8

    start_col, end_col = color_pairs[idx % len(color_pairs)]

    # Draw gradient ring
    # Match AI_zhixi.png: outer diameter == image size (306), so outer_r = 153
    # Measured thickness in AI_zhixi.png ~151 pixels, so inner_r = outer_r - 151 = 2
    # Outer diameter = 306 -> nominal outer_r = 153
    nominal_outer_r = W // 2
    # Inner diameter = 290 -> nominal inner_r = 145
    nominal_inner_r = 145
    # Scale ring size by factor (e.g., 0.95 means 95%)
    SCALE = 0.95
    outer_r = int(nominal_outer_r * SCALE)
    inner_r = int(nominal_inner_r * SCALE)
    draw_gradient_ring(draw, (cx, cy), inner_r=inner_r, outer_r=outer_r, start_col=start_col, end_col=end_col)

    # Draw semi-transparent filled circle as icon background (slightly darker)
    draw.ellipse([cx-58, cy-58, cx+58, cy+58], fill=(0,0,0,0))

    # Draw icon in white
    draw_icon(draw, (cx, cy-18), label, scale=1.0)

    # Draw text centered
    # use font (already loaded) but adjust size if needed
    txt = label
    # try to use a font size that fits
    fnt = font
    # If truetype, adjust size
    if isinstance(fnt, ImageFont.FreeTypeFont):
        # create appropriate sized font for fitting
        for size in (36,32,28,24):
            try:
                ftest = ImageFont.truetype(fnt.path, size)
                try:
                    wtxt, htxt = ftest.getsize(txt)
                except Exception:
                    wtxt, htxt = draw.textbbox((0,0), txt, font=ftest)[2:]
                if wtxt <= W-20:
                    fnt = ftest
                    break
            except Exception:
                pass
        # ensure we have measured text size with the chosen font
        try:
            wtxt, htxt = fnt.getsize(txt)
        except Exception:
            box = draw.textbbox((0,0), txt, font=fnt)
            wtxt, htxt = box[2]-box[0], box[3]-box[1]
    else:
        try:
            wtxt, htxt = fnt.getsize(txt)
        except Exception:
            box = draw.textbbox((0,0), txt, font=fnt)
            wtxt, htxt = box[2]-box[0], box[3]-box[1]

    draw.text((cx - wtxt/2, cy+48), txt, font=fnt, fill=(255,255,255,240))

    out_path = os.path.join(OUT_DIR, f"{idx+1:02d}_{txt}.png")
    img.save(out_path, dpi=(DPI, DPI))
    print('Saved', out_path)

print('Done')
