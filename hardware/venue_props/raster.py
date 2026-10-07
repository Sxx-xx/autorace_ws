import sys, glob, os, numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render import load_stl  # render.py: STL 읽기
from PIL import Image, ImageDraw, ImageFont

def rot(el, az):
    a, e = np.radians(az), np.radians(el)
    Rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    Rx = np.array([[1, 0, 0], [0, np.cos(e), -np.sin(e)], [0, np.sin(e), np.cos(e)]])
    return Rx @ Rz

def render(tris, size=420, el=-65, az=-35, color=(200, 205, 215)):
    """Orthographic z-buffer render. Camera looks along -z after rotation."""
    R = rot(el, az)
    v = tris.reshape(-1, 3) @ R.T
    v = v.reshape(-1, 3, 3)
    lo, hi = v.reshape(-1, 3).min(0), v.reshape(-1, 3).max(0)
    c = (lo + hi) / 2; s = (size * 0.84) / max(hi[0] - lo[0], hi[1] - lo[1])
    p = (v - c) * s
    px = p[:, :, 0] + size / 2; py = size / 2 - p[:, :, 1]; pz = p[:, :, 2]
    n = np.cross(v[:, 1] - v[:, 0], v[:, 2] - v[:, 0])
    nn = np.linalg.norm(n, axis=1); keep = nn > 1e-9
    n = n[keep] / nn[keep, None]; px, py, pz = px[keep], py[keep], pz[keep]
    light = np.array([-0.4, 0.5, 0.75]); light /= np.linalg.norm(light)
    shade = 0.25 + 0.75 * np.abs(n @ light)
    zbuf = np.full((size, size), -np.inf); img = np.full((size, size, 3), 255, np.uint8)
    col = np.array(color, float)
    for i in range(len(px)):
        x, y, z = px[i], py[i], pz[i]
        x0, x1 = int(max(0, np.floor(x.min()))), int(min(size - 1, np.ceil(x.max())))
        y0, y1 = int(max(0, np.floor(y.min()))), int(min(size - 1, np.ceil(y.max())))
        if x1 < x0 or y1 < y0: continue
        X, Y = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
        d = (x[1] - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (y[1] - y[0])
        if abs(d) < 1e-9: continue
        w1 = ((X - x[0]) * (y[2] - y[0]) - (x[2] - x[0]) * (Y - y[0])) / d
        w2 = ((x[1] - x[0]) * (Y - y[0]) - (X - x[0]) * (y[1] - y[0])) / d
        w0 = 1 - w1 - w2
        inside = (w0 >= -1e-3) & (w1 >= -1e-3) & (w2 >= -1e-3)
        if not inside.any(): continue
        Z = w0 * z[0] + w1 * z[1] + w2 * z[2]
        sub = zbuf[y0:y1 + 1, x0:x1 + 1]
        upd = inside & (Z > sub)
        sub[upd] = Z[upd]
        img[y0:y1 + 1, x0:x1 + 1][upd] = np.clip(col * shade[i], 0, 255).astype(np.uint8)
    return Image.fromarray(img)

def grid(paths, out, cols=4, size=420):
    font = ImageFont.load_default()
    rows = (len(paths) + cols - 1) // cols
    sheet = Image.new('RGB', (cols * size, rows * (size + 26)), 'white')
    d = ImageDraw.Draw(sheet)
    for i, p in enumerate(paths):
        tris = load_stl(p)
        im = render(tris, size)
        x, y = (i % cols) * size, (i // cols) * (size + 26)
        sheet.paste(im, (x, y))
        bb = tris.reshape(-1, 3); dims = bb.max(0) - bb.min(0)
        d.text((x + 6, y + size + 4), '%s   %.0f x %.0f x %.0f mm' % (os.path.basename(p)[:-4], *dims), fill=(40, 40, 40), font=font)
        print('rendered', p, flush=True)
    sheet.save(out)

if __name__ == '__main__':
    grid(sys.argv[2:], sys.argv[1], cols=int(os.environ.get('COLS', 4)))
