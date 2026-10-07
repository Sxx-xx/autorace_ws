import sys, struct, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

def load_stl(path):
    with open(path, 'rb') as f:
        data = f.read()
    if data[:5] == b'solid' and b'facet' in data[:300]:
        tris = []
        cur = []
        for line in data.decode(errors='ignore').splitlines():
            line = line.strip()
            if line.startswith('vertex'):
                cur.append([float(v) for v in line.split()[1:4]])
                if len(cur) == 3:
                    tris.append(cur); cur = []
        return np.array(tris)
    n = struct.unpack('<I', data[80:84])[0]
    arr = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
    return arr['v'].astype(float)

def render(paths, out, views=((30, -60), (20, 120), (-25, -60))):
    fig = plt.figure(figsize=(5 * len(views), 5 * len(paths)))
    for i, p in enumerate(paths):
        t = load_stl(p)
        lo, hi = t.reshape(-1, 3).min(0), t.reshape(-1, 3).max(0)
        c = (lo + hi) / 2; r = (hi - lo).max() / 2
        n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
        n /= np.linalg.norm(n, axis=1)[:, None] + 1e-9
        light = np.array([0.4, -0.6, 0.7]); light /= np.linalg.norm(light)
        shade = 0.35 + 0.65 * np.clip(n @ light, 0, 1)
        for j, (el, az) in enumerate(views):
            ax = fig.add_subplot(len(paths), len(views), i * len(views) + j + 1, projection='3d')
            col = Poly3DCollection(t, facecolors=plt.cm.Greys_r(shade * 0.8 + 0.1), edgecolor='none')
            ax.add_collection3d(col)
            ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
            ax.view_init(el, az); ax.set_box_aspect((1, 1, 1))
            ax.set_title(f'{p.split("/")[-1]}  el{el} az{az}', fontsize=8)
    plt.tight_layout(); plt.savefig(out, dpi=70); plt.close()

if __name__ == '__main__':
    render(sys.argv[2:], sys.argv[1])
