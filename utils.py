import numpy as np
from scipy.interpolate import interp1d

def smooth_path(path, height, width):
    if len(path) < 3:
        return path
    path = np.array(path)
    x, y = path[:, 0], path[:, 1]
    t = np.linspace(0, 1, len(x))
    fx = interp1d(t, x, kind='linear')
    fy = interp1d(t, y, kind='linear')
    t_new = np.linspace(0, 1, len(x)*5)
    smooth_pts = []
    last_pt = None
    for ti in t_new:
        nx = int(round(fx(ti).item()))
        ny = int(round(fy(ti).item()))
        nx = min(max(nx, 0), height-1)
        ny = min(max(ny, 0), width-1)
        pt = (nx, ny)
        if pt != last_pt:
            smooth_pts.append(pt)
            last_pt = pt
    return smooth_pts
