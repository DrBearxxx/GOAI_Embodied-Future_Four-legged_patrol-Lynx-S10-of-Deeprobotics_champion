import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]

def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8-sig'))

def write(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    temp.replace(path)

def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(1 << 20), b''): h.update(block)
    return h.hexdigest()

def mat(p, q):
    p, q = np.asarray(p, float), np.asarray(q, float)
    if p.shape != (3,) or q.shape != (4,) or not np.isfinite(np.r_[p,q]).all() or abs(np.linalg.norm(q)-1) > .05:
        raise ValueError('Invalid position/quaternion')
    T = np.eye(4); T[:3,:3] = Rotation.from_quat(q).as_matrix(); T[:3,3] = p
    return T

def inv(T):
    U = np.eye(4); U[:3,:3] = T[:3,:3].T; U[:3,3] = -U[:3,:3] @ T[:3,3]
    return U

def apply(T, p): return np.asarray(p) @ T[:3,:3].T + T[:3,3]
def delta(A, B):
    return float(np.linalg.norm(A[:3,3]-B[:3,3])), float(np.rad2deg(Rotation.from_matrix(A[:3,:3].T @ B[:3,:3]).magnitude()))
def wrap(a): return float(np.arctan2(np.sin(a), np.cos(a)))
def yaw(T): return float(np.arctan2(T[1,0], T[0,0]))
def voxel(p, resolution):
    p = np.asarray(p); p = p[np.isfinite(p).all(axis=1)]
    if not len(p): return p
    _, ix = np.unique(np.floor(p[:,:3] / resolution).astype(np.int64), axis=0, return_index=True)
    return p[np.sort(ix)]
def cap(p, n): return p if len(p) <= n else p[np.linspace(0, len(p)-1, n, dtype=int)]
def stats(values):
    a = np.asarray(values, float); a = a[np.isfinite(a)]
    return dict(n=len(a), median=float(np.median(a)), p95=float(np.quantile(a,.95)), max=float(a.max())) if len(a) else dict(n=0)

def lidar_extrinsic(side):
    T = np.eye(4); T[:3,:3] = Rotation.from_euler('xyz',[0,-1.5707,-3.14159 if side=='front' else 0]).as_matrix()
    T[:3,3] = [.22341 if side=='front' else -.22341,0,-.0001]
    return T
