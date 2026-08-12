# -*- coding: utf-8 -*-
"""处理 leg_01_V1.obj：四边面->三角面 -> 顶点聚类减面 -> 方向归一化 -> ASCII 预览。"""
import numpy as np
import sys

SRC = r'C:\Users\林~\Desktop\leg_01_v1_L1.123c4b75505f-0901-4ae7-a875-a46575417065\11539_leg_01_V1.obj'

# ---------- 读取 OBJ ----------
vs, fs = [], []
with open(SRC) as fh:
    for line in fh:
        if line.startswith('v '):
            vs.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '):
            fs.append([int(x.split('/')[0]) - 1 for x in line.split()[1:]])
V = np.array(vs)
print('verts', V.shape, 'faces', len(fs))

# ---------- 四边面 -> 两个三角面 ----------
quads = np.array(fs)
tris = np.vstack([quads[:, [0, 1, 2]], quads[:, [0, 2, 3]]])
print('tris', tris.shape)

# ---------- 顶点聚类减面（目标 ~2500 三角） ----------
CELL = 3.0  # 单元尺寸（模型长约 99）
cells = np.floor(V / CELL).astype(np.int64)
uniq, inv = np.unique(cells, axis=0, return_inverse=True)
merged = np.zeros((len(uniq), 3))
np.add.at(merged, inv, V)
merged /= np.bincount(inv)[:, None]
t = inv[tris]
keep = (t[:, 0] != t[:, 1]) & (t[:, 1] != t[:, 2]) & (t[:, 0] != t[:, 2])
t = t[keep]
t = np.unique(np.sort(t, axis=1), axis=0)  # 去重
print('merged verts', len(uniq), 'tris after dedup', len(t))

# 再用更大的单元降一次，控制到目标规模
while len(t) > 2800 and CELL < 6.0:
    CELL += 1.5
    cells = np.floor(V / CELL).astype(np.int64)
    uniq, inv = np.unique(cells, axis=0, return_inverse=True)
    merged = np.zeros((len(uniq), 3))
    np.add.at(merged, inv, V)
    merged /= np.bincount(inv)[:, None]
    t = inv[tris]
    keep = (t[:, 0] != t[:, 1]) & (t[:, 1] != t[:, 2]) & (t[:, 0] != t[:, 2])
    t = t[keep]
    t = np.unique(np.sort(t, axis=1), axis=0)
    print('  cell', CELL, '-> verts', len(uniq), 'tris', len(t))

# ---------- 方向归一化：模型 +Z 脚(小z)->髋(大z)；髋在场景原点，腿向下(-Y)，脚朝前(+Z) ----------
zmin, zmax = merged[:, 2].min(), merged[:, 2].max()
x0, y0 = merged[:, 0].mean(), merged[:, 1].mean()
SCALE = 150.0 / (zmax - zmin)        # 腿长约 -> 150 场景单位
M = np.empty_like(merged)
M[:, 0] = (merged[:, 0] - x0) * SCALE   # 模型 X -> 场景 X
M[:, 1] = (merged[:, 2] - zmax) * SCALE  # 模型 Z -> 场景 -Y（髋在 y=0，脚在 y=-150）
M[:, 2] = (y0 - merged[:, 1]) * SCALE   # 模型 Y -> 场景 Z（翻转使脚尖朝前 +Z）
print('scene bbox y:', M[:, 1].min(), M[:, 1].max(), 'z:', M[:, 2].min(), M[:, 2].max())

# ---------- ASCII 预览（正视图 x-y 与侧视图 z-y） ----------
def preview(proj, W, H, label):
    x = proj[0]; y = proj[1]
    xmin, xmax = x.min(), x.max()
    ymin, ymax = y.min(), y.max()
    gx = ((x - xmin) / (xmax - xmin) * (W - 1)).astype(int)
    gy = ((ymax - y) / (ymax - ymin) * (H - 1)).astype(int)
    grid = [[' '] * W for _ in range(H)]
    for i in range(len(gx)):
        grid[gy[i]][gx[i]] = '#'
    print('===', label, '===')
    for row in grid:
        print(''.join(row))

preview([M[:, 0], M[:, 1]], 60, 40, 'FRONT (x->right, y->up)')
preview([M[:, 2], M[:, 1]], 60, 40, 'SIDE (z->right, y->up)')

# ---------- 输出紧凑 JS ----------
np.save('/tmp/leg_v.npy', M)
np.save('/tmp/leg_f.npy', t)
print('saved /tmp/leg_v.npy /tmp/leg_f.npy')
