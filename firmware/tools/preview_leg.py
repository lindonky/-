# -*- coding: utf-8 -*-
"""把减面后的腿模渲染成 PNG（前/侧视图），供确认方向。"""
import numpy as np
from PIL import Image, ImageDraw

V = np.load('/tmp/leg_v.npy')   # 场景坐标：腿沿 -Y（髋在 y=0）
F = np.load('/tmp/leg_f.npy')

def render(proj_x, proj_y, path, W=220, H=320, flip_x=False):
    x = proj_x; y = proj_y
    xmin, xmax = x.min(), x.max(); ymin, ymax = y.min(), y.max()
    S = min((W-20)/(xmax-xmin), (H-20)/(ymax-ymin))
    px = (x - xmin) * S + 10
    py = (ymax - y) * S + 10      # y 向上
    if flip_x:
        px = W - px
    img = Image.new('RGB', (W, H), (15, 18, 32))
    d = ImageDraw.Draw(img)
    # 按深度排序画三角（painter）
    zc = (V[F[:,0],0] + V[F[:,1],0] + V[F[:,2],0]) / 3
    order = np.argsort(zc)[::-1]
    for fi in order:
        t = F[fi]
        pts = [(px[t[0]], py[t[0]]), (px[t[1]], py[t[1]]), (px[t[2]], py[t[2]])]
        d.polygon(pts, fill=(56, 132, 255))
    img.save(path)
    print('saved', path, 'x', (xmin,xmax), 'y', (ymin,ymax))

render(V[:,0], V[:,1], '/tmp/leg_front.png')
render(V[:,2], V[:,1], '/tmp/leg_side.png')
render(V[:,0], V[:,1], '/tmp/leg_back.png', flip_x=True)
