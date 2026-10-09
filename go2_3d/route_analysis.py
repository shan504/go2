"""Cell connectivity and static inscribed-clearance checks; no ROS or writes."""
from collections import deque
import numpy as np


def connected(data,start,goal):
    if start is None or goal is None:
        return 'outside grid'
    if data.size>500_000:
        return 'not checked: grid exceeds 500000 cells'
    if not 0<=data[goal[1],goal[0]]<99:
        return 'NO: goal cell blocked'
    visited=np.zeros(data.shape,dtype=bool)
    visited[start[1],start[0]]=True
    pending=deque([start])
    while pending:
        x,y=pending.popleft()
        if (x,y)==goal:
            return 'YES (cell centers only; not a Nav2 plan)'
        for nx,ny in ((x-1,y),(x+1,y),(x,y-1),(x,y+1)):
            if (0<=nx<data.shape[1] and 0<=ny<data.shape[0] and
                    not visited[ny,nx] and 0<=data[ny,nx]<99):
                visited[ny,nx]=True;pending.append((nx,ny))
    return 'NO: disconnected at cell-center level'


def inscribed_radius(footprint,padding):
    points=np.asarray(footprint,dtype=float)
    points=points+np.sign(points)*padding
    ends=np.roll(points,-1,axis=0)
    edge=ends-points
    lengths=np.linalg.norm(edge,axis=1)
    if len(points)<3 or np.any(lengths<=0):
        raise ValueError('Invalid footprint')
    return float(np.min(np.abs(points[:,0]*ends[:,1]-points[:,1]*ends[:,0])/lengths))


def static_clearance(data,resolution,radius):
    """Predict only the static inscribed/lethal band, retaining unknown space.

    Soft inflation below cost 99 cannot disconnect a NavFn cell route. This
    separates static body clearance from extra live global obstacle marks.
    It is not a full rectangular-footprint trajectory check.
    """
    result=data.copy()
    rows,cols=np.nonzero(data>=99)
    size=int(np.ceil(radius/resolution))
    for dy in range(-size,size+1):
        for dx in range(-size,size+1):
            if np.hypot(dx,dy)*resolution>radius+1e-8:
                continue
            y,x=rows+dy,cols+dx
            good=(y>=0)&(y<data.shape[0])&(x>=0)&(x<data.shape[1])
            result[y[good],x[good]]=99
    return result
