import numpy as np
from ctypes import *


class Point(Structure):
    "C module point struct"
    _fields_ = [("X", c_double),
                ("Y", c_double)]


so_file = "stagger\\c_lib\\lib_stagger.so"
lib = CDLL(so_file)
lib.intersect.restype = c_int
lib.intersect.argtypes = [POINTER(Point), c_int]

lib.interparc.restype = POINTER(POINTER(c_double))
lib.interparc.argtypes = [
    c_int, POINTER(c_double),
    POINTER(c_double), c_int
]
'''
lib.FVC.restype = POINTER(c_double)
lib.FVC.argtypes = [
    POINTER(Point),
    c_double,
    c_double,
    Point,
    Point,
    c_int,
    POINTER(c_double)
]'''

# Resampling with C integration


def resample(curve: np.ndarray, p: int = 360) -> np.ndarray:
    curve = curve[:p//2+1]
    px: list = curve[:, 0].tolist()
    py: list = curve[:, 1].tolist()
    px_p = (c_double * len(px))(*px)
    py_p = (c_double * len(py))(*py)
    new = lib.interparc(p, px_p, py_p, len(px))
    new1 = np.ndarray((p, 2))
    for x in range(p):
        new1[x, 0] = new[x][0]
        new1[x, 1] = new[x][1]
    return new1


def PCA(curve) -> tuple:
    ANC = (0, 0)
    mean_x = np.mean(curve[:, 0])
    mean_y = np.mean(curve[:, 1])
    COM = np.array((mean_x, mean_y))
    COVARI = np.cov([curve[:, 0], curve[:, 1]])
    try:
        w, v = np.linalg.eig(COVARI)
    except:
        return (-1)
    L_MAX = COVARI[0, 0]
    L_MIN = COVARI[1, 1]
    return [COM, ANC, v[0], v[1], L_MAX, L_MIN]

# intercetions calculation with C integration


def intersections(curve: np.ndarray) -> int:
    point_array = (Point * len(curve))()
    for i, (x, y) in enumerate(curve):
        point_array[i].X = x
        point_array[i].Y = y
    return lib.intersect(point_array, 360)

# FVC calculation - C implementation attempt (*ONLY* for *VERY* beefy computers)


def FVC_C(curve, Lmax, Lmin, Vmax, anc, com, p=360) -> np.ndarray:
    d = anc-com
    point_array = (Point * len(curve))()
    for i, (x, y) in enumerate(curve):
        point_array[i].X = x
        point_array[i].Y = y
    features_arr = (c_double*6)()
    re = lib.FVC(point_array, c_double(Lmax), c_double(Lmin), Point(
        d[0], d[1]), Point(Vmax[0], Vmax[1]), p, features_arr)
    res = np.array([re[0], re[1], re[2], re[3], re[4], re[5]])
    return res

# FVC calculation


def FVC(curve, Lmax, Lmin, Vmax, anc, com, p=360) -> np.ndarray:
    edges = np.array([curve[i] - curve[i-1] for i in np.arange(0, p-1)])
    f_1 = np.sum(np.linalg.norm(edges[:]))
    f_2 = np.sum(np.abs(np.cross(curve[:-1], curve[1:])))/2
    f_3 = Lmin / Lmax
    d = anc-com
    f_4 = np.linalg.norm(d)
    f_5 = np.arcsin(np.linalg.norm(np.cross((d/np.linalg.norm(d)), Vmax)))
    f_6 = intersections(curve)
    return np.array([f_1, f_2, f_3, f_4, f_5, f_6])

# Cosine simmilarity


def Sim(fi, fj) -> float:
    d = np.dot(fi, fj)/(np.linalg.norm(fi)*np.linalg.norm(fj))
    return float(d)
