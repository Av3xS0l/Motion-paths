import numpy as np
import random
import stagger as st
import math
import time


class attempt(object):
    def __init__(self, id, o_params, params=[None]*13, ini=False):
        self.id = id
        self.o_params = o_params
        self.params = params
        self.fit = -2
        suc = True
        
        while suc:
            try:
                system = self.create_system(random=ini)
                self.sys = system
                suc = False
            except (ValueError, ZeroDivisionError) as e:
                if ini == False:
                    suc = False
                    self.fit = -1
        if self.fit == -2:
            try:
                self.aprox = self.make_aprox()
                if self.aprox == (-1):
                    raise ValueError
                if self.aprox[1][5] != self.o_params[1][5]:
                    self.fit = -1
                else:
                    self.fit = st.Sim(self.o_params[1], self.aprox[1])
            except ValueError as e:
                self.fit = -1

    def create_system(self, random=False) -> list: 
        if random:
            self.params = self.genarate_params()
        bl_1, bj_1, bl_2, x_1, y_1, r_1, s_1, i_1, x_2, y_2, r_2, s_2, i_2 = tuple(
            self.params)
        self.bar1 = st.Bar(bl_1, bj_1)
        self.bar2 = st.Bar(bl_2)
        self.drive1 = st.Anchor(x_1, y_1, r_1, s_1, i_1)
        self.drive2 = st.Anchor(x_2, y_2, r_2, s_2, i_2)
        self.motionSystem = st.TwoBar(
            self.drive1, self.drive2, self.bar1, self.bar2)
        inputRange = list(
            map((lambda x: x * self.motionSystem.stepSize), np.arange(0, 360)))
        return list(map(self.motionSystem.end_path, inputRange))

    def genarate_params(self) -> list:
        x_1 = 0
        y_1 = 0
        r_1 = random.randint(1, 10)
        r_2 = random.randint(1, 10)
        s_1 = 1
        i_1 = random.randint(0, 359)
        s_2 = 2
        i_2 = random.randint(0, 359)
        neder = True
        while neder:
            x_2 = random.randint(0, 60)
            y_2 = random.randint(0, 60)
            if math.dist((x_1, y_1), (x_2, y_2)) >= r_1+r_2:
                neder = False
        bl_1 = random.randint(15, 80)
        bj_1 = random.randint(15, bl_1)
        bl_2 = random.randint(5, 60)
        return [bl_1, bj_1, bl_2, x_1, y_1, r_1, s_1, i_1, x_2, y_2, r_2, s_2, i_2]

    def make_aprox(self) -> tuple:
        cj = st.resample(np.array(self.sys))
        _pca = st.PCA(cj)
        if _pca == (-1):
            return (-1)
        fj= st.FVC(cj, _pca[4], _pca[5], _pca[2], _pca[1], _pca[0])
        return (cj, fj)
