import os
import random
from datetime import datetime as dt
from multiprocessing.pool import Pool
from time import perf_counter

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
from tqdm import tqdm

import stagger as st
import stagger.attempt as attempt


class main():
    def __init__(self) -> None:
        self.TEST_ID = str(dt.now())
        self.TEST_ID = self.TEST_ID.replace(":", "-")
        self.TEST_ID = self.TEST_ID.replace(" ", "-")
        self.TEST_ID = self.TEST_ID.replace(".", "-")
        print(self.TEST_ID)
        st.log(self.TEST_ID, "ID", "Error", "Time elapsed")
        self.PREF_START = dt.now()

        self.gen_size = 10000  # 5000
        self.good = 128  # 128
        self.mutate_prob = 60
        self.cross_prob = 80
        self.POOL = 6
        self.best_fit = -1
        self.prev_best_fits = []
        self.o_cords = self.get_cords()
        self.CURVE = st.resample(self.o_cords)
        self.pca = st.PCA(self.CURVE)
        # print(self.CURVE)
        self.fi = st.FVC(
            self.CURVE, self.pca[4], self.pca[5], self.pca[2], self.pca[1], self.pca[0])
        self.parse = (self.CURVE, self.fi)
        # print(self.parse[1])
        self.Population = []

        self.prev_pop = []
        self.prev_time = self.PREF_START
        self.initial_pop()
        print("Initialized")
        self.diference = dt.now()-self.prev_time
        print(self.diference)
        self.prev_time = dt.now()
        self.converge = False
        self.counter = 1

        while not self.converge:
            self.diference = dt.now()-self.prev_time
            self.prev_time = dt.now()
            self.selection()
            print(self.counter, self.best_fit, self.diference)
            st.log(self.TEST_ID, self.counter, self.best_fit, self.diference)
            self.next_gen()
            self.counter += 1
            if self.counter > 3:
                self.test_converge()

        self.Population.sort(key=lambda test: test.fit, reverse=True)
        st.save(self.Population[0].id, self.Population[0].sys)
        st.save_params(
            self.Population[0].id, self.Population[0].params, self.Population[0].fit)
        print(
            f"Convergence in {self.counter} genereation\nAvg. distance: {self.best_fit}\nTotal time elapsed: {dt.now()-self.PREF_START}")
        st.log(self.TEST_ID, self.counter,
               self.best_fit, dt.now()-self.PREF_START)
        return

    def load_image(self) -> list:
        cords = []
        try:
            img = Image.open(f"outputs/base.png")
        except FileNotFoundError as e:
            print(f"{e}: Try running GeneratePath.py")
            exit()
        pixels = img.load()
        width, height = img.size
        for x in np.arange(width):
            for y in np.arange(height):
                v = pixels[x, y]
                if v != 255:
                    cords.append((x, y))
        return cords

    def cross(self, first: attempt.attempt, second: attempt.attempt, i):
        o_1 = first.params
        o_2 = second.params
        if random.randint(1, 100) < self.cross_prob:
            crosover = random.randint(0, 12)
            f_1 = [a for a in o_1]
            f_2 = [a for a in o_2]

            for iz in np.arange(crosover):
                f_1[iz] = o_2[iz]
                f_2[iz] = o_1[iz]
            return attempt.attempt(f"{self.counter}_{i}", self.parse, f_1), attempt.attempt(f"{self.counter}_{i+1}", self.parse, f_2)
        return attempt.attempt(f"{self.counter}_{i}", self.parse, o_1), attempt.attempt(f"{self.counter}_{i+1}", self.parse, o_2)

    def itt(self, i) -> attempt.attempt:
        return attempt.attempt(f"0_{i}", self.parse, ini=True)

    def initial_pop(self):
        iter = tqdm(np.arange(self.gen_size))

        pool = Pool(self.POOL)
        for result in pool.map(self.itt, iter):
            self.Population.append(result)
        # FOR SINGLE PROCESS:
        # for result in iter:
        #     r = self.itt(result)
        #     st.save(r.id, r.sys, "many/")
        #     self.Population.append(r)

    def selection(self):
        self.prev_pop = []
        self.Population.sort(key=lambda test: test.fit, reverse=True)

        self.best_fit = self.Population[0].fit
        self.prev_best_fits.append(self.best_fit)

        for i in range(self.good):
            self.prev_pop.append(self.Population[i])

    def mutate(self, element):

        for i in np.arange(13):
            chance = random.randint(1, 100)
            borders = [
                (15, 80),
                (15, element.params[0]),
                (5, 60),
                (0, 0),
                (0, 0),
                (1, 10),
                (1, 1),
                (0, 359),
                (0, 60),
                (0, 60),
                (1, 10),
                (2, 2),
                (0, 359),
            ]
            lB, hB = borders[i]
            if chance < self.mutate_prob:
                amount = random.randint(-10, 10)
                if element.params[i] + amount > lB and element.params[i] + amount < hB:
                    element.params[i] += amount

    def new_gen(self, i):
        parents = random.sample(self.prev_pop, k=2)
        child = self.cross(parents[0], parents[1], i)

        for n in np.arange(2):
            mutation = random.randint(0, 99)
            if mutation > 40:
                self.mutate(child[n])
        return (child[0], child[1])

    def next_gen(self):
        self.Population = []

        iter = tqdm(np.arange(int((self.gen_size)//2)))
        pool = Pool(self.POOL)
        for result in pool.map(self.new_gen, iter):
            self.Population.append(result[0])
            self.Population.append(result[1])
        # FOR SINGLE PROCESS
        # for r in iter:
        #     result = self.new_gen(r)
        #     self.Population.append(result[0])
        #     self.Population.append(result[1])

    def test_converge(self):
        if self.prev_best_fits[-1] > 0.9999999999000:
            self.converge = True

    def get_cords(self, add: str = ""):
        return np.genfromtxt(f"{os.getcwd()}\\outputs\\base{add}.cor", delimiter=",")


if __name__ == '__main__':
    root = main()
# Sākotnējās populācijas izveide
