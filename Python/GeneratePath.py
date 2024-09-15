# -*- coding: utf-8 -*-

import random
import numpy as np
import stagger as st
from PIL import Image, ImageDraw, ImageOps


class GeneratePath(object):

    def __init__(self):
        sucsess = True
        while sucsess:
            try:
                self.system = self.create_system()
                
                self.save_png('outputs/base.png', self.system, 10)
                with open("outputs/base.par", "w") as f:
                    for i in self.params:
                        f.write(str(i)+" ")
                sucsess = False
                st.save_cords(self.system)

            except (ValueError, ZeroDivisionError) as e:
                pass
        
        
    def create_system(self, bl_1=30, bj_1=15, bl_2=20, x_1=-10, y_1=-20, r_1=5, s_1=1, i_1=0, x_2=10, y_2=-20, r_2=6, s_2=2, i_2=180) -> list:
        self.params = self.genarate_params()
        bl_1, bj_1, bl_2, x_1, y_1, r_1, s_1, i_1, x_2, y_2, r_2, s_2, i_2 = tuple(
            self.params)
        # Defaults:
        # Bar 1 - bl_1= 60, bj_1= 30;
        # Bar 2 - bl_2= 30;
        # Drive 1 - x_1= -20, y_1= -20, r_1= 5, s_1= 2, i_1= 0;
        # Drive 1 - x_2=  20, y_2= -20, r_2= 6, s_2= 3, i_2= 180;
        self.bar1 = st.Bar(bl_1, bj_1)
        self.bar2 = st.Bar(bl_2)

        # (x, y, r, speed = 1, initial = 0)
        self.drive1 = st.Anchor(x_1, y_1, r_1, s_1, i_1)
        self.drive2 = st.Anchor(x_2, y_2, r_2, s_2, i_2)

        self.motionSystem = st.TwoBar(self.drive1, self.drive2, self.bar1,
                                           self.bar2)

        inputRange = list(
            map((lambda x: x * self.motionSystem.stepSize), range(0, 360)))
        return list(map(self.motionSystem.end_path, inputRange))

    def genarate_params(self) -> list:
        '''
        Funkcija ģenerē nejaušas sākuma vērtības\n
        Pašreizējās vērtības ir maināmas.
        '''
        x_1 = 0
        y_1 = 0
        r_1 = random.randint(1, 10)
        s_1 = 1
        i_1 = random.randint(0, 359)
        s_2 = 3
        i_2 = random.randint(0, 359)
        bl_1 = random.randint(15, 80)
        bj_1 = random.randint(15, bl_1)
        r_2 = random.randint(1, 10)
        x_2 = random.randint((x_1+r_1+r_2), (30+x_1+r_1+r_2))
        y_2 = random.randint(0, 60)
        bl_2 = random.randint(5, 60)
        return [bl_1, bj_1, bl_2, x_1, y_1,
                r_1, s_1, i_1, x_2, y_2, r_2, s_2, i_2]

    def save_png(self, filename, data, scaling):
        data, boundingBox = self.reposition(data, scaling)
        im = Image.new('L', boundingBox, 255)
        draw = ImageDraw.Draw(im)
        for i in range(len(data) - 1):
            draw.line(data[i] + data[i + 1], fill=0, width=1)
        del draw
        # Image origin is top left, convert to CAD-style bottom left
        im = ImageOps.flip(im)
        im.save(filename, "PNG")
    
    
                

    def reposition(self, data, scaling=1):
        '''Returns data, boundingBox'''
        xMin = data[0][0]
        yMin = data[0][1]
        xMax = data[0][0]
        yMax = data[0][1]

        for i in range(len(data)):
            if data[i][0] < xMin:
                xMin = data[i][0]
            if data[i][1] < yMin:
                yMin = data[i][1]
            if data[i][0] > xMax:
                xMax = data[i][0]
            if data[i][1] > yMax:
                yMax = data[i][1]

        repoData = []

        for i in range(len(data)):
            repoData.append((int((data[i][0] - xMin) * scaling), (int(
                (data[i][1] - yMin) * scaling))))

        return repoData, (int(
            (xMax - xMin) * scaling)+1, int((yMax - yMin) * scaling)+1)


if __name__ == '__main__':
    main = GeneratePath()
