import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
from matplotlib.patches import Circle
from matplotlib.collections import PatchCollection
import stagger as st


class GraphAnimation(object):
    def __init__(self):
        # (length, joint = 0)
        a = self.get_params()
        self.bar1 = st.Bar(a[0], a[1])
        self.bar2 = st.Bar(a[2])

        # (x, y, r, speed = 1, initial = 0)
        self.drive1 = st.Anchor(a[3], a[4], a[5], a[6], a[7])
        self.drive2 = st.Anchor(a[8], a[9], a[10], a[11], a[12])

        self.motionSystem = st.TwoBar(
            self.drive1, self.drive2, self.bar1, self.bar2)

        # xlim, ylim, line1 style, weight, line2 style, weight
        self.init_graph((-25, 50), (-10, 65), 'y-', 4, 'y-', 4)

        #facecolor1, facecolor2
        self.init_circles('g', 'r')

        self.patchbar1Base = plt.Circle((5, -5), 1.5, fc='r')
        self.patchbar2Base = plt.Circle((5, -5), 1.5, fc='r')
        self.patchbar2End = plt.Circle((5, -5), 1.5, fc='r')

        self.patchJoint = plt.Circle((5, -5), 1.5, fc='r')

        last = self.motionSystem.end_path(0)

        for i in range(self.motionSystem.totalFrames):
            next = self.motionSystem.end_path(i * self.motionSystem.stepSize)
            if i > 0:
                plt.plot([last[0], next[0]], [last[1], next[1]], 'k-')
            last = next
        # Begin animation
        anim = animation.FuncAnimation(self.fig, self.animate_frame, init_func=self.init_animation, frames=self.motionSystem.totalFrames, interval=10, blit=True)
        # anim.save(".\\outputs\\anim.gif", writer="imagemagick", fps=30, )
        # Display drawn path to be output
        print(self.motionSystem.totalFrames, self.motionSystem.stepSize)
        
        # self.CURVE = st.resample(self.motionSystem.)
        # self.pca = st.PCA(self.CURVE)
        
        # self.fi= st.FVC(self.CURVE, self.pca[4], self.pca[5], self.pca[2], self.pca[1], self.pca[0])
        plt.show()

    def init_graph(self, xlim, ylim, ls1, lw1, ls2, lw2):
        self.fig = plt.figure(figsize=(6, 6))
        self.ax = plt.axes(xlim=xlim, ylim=ylim)
        self.line1, = self.ax.plot([], [], ls1, lw=lw1)
        self.line2, = self.ax.plot([], [], ls2, lw=lw2)

    def init_circles(self, fc1, fc2):
        #return
        patches = []
        gear1 = Circle((self.drive1.x, self.drive1.y), self.drive1.r)
        patches.append(gear1)
        gear2 = Circle((self.drive2.x, self.drive2.y), self.drive2.r)
        patches.append(gear2)
        p = PatchCollection(patches)
        self.ax.add_collection(p)

    def init_animation(self):
       # return
        self.line1.set_data([], [])
        self.line2.set_data([], [])
        self.patchbar1Base.center = (5, 5)
        self.patchbar2Base.center = (5, 5)
        self.patchJoint.center = (5, 5)

        self.ax.add_patch(self.patchbar1Base)
        self.ax.add_patch(self.patchbar2Base)
        self.ax.add_patch(self.patchJoint)
        self.ax.add_patch(self.patchbar2End)

        return self.line1, self.line2, self.patchbar1Base, self.patchbar2Base, self.patchJoint, self.patchbar2End

    def animate_frame(self, i):
        #return
        # drive 1
        angle1 = i * self.drive1.speed * self.motionSystem.stepSize
        drive1X, drive1Y = self.drive1.base_point(angle1)

        self.patchbar1Base.center = (drive1X, drive1Y)
        # drive 2
        angle2 = i * self.drive2.speed * self.motionSystem.stepSize
        drive2X, drive2Y = self.drive2.base_point(angle2)

        self.patchbar2Base.center = (drive2X, drive2Y)

        driveLengthR, driveAngle = self.drive1.base_point_distance(
            angle1, angle2, self.drive2)

        angle = self.motionSystem.sides_to_angle(
            self.bar1.joint, driveLengthR, self.bar2.length)
        x2, y2 = self.motionSystem.line_end(
            drive1X, drive1Y, self.bar1.length, angle + driveAngle)

        self.line1.set_data([drive1X, x2], [drive1Y, y2])
        self.patchJoint.center = self.motionSystem.line_end(
            drive1X, drive1Y, self.bar1.joint, angle + driveAngle)
        self.patchbar2End.center = (x2, y2)

        angle = self.motionSystem.sides_to_angle(
            self.bar2.length, driveLengthR, self.bar1.joint)
        x2, y2 = self.motionSystem.line_end(
            drive2X, drive2Y, self.bar2.length, (np.pi - angle) + driveAngle)
        self.line2.set_data([drive2X, x2], [drive2Y, y2])

        return self.line1, self.line2, self.patchbar1Base, self.patchbar2Base, self.patchJoint, self.patchbar2End,

    def get_params(self):
        return np.genfromtxt(r"outputs\\base.par", delimiter=" ", dtype=np.int16)

if __name__ == '__main__':
    main = GraphAnimation()
