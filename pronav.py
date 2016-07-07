import numpy as np
from scipy.integrate import odeint
import math
def tovpy(v):
    from visual import vector
    return vector(v[1], -v[2], -v[0])

class Copter:
    def __init__(self):
        self.t = 0.
        self.dt = 1./60.
        self.state = np.array([5.,5.,-10.,0.,0.,0.5])

    def get_pos(self):
        return self.state[0:3]

    def get_vel(self):
        return self.state[3:6]

    def get_accel(self):
        KD = 5.
        omega = np.cross(self.get_pos(), self.get_vel())/sum([x**2 for x in self.get_pos()])
        return np.array([KD*self.get_vel()[2]*omega[1],-KD*self.get_vel()[2]*omega[0],0.])

    def dyn(self, y, t):
        self.t = t
        self.state = y
        return np.concatenate((self.get_vel(), self.get_accel()))

    def update(self):
        tbegin = self.t
        tend = self.t+self.dt
        self.state = odeint(self.dyn, self.state, np.linspace(tbegin,tend,2.))[-1]


import visual as vpy

scene = vpy.display(title = 'proportional nav visualizer', width=1000, height=1000, center = tovpy((0.,0.,-5.)))
scene.autoscale = False
scene.range = 10
scene.forward = tovpy((math.cos(math.radians(0.)),0.,math.sin(math.radians(0.))))
scene.ambient = vpy.color.gray(0.6)
scene.background = vpy.color.white

copterModel = Copter()
copter = vpy.sphere(pos=tovpy(copterModel.get_pos()), radius=0.1, color=vpy.color.red)
target = vpy.cylinder(axis=tovpy((0.,0.,0.1)), radius=0.3, color=vpy.color.red)
while True:
    vpy.rate(60)
    copterModel.update()
    copter.pos = tovpy(copterModel.get_pos())
