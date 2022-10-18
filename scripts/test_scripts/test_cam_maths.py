import numpy as np
import time
from state_machine_ros import StateMachine
import random
from constants import BASE_TO_BELT

class BoxTransformArray:
    def __init__(self, transforms):
        self.transforms = transforms

class BoxTransform:
    def __init__(self, fiducial_id, transform):
        self.fiducial_id = fiducial_id
        self.transform = transform

class Transform:
    def __init__(self, translation, rotation):
        self.translation = translation
        self.rotation = rotation

class VectorQuat:
    def __init__(self, x, y, z, w=None):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

sm = StateMachine(False)

steps = 20
ttime = 6
#xvel = 1
#yvel = 1
omega = 6 / 30 * np.pi

def posfunc(t, rad=0.1, noise=0):
    if t > 2 and t <= 4:
        t = 2
    elif t > 4:
        t -= 2
    return BASE_TO_BELT + rad*np.cos(omega*t)+random.gauss(0, noise), rad*np.sin(omega*t)+random.gauss(0, noise)
    #return xvel*t, yvel*t

def velfunc(t, rad=0.1):
    dt = 0.0001
    x2, y2 = posfunc(t+dt, rad)
    x1, y1 = posfunc(t-dt, rad)
    return (x2-x1)/(2*dt), (y2-y1)/(2*dt)

def spdfunc(t, rad=0.1):
    x, y = velfunc(t, rad)
    return np.sqrt(x**2 + y**2)

for t in np.linspace(0, ttime, steps+1):
    stime = time.time()
    x, y = posfunc(t, rad=0.1, noise=0.005)
    translation = VectorQuat(x, y, 0)
    rotation = VectorQuat(0, 0, 0, 0)
    transform = Transform(translation, rotation)
    boxtransform1 = BoxTransform(1, transform)

    if random.random() < 0.8:
        x, y = posfunc(t, rad=0.15, noise=0.001)
        translation = VectorQuat(x, y, 0)
        rotation = VectorQuat(0, 0, 0, 0)
        transform = Transform(translation, rotation)
        boxtransform2 = BoxTransform(2, transform)
        msg = BoxTransformArray([boxtransform1, boxtransform2])
    else:
        msg = BoxTransformArray([boxtransform1])
    sm.camera_callback(msg)
    #print([a*30/np.pi if a is not None else None for a in sm.omegas])
    print(sm.omega*30/np.pi)
    print(f'actual vel = {spdfunc(t, 0.15)}')
    ltime = time.time() - stime
    print(f'loop time = {ltime*1000} ms')
    time.sleep(ttime/steps - ltime)
