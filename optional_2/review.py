import pybullet as p
import pybullet_data
import time
import os

physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, 0)

planeId = p.loadURDF(os.path.expanduser("~/.local/lib/python3.12/site-packages/pybullet_data/plane.urdf"))

euler_angles = [0, 0, 0]

start_orientation = p.getQuaternionFromEuler(euler_angles)
start_position = [0, 0, 3]

robotId = p.loadURDF(os.path.expanduser("~/.local/lib/python3.12/site-packages/pybullet_data/sphere2.urdf"), start_position, start_orientation)

g = -9.81
z = start_position[2]
v = 0
dt = 1/240.
e = 0.8

while(1):
    z = z + v + (1/2)*g*dt**2
    v = v + g *dt

    if z <= 0.5:
        v = -v * e
        if abs(v) < 0.15:
            v = 0

    p.resetBasePositionAndOrientation(robotId, [0, 0, z], start_orientation)
    p.stepSimulation()
    time.sleep(dt)

p.disconnect()