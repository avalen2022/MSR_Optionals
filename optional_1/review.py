import pybullet as p
import pybullet_data
import time
import os

physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

planeId = p.loadURDF(os.path.expanduser("~/.local/lib/python3.12/site-packages/pybullet_data/plane.urdf"))

euler_angles = [0, 0, 0]

start_orientation = p.getQuaternionFromEuler(euler_angles)
start_position = [0, 0, 1]

robotId = p.loadURDF(os.path.expanduser("review.urdf"), start_position, start_orientation)

# Get joints
numJoints = p.getNumJoints(robotId)
print("NumJoints: " + str(numJoints))

for j in range (numJoints):
    print("%d - %s" % (p.getJointInfo(robotId, j)[0], p.getJointInfo(robotId, j)[1].decode("utf-8")))

#rotateId = p.addUserDebugParameter("rotate_vel", -30, 30, 5)
frictionId = p.addUserDebugParameter("friction", 0, 5, 0.05)
torqueId = p.addUserDebugParameter("torque", -20, 20, 0)

while(1):
    #vel_fore = p.readUserDebugParameter(rotateId)
    friction_fore = p.readUserDebugParameter(frictionId)
    torque_fore = p.readUserDebugParameter(torqueId)
    p.setJointMotorControl2(robotId, 1, p.TORQUE_CONTROL, force=torque_fore)
    p.setJointMotorControl2(robotId, 1, p.VELOCITY_CONTROL, targetVelocity=0, force=friction_fore)
    p.changeDynamics(robotId, 1, spinningFriction=friction_fore)

    p.stepSimulation()
    time.sleep(1/240.)

p.disconnect()