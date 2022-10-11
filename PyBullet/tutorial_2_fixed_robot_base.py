import pybullet
import time
import pybullet_data


# To launch simulation
physicsClient = pybullet.connect(pybullet.GUI)

# Add Search Path
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load floor
planeID = pybullet.loadURDF("plane.urdf")

# Load robot with fixed base and base position at 0
robot = pybullet.loadURDF("/home/sp/repos/PyBulletProject/kuka_experimental/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf", [0, 0, 0], useFixedBase = 1)

# Simulate gravity
pybullet.setGravity(0, 0, -9.81)

pybullet.setRealTimeSimulation(0) # no realtime simulation
# Don't really need the above line since it's by default

# Move joints
pybullet.setJointMotorControlArray(robot, range(7), pybullet.POSITION_CONTROL,
targetPositions=[1.5]*7)

for _ in range(1000):
    pybullet.stepSimulation()
    time.sleep(0.1)

pybullet.disconnect()
