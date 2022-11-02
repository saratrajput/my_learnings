import pybullet
import time
import pybullet_data


# To launch simulation
physicsClient = pybullet.connect(pybullet.GUI)

# Add Search Path
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

# Serves as ground in our simulation
plainID = pybullet.loadURDF("plane.urdf")  # Present in pybullet_data

# Load robot
robot = pybullet.loadURDF(
    "/home/sp/repos/PyBulletProject/kuka_experimental/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf"
)

# robot = pybullet.loadURDF("/home/sp/repos/PyBulletProject/lbr_iiwa_14_r820.urdf")
# Get info about robot
pybullet.getNumJoints(robot)

# Orientation is a quaternion
position, orientation = pybullet.getBasePositionAndOrientation(robot)

pybullet.getJointInfo(robot, 7)

joint_positions = [j[0] for j in pybullet.getJointStates(robot, range(7))]

print(joint_positions)

world_position, world_orientation = pybullet.getLinkState(robot, 2)[:2]

print(world_position)

# Simulate gravity
pybullet.setGravity(0, 0, -9.81)

pybullet.setRealTimeSimulation(0)  # no realtime simulation

pybullet.setJointMotorControlArray(
    robot, range(7), pybullet.POSITION_CONTROL, targetPositions=[0.2] * 7
)

for _ in range(1000):
    pybullet.stepSimulation()
    time.sleep(1.0 / 240.0)

pybullet.disconnect()
