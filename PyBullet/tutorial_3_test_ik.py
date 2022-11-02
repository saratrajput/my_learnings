import pybullet
import time
import pybullet_data

# To launch simulation
physicsClient = pybullet.connect(pybullet.GUI)

# Add Search Path
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load floor
planeID = pybullet.loadURDF("plane.urdf")

# Load robot again with fixed base and base position at 0
robot = pybullet.loadURDF(
    "/home/sp/repos/PyBulletProject/kuka_experimental/kuka_lbr_iiwa_support/urdf/lbr_iiwa_14_r820.urdf",
    [0, 0, 0],
    useFixedBase=1,
)

# Simulate gravity
pybullet.setGravity(0, 0, -9.81)

pybullet.setRealTimeSimulation(0)  # no realtime simulation

# Get Orientation
# We want to move the robot such as the head of the end-effector is pointed downwards
Orientation = pybullet.getQuaternionFromEuler([3.14, 0.0, 0.0])

# Calculate IK
targetPositionJoints = pybullet.calculateInverseKinematics(
    robot, 7, [0.1, 0.1, 0.4], targetOrientation=Orientation
)

# Move joints
pybullet.setJointMotorControlArray(
    robot, range(7), pybullet.POSITION_CONTROL, targetPositions=targetPositionJoints
)

for _ in range(500):
    pybullet.stepSimulation()
    time.sleep(0.2)

pybullet.disconnect()
