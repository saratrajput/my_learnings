import numpy as np
from PIL import Image
import cv2
import matplotlib.pyplot as plt
import pickle
from matplotlib import style
import time  # to set dynamic q-table file names

style.use("ggplot")

# We'll make a 10x10 grid
SIZE = 10
ACTION_SPACE = 8
# How many episodes
HM_EPISODES = 25000
MOVE_PENALTY = 1
ENEMY_PENALTY = 300
FOOD_REWARD = 25
# epsilon = 0.9
epsilon = 0.0  # For second iteration
EPS_DECAY = 0.9998
# SHOW_EVERY = 3000
SHOW_EVERY = 1  # For second iteration

# start_q_table = None  # or filename
# For second iteration
start_q_table = "q_table-1617617971.pickle"  # or filename
# Useful if you want to continue training or play around with epsilon starting
# from some episodes

LEARNING_RATE = 0.1
DISCOUNT = 0.95

# These are numbers for keys in a dictionary
PLAYER_N = 1
FOOD_N = 2
ENEMY_N = 3

# Colors are being defined in BGR
d = {1: (255, 175, 0),  # lightish blue
     2: (0, 255, 0),  # Food is green
     3: (0, 0, 255)}  # Enemy is red


class Blob:
    """
    All the blobs need to be able to move, need a starting location, need to be
    initialized randomly.
    The observation space is gonna be the relative position of the food, and the
    relative position of the enemy to the player.
    """
    def __init__(self):
        """
        There is a possibility that the player, enemy and food positions
        might overlap
        """
        self.x = np.random.randint(0, SIZE)
        self.y = np.random.randint(0, SIZE)

    def __str__(self):
        """
        Print blob's location.
        """
        return f"{self.x}, {self.y}"

    def __sub__(self, other):
        """
        Operator overloading.
        Subtract a blob from another blob.
        """
        return self.x - other.x, self.y - other.y

    def action(self, choice):
        """
        Only player is gonna use action.
        Action will interact with move method.
        We wanna pass a discrete value to action.

        Input:
         choice: a discrete value.

        The player can move only diagonally, but it still works.
        Note: I've added extra actions to move up, down, right, left too.
        """
        if choice == 0:
            self.move(x=1, y=1)
        elif choice == 1:
            self.move(x=-1, y=-1)
        elif choice == 2:
            self.move(x=-1, y=1)
        elif choice == 3:
            self.move(x=1, y=-1)
        elif choice == 4:
            self.move(x=0, y=1)
        elif choice == 5:
            self.move(x=1, y=0)
        elif choice == 6:
            self.move(x=-1, y=0)
        elif choice == 7:
            self.move(x=0, y=-1)

    def move(self, x=False, y=False):
        """
        The random moves here can move up and down.
        :param x:
        :param y:
        :return:
        """
        if not x:
            self.x += np.random.randint(-1, 2)  # Between -1 and 2
        else:
            self.x += x

        if not y:
            self.y += np.random.randint(-1, 2)  # Between -1 and 2
        else:
            self.y += y

        if self.x < 0:
            self.x = 0
        elif self.x > SIZE-1:
            self.x = SIZE-1

        if self.y < 0:
            self.y = 0
        elif self.y > SIZE - 1:
            self.y = SIZE - 1


# Either create a q-table or load a q-table
if start_q_table is None:
    q_table = {}
    # Observation space is gonna be two coordinates. The first one is gonna be
    # delta to the food, so relative difference between player and food. And the
    # second one is with the enemy. So it's gonna be like (x1, y1) (x2, y2)
    for x1 in range(-SIZE+1, SIZE):
        for y1 in range(-SIZE + 1, SIZE):
            for x2 in range(-SIZE + 1, SIZE):
                for y2 in range(-SIZE + 1, SIZE):
                    q_table[((x1, y1), (x2, y2))] = [np.random.uniform(-5, 0)
                                                     for i in range(ACTION_SPACE)]
                    # Since our action space is now 8
else:
    with open(start_q_table, "rb") as f:
        q_table = pickle.load(f)

# Training code
episode_rewards = []
for episode in range(HM_EPISODES):
    player = Blob()
    food = Blob()
    enemy = Blob()

    if episode % SHOW_EVERY == 0:
        print(f"on # {episode}, epsilon: {epsilon}")
        print(f"{SHOW_EVERY} ep mean {np.mean(episode_rewards[-SHOW_EVERY:])}")
        show = True
    else:
        show = False

    episode_reward = 0
    for i in range(200):  # how many steps we'd take
        obs = (player-food, player-enemy)
        if np.random.random() > epsilon:
            action = np.argmax(q_table[obs])  # Regular action
        else:
            action = np.random.randint(0, 8)  # Random action

        player.action(action)

        ### Maybe later
        enemy.move()
        food.move()
        ###

        # Reward process
        # Same pos as enemy
        if player.x == enemy.x and player.y == enemy.y:
            reward = -ENEMY_PENALTY
        elif player.x == food.x and player.y == food.y:
            reward = FOOD_REWARD
        else:
            reward = -MOVE_PENALTY

        # We need to make a new observation based on the movement
        new_obs = (player-food, player-enemy)
        max_future_q = np.max(q_table[new_obs])
        current_q = q_table[obs][action]

        if reward == FOOD_REWARD:
            new_q = FOOD_REWARD  # Once we've reached food, we're done
        elif reward == -ENEMY_PENALTY:
            new_q = -ENEMY_PENALTY
        else:
            # Q-formula
            new_q = (1 - LEARNING_RATE) * current_q + LEARNING_RATE * \
                    (reward + DISCOUNT * max_future_q)

        q_table[obs][action] = new_q

        if show:
            # Create environment
            env = np.zeros((SIZE, SIZE, 3), dtype=np.uint8)  # Blank environment
            env[food.y][food.x] = d[FOOD_N]  # Food color
            env[player.y][food.x] = d[PLAYER_N]  # Player color
            env[enemy.y][enemy.x] = d[ENEMY_N]  # Enemy color

            img = Image.fromarray(env, "RGB")
            img = img.resize((300, 300))
            cv2.imshow("My Env", np.array(img))

            if reward == FOOD_REWARD or reward == -ENEMY_PENALTY:
                if cv2.waitKey(500) & 0xFF == ord("q"):
                    break
            else:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

        episode_reward += reward
        if reward == FOOD_REWARD or reward == -ENEMY_PENALTY:
            break

    episode_rewards.append(episode_reward)
    epsilon *= EPS_DECAY

# Create a moving average
moving_avg = np.convolve(episode_rewards, np.ones((SHOW_EVERY,)) / SHOW_EVERY,
                         mode="valid")
plt.plot([i for i in range(len(moving_avg))], moving_avg)
plt.ylabel(f"reward {SHOW_EVERY}ma")
plt.xlabel("episode #")
plt.show()

with open(f"q_table-{int(time.time())}.pickle", "wb") as f:
    pickle.dump(q_table, f)
