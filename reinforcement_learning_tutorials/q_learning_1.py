'''
Agent doesn't need to know anything about the environment. This is only for
your information. MountainCar env has 3 actions.
'''
import gym
import numpy as np

# We're gonna import the following environment
env = gym.make("MountainCar-v0")
# And every time we import an environment, we want to reset it
env.reset()

# Print the highest and the lowest possible values of states
print(env.observation_space.high) # [0.6 0.07]
print(env.observation_space.low) # [-1.2 -0.07]
# Print number of actions in the env
print(env.action_space.n) # 3


# discrete observation space size
# Usually we wouldn't hardcode this as this would change by environment
DISCRETE_OS_SIZE = [20] * len(env.observation_space.high)
# Now we're gonna break our continous values in 20 chunks
discrete_os_win_size = (env.observation_space.high - env.observation_space.low) / DISCRETE_OS_SIZE

print(discrete_os_win_size)

# Initiliaze Q-table
# These two values you might change depending on the environment
q_table = np.random.uniform(low=-2, high=0, size=(DISCRETE_OS_SIZE + [env.action_space.n]))
# This will make a 20x20x3 table, so every combination of possible environment
# observations x 3 actions that we could take. And inside of those cells, right
# now we have random q values.
print(q_table.shape)
print(q_table)
                                            

'''
done = False

while not done:
    # We want to step through the environment
    action = 2
    # Everytime we step through the environment we get a new state which is like
    # an observation we sense from the environment
    new_state, reward, done, _ = env.step(action)
    print(reward, new_state)
    # So we saw that the reward is gonna be -1 until it reaches the goal flag
    # then it'll be zero.

    # We can see the new state values are upto 8 decimal points
    # If we were to create a Q-table of all the combinations of these continous values
    # then it would be highly compute intensive. So we're gonna first convert these
    # continuous values to discrete values.

    # Render the environment
    env.render()

env.close()
'''
