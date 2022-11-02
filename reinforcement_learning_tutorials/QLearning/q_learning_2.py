"""
Agent doesn't need to know anything about the environment. This is only for
your information. MountainCar env has 3 actions.
"""
import gym
import numpy as np

# We're gonna import the following environment
env = gym.make("MountainCar-v0")
# And every time we import an environment, we want to reset it

LEARNING_RATE = 0.1
# Discount is kind of like a weight. It's a measure of how important do we find
# future actions
# It's a measure of how important our future actions over current actions
# Our future reward vs our current reward
DISCOUNT = 0.95
EPISODES = 25000

# Every once in a while check to make sure everything is going as planned
SHOW_EVERY = 2000

# discrete observation space size
DISCRETE_OS_SIZE = [20] * len(env.observation_space.high)
# Now we're gonna break our continous values in 20 chunks
discrete_os_win_size = (
    env.observation_space.high - env.observation_space.low
) / DISCRETE_OS_SIZE

print(discrete_os_win_size)

# To add some random actions. For exploratory moves
epsilon = 0.5
START_EPSILON_DECAYING = 1
END_EPSILON_DECAYING = EPISODES // 2  # Always divide out to an integer
epsilon_decay_value = epsilon / (END_EPSILON_DECAYING - START_EPSILON_DECAYING)

# Initiliaze Q-table
q_table = np.random.uniform(
    low=-2, high=0, size=(DISCRETE_OS_SIZE + [env.action_space.n])
)


def get_discrete_state(state):
    """
    Convert continous states of position and velocity to discrete states
    """
    discrete_state = (state - env.observation_space.low) / discrete_os_win_size
    return tuple(discrete_state.astype(np.int64))


# Print the initial state
# print(discrete_state)
# Print our q-values which are random right now
# print(q_table[discrete_state])
# Print the index of the max q-value
# print(np.argmax(q_table[discrete_state]))


# Iterate over episodes
for episode in range(EPISODES):
    if episode % SHOW_EVERY == 0:
        print(episode)
        render = True
    else:
        render = False
    # env.reset() just returns the initial state so we can use it for testing
    discrete_state = get_discrete_state(env.reset())
    done = False

    while not done:

        ## Action depends on the max q value for the current discrete state
        # action = np.argmax(q_table[discrete_state])

        # If-else block is for using the epsilon values to make use of epsilon values
        # for random actions, for exploratory moves
        if np.random.random() > epsilon:
            # Action depends on the max q value for the current discrete state
            action = np.argmax(q_table[discrete_state])
        else:
            # This is the part for exploratory moves
            action = np.random.randint(0, env.action_space.n)

        new_state, reward, done, _ = env.step(action)
        new_discrete_state = get_discrete_state(new_state)

        if render:
            env.render()

        if not done:
            # max and not argmax because here we want the q-value rather than the arg-max
            max_future_q = np.max(q_table[new_discrete_state])
            # we get the q-value for that particular action
            current_q = q_table[discrete_state + (action,)]

            # This is the Q-learning formula
            new_q = (1 - LEARNING_RATE) * current_q + LEARNING_RATE * (
                reward + DISCOUNT * max_future_q
            )
            # the way it back-propagates is dependent on the DISCOUNT and max_future_q
            # values.

            # Update our q-table with the new q-value
            q_table[discrete_state + (action,)] = new_q
            # Pay attention that we're updating the discrete state after we took the step
            # of that discrete state

        elif new_state[0] >= env.goal_position:
            print(f"We made it on episode {episode}")
            # Set reward for completion which is zero
            q_table[discrete_state + (action,)] = 0

        discrete_state = new_discrete_state

    if END_EPSILON_DECAYING >= episode >= START_EPSILON_DECAYING:
        epsilon -= epsilon_decay_value

env.close()
