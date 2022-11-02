"""
Agent doesn't need to know anything about the environment. This is only for
your information. MountainCar env has 3 actions.

Full tutorial:
https://pythonprogramming.net/q-learning-analysis-reinforcement-learning-python-tutorial/
"""
import gym
import numpy as np
import matplotlib.pyplot as plt

# We're gonna import the following environment
env = gym.make("MountainCar-v0")

# Last example, we set these values manually, and in the real world it might
# not work
LEARNING_RATE = 0.1
DISCOUNT = 0.95
EPISODES = 2000

# Also this is not a statistically significant sample
SHOW_EVERY = 500

# discrete observation space size
DISCRETE_OS_SIZE = [20] * len(env.observation_space.high)
# Now we're gonna break our continuous values in 20 chunks
discrete_os_win_size = (
    env.observation_space.high - env.observation_space.low
) / DISCRETE_OS_SIZE

print(discrete_os_win_size)

# To add some random actions. For exploratory moves
epsilon = 0.5
START_EPSILON_DECAYING = 1
END_EPSILON_DECAYING = EPISODES // 2  # Always divide out to an integer
epsilon_decay_value = epsilon / (END_EPSILON_DECAYING - START_EPSILON_DECAYING)

# Initialize Q-table
q_table = np.random.uniform(
    low=-2, high=0, size=(DISCRETE_OS_SIZE + [env.action_space.n])
)

# Episode rewards
ep_rewards = []
# A dictionary tracking episode number, average, min and max
aggr_ep_rewards = {"ep": [], "avg": [], "min": [], "max": []}
# ep: is like the x-axis of the graph
# avg: is the windowed average, our window is like 500. It should go up as our model
# improves
# min: is the worst model that we have
# max: is the best model that we have. We might have cases where we want our worst
# model to be somewhat decent compared to the average
# We'll use these metrics to optimize depending on our task


def get_discrete_state(state):
    """
    Convert continuous states of position and velocity to discrete states
    """
    discrete_state = (state - env.observation_space.low) / discrete_os_win_size
    return tuple(discrete_state.astype(np.int64))


# Iterate over episodes
for episode in range(EPISODES):
    # Track episode rewards
    episode_reward = 0
    if episode % SHOW_EVERY == 0:
        print(episode)
        render = True
    else:
        render = False
    # env.reset() just returns the initial state so we can use it for testing
    discrete_state = get_discrete_state(env.reset())
    done = False

    while not done:

        # Action depends on the max q value for the current discrete state
        # action = np.argmax(q_table[discrete_state])

        # If-else block is for using the epsilon values to make use of epsilon
        # values for random actions, for exploratory moves
        if np.random.random() > epsilon:
            # Action depends on the max q value for the current discrete state
            action = np.argmax(q_table[discrete_state])
        else:
            # This is the part for exploratory moves
            action = np.random.randint(0, env.action_space.n)

        new_state, reward, done, _ = env.step(action)
        episode_reward += reward

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

    ep_rewards.append(episode_reward)

    if not episode % SHOW_EVERY:
        # Save the q-table
        np.save(f"qtables/{episode}-qtable.npy", q_table)
        # Make the aggr_ep_rewards dictionary
        average_reward = sum(ep_rewards[-SHOW_EVERY:]) / len(ep_rewards[-SHOW_EVERY:])
        aggr_ep_rewards["ep"].append(episode)
        aggr_ep_rewards["avg"].append(average_reward)
        aggr_ep_rewards["min"].append(min(ep_rewards[-SHOW_EVERY:]))
        aggr_ep_rewards["max"].append(max(ep_rewards[-SHOW_EVERY:]))

        print(
            f"Episode: {episode} avg:{average_reward} min:{min(ep_rewards[-SHOW_EVERY:])} "
            f"max:{max(ep_rewards[-SHOW_EVERY:])}"
        )

env.close()

plt.plot(aggr_ep_rewards["ep"], aggr_ep_rewards["avg"], label="avg")
plt.plot(aggr_ep_rewards["ep"], aggr_ep_rewards["min"], label="min")
plt.plot(aggr_ep_rewards["ep"], aggr_ep_rewards["max"], label="max")
plt.legend(loc=4)
plt.show()
