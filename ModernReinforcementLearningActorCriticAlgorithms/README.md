# Modern Reinforcement Learning: Actor-Critic Algorithms

## Section-2: Fundamentals of Reinforcement Learning

### Review of Fundamental Concepts

#### Agent, Environment, Action

* Agent interacting with an environment which causes it to transition from one state to another.
* These actions will result in agent getting a reward from the environment which tells if it
was good or bad.
* Various algorithms look for correlations between these actions, state transitions and rewards
to maximize the agents total reward over time.

#### Markov Decision Process

* State depends only on previous state and action.
* We can represent the history of the system as a set of state action reward tuples.

#### Episodic Returns

* States have value.
    * We can add up and weigh the total rewards the agent receives over time to determine the
    value of whatever state the agent finds itself in.
* Present rewards are worth more.
* Sum of discounted rewards. -> Episode return.

#### The Agent's Policy.

* Mapping of states to actions.
* Can be probabilistic.

#### Learning from Experience.

* Interact with environment.
* Keep track of rewards.

#### Optimal Policies

* Compare policies.
* Known dynamics -> Model based.
* Unknown dynamics -> Model free.

#### Conclusion

* Keep track of rewards to estimate value and action value functions.
* Recursive relationship between functions.
* Have to interact w/ environment to learn dynamics.
* Policy gradient & Actor critic -> on policy model free.

### Teaching an AI about Black Jack with Monte Carlo Prediction

#### Monte Carlo (MC) Methods
* Model free algorithms.
* Average received rewards.

#### Prediction & Control Problems

* Finding the optimal value functions and then using that to optimize the policy
is two discrete sets of operations.
* There are some algorithms for calculating the state value an action value functions.
* There are some outcomes for finding the optimal policies.
* These are called prediction and control problems respectively.

#### First vs Every Visit MC.

* Tracking rewards received after visiting states.
* Rewards received after first visit. -> First visit MC.
* Rewards received after every visit. -> Every visit MC.
