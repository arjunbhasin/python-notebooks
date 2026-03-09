# Reinforcement Learning — A Deep Primer
## From Intuition to SAC, With Full Mathematical Detail

> **Who this is for.** You understand robotics kinematics well and have seen RL at a surface level.
> This document builds RL from the ground up with full mathematical rigour, genuine intuition,
> and a running example that eventually becomes your null-space policy. Read every section in order
> the first time. The concepts build on each other in a way that makes the later parts much cleaner
> if you haven't skipped anything.

---

## Part I — The Problem RL Is Solving

---

### 1. Sequential Decision Making — What Makes It Hard

Almost every interesting problem in robotics, games, and control involves making a **sequence of decisions** where the outcome of each decision depends on what came before, and the consequences of a decision often only become apparent many steps later. This is the core difficulty RL addresses.

Consider what happens when your 4-DOF arm is in an awkward configuration — joints bunched near their limits, manipulability low. Getting into that situation might have been the result of many small null-space actions taken 50 steps ago. A controller that only looks one step ahead had no way to know it was digging itself into a hole. This is the **credit assignment problem**: which past actions deserve credit (or blame) for the current situation?

Supervised learning sidesteps this entirely — you just need labelled input-output pairs. But in RL, there are no labels. There is only the reward signal, which arrives after an action and mixes together the effects of dozens of prior decisions. Building a learning algorithm that can untangle this is the central achievement of modern RL.

---

### 2. The Reinforcement Learning Framework — A Bird's Eye View

Before diving into equations, here is the mental model to hold throughout. There is an **agent** (your policy/neural network) that interacts with an **environment** (your robot simulation). At each moment, the agent observes the current **state** of the environment, produces an **action**, the environment transitions to a new state, and emits a **reward** signal. This loop repeats.

```
         ┌──────────────────────────────────────────────┐
         │                 Environment                  │
         │                                              │
         │   s_{t+1}, r_t ◄── transition(s_t, a_t)     │
         └──────────┬──────────────────────┬────────────┘
                    │ s_{t+1}, r_t         │ s_t
                    ▼                      │
         ┌──────────────────────────────────────────────┐
         │                  Agent                       │
         │                                              │
         │   a_t ◄── π(s_t)    [policy]                │
         │   V(s_t) ◄── value estimate  [critic]        │
         └──────────────────────────────────────────────┘
                    │ a_t
                    └──────────────────────►
```

The agent's goal is to find a **policy** — a rule for picking actions — that maximises the total reward accumulated over time. That's it. Everything else in RL is machinery to solve this efficiently and stably.

---

## Part II — Markov Decision Processes (MDPs)

---

### 3. Formal Definition of an MDP

An MDP is the mathematical framework that makes "sequential decision making" precise. Formally, it is a tuple $(\mathcal{S}, \mathcal{A}, P, r, \gamma, \rho_0)$ where each component captures one aspect of the problem.

**State space $\mathcal{S}$** is the set of all possible situations the environment can be in. In your null-space env, a state is a vector $\boldsymbol{s} = [\boldsymbol{q}/\pi,\; \dot{\boldsymbol{q}},\; \boldsymbol{e}_{ee},\; z_\text{phase}] \in \mathbb{R}^{11}$. Note that a "state" in the MDP sense must be **Markov** — meaning the future evolution of the system depends only on the current state, not on the history of how you got there. This is the Markov property.

**The Markov property** states:

$$P(s_{t+1} \mid s_t, a_t, s_{t-1}, a_{t-1}, \ldots) = P(s_{t+1} \mid s_t, a_t)$$

In English: knowing the current state gives you all the information that matters. The past is irrelevant once you know the present. Your robot environment satisfies this because the joint angles and velocities fully describe the mechanical state — you don't need to know *how* the arm got to that configuration.

**Action space $\mathcal{A}$** is the set of available actions. For continuous control (your case), $\mathcal{A} = [-1, 1]^4 \subset \mathbb{R}^4$ — a compact subset of 4-dimensional real space. This is one of the key technical differences from discrete RL (like Atari games), and it requires different algorithms.

**Transition dynamics $P(s' \mid s, a)$** is a probability distribution over next states given the current state and action. In your deterministic simulation, this is a delta function: $P(s' \mid s, a) = \delta(s' - f(s, a))$ where $f$ is your env's step function. In the real world, there would be stochasticity from motor noise, sensor noise, etc.

**Reward function $r : \mathcal{S} \times \mathcal{A} \to \mathbb{R}$** is a scalar signal emitted after each transition. In your case:

$$
r(s, a) \;=\; \underbrace{w_1\,\sqrt{\det\!\bigl(\mathbf{J}\mathbf{J}^\top\bigr)}}_{\text{manipulability}}
\;+\; \underbrace{w_2\!\left(-\sum_i \left(\frac{q_i}{q_{\max}}\right)^{\!4}\right)}_{\text{joint-limit avoidance}}
\;+\; \underbrace{w_3\bigl(-\|\Delta\boldsymbol{q}\|^2\bigr)}_{\text{smoothness}}
$$

**Discount factor $\gamma \in [0, 1)$** controls how much the agent cares about future rewards relative to immediate ones. More on this in Section 5.

**Initial state distribution $\rho_0$** is the distribution over starting states. For your env, this is a uniform distribution over joint configurations away from singularities.

---

### 4. A Running Simple Example — The Ball-in-Bowl

To build intuition without the complexity of a robot, consider a ball on a bowl-shaped surface. The state is the ball's position $s \in [-1, 1]$. The action is a horizontal force $a \in [-1, 1]$. The reward is $r(s, a) = -s^2 - 0.01 a^2$ — the ball gets rewarded for being near the centre ($s = 0$) and penalised slightly for large forces (energy cost).

This is trivially solvable by calculus, but it is valuable because we can track exactly what every RL formula means in concrete terms. The optimal policy is obviously $\pi^*(s) = -\text{sign}(s) \cdot k|s|$ for some gain $k$ — push the ball toward centre proportional to how far it is. We will use this example repeatedly to sanity-check RL formulas before applying them to your robot.

---

### 5. The Discount Factor — Why It Exists and What It Really Means

The discount factor $\gamma$ is one of those things that looks like a technical trick but actually carries deep meaning. Let's unpack it carefully.

**The mathematical reason.** Without discounting, the total reward $\sum_{t=0}^\infty r_t$ might not converge — if rewards are bounded but never zero, the sum is infinite. With $\gamma < 1$, the geometric series $\sum_{t=0}^\infty \gamma^t |r_\text{max}| = |r_\text{max}| / (1 - \gamma)$ is always finite.

**The economic reason.** A reward received now is worth more than the same reward received later, because receiving it now gives you the opportunity to use it sooner. This is time value of money applied to reward.

**The practical reason for your system.** With $\gamma = 0.99$ and 200-step episodes, the reward at step $t$ from the start is discounted by $\gamma^t$. At step 100, a reward is worth $0.99^{100} \approx 0.37$ of face value. At step 200, it's worth $0.99^{200} \approx 0.13$. This means the agent prioritises near-term improvements — which is good for stability — but still cares about long-horizon consequences.

**The conceptual reason.** $\gamma$ controls the *effective horizon* of the agent. You can show that the effective number of steps the agent "looks ahead" is approximately $1/(1-\gamma)$. For $\gamma = 0.99$, the effective horizon is 100 steps. For $\gamma = 0.9$, it's 10 steps. Choosing $\gamma$ is really about choosing how far-sighted you want your agent to be.

**Computing the discounted return.** Define the return $G_t$ as the total discounted reward from step $t$ onward:

$$G_t = r_t + \gamma r_{t+1} + \gamma^2 r_{t+2} + \cdots = \sum_{k=0}^{\infty} \gamma^k r_{t+k}$$

Notice the beautiful recursive structure: $G_t = r_t + \gamma G_{t+1}$. This **Bellman recursion** is the most important identity in all of RL. It says: "the total return from now is the immediate reward plus the discounted total return from the next step." This recursion is what makes it possible to estimate $G_t$ without seeing the entire future.

**Ball-in-bowl example.** Suppose the ball is at $s = 0.5$, you apply force $a = -0.3$, and the ball gradually converges to centre. The returns might look like:

$$G_0 = -0.25 + 0.99 \cdot (-0.16) + 0.99^2 \cdot (-0.09) + \cdots \approx -3.1$$

An agent that just looks at immediate reward would see $r_0 = -0.25 - 0.01(0.09) = -0.259$ — and might prefer doing nothing ($r = -0.25$, slightly better immediately). But looking at $G_0$, applying the force is better because it drives future rewards toward zero.

---

## Part III — Value Functions

---

### 6. The State Value Function $V^\pi(s)$

The value function is the most central object in RL. It answers: "if I am in state $s$ and follow policy $\pi$ from here, what total discounted reward can I expect?"

$$V^\pi(s) = \mathbb{E}_\pi\left[G_t \;\middle|\; s_t = s\right] = \mathbb{E}_\pi\left[\sum_{k=0}^{\infty} \gamma^k r_{t+k} \;\middle|\; s_t = s\right]$$

The $\mathbb{E}_\pi[\cdot]$ notation means the expectation is taken over the randomness of the policy $\pi$ (stochastic action selection) and the environment transitions.

**Why is this useful?** Instead of tracking the infinite future, we want a function that encodes "how good is this state?" Once we have $V^\pi$, we can improve the policy by preferring actions that lead to high-value states.

**Ball-in-bowl value function.** For the optimal policy, $V^*(s) \approx -s^2 / (1-\gamma)$ (roughly — the agent keeps getting penalised proportional to $s^2$ until it reaches zero, and discounting compresses this into a finite number). States near centre have value close to zero. States at the edges have large negative value. The value function is a landscape of "how bad is this situation."

**Computing $V^\pi$ for your robot.** In your null-space env, $V^\pi(s)$ represents: "given the current joint configuration, EE error, and task phase, how much total null-space quality (manipulability + joint limit + smoothness) can I expect from here under policy $\pi$?" A high-value state means the arm is in a good configuration to sustain high null-space quality throughout the episode.

---

### 7. The Bellman Equation for $V^\pi$

The Bellman equation is what makes value functions *learnable*. It expresses $V^\pi(s)$ as an equation relating the value of the current state to the values of reachable next states:

$$V^\pi(s) = \mathbb{E}_{a \sim \pi(\cdot|s)}\left[r(s, a) + \gamma \mathbb{E}_{s' \sim P(\cdot|s,a)}\left[V^\pi(s')\right]\right]$$

In words: "the value of state $s$ equals the expected immediate reward I get from following $\pi$, plus $\gamma$ times the expected value of where I end up."

**For a deterministic policy and environment** (your PoC case), this simplifies cleanly to:

$$V^\pi(s) = r(s, \pi(s)) + \gamma\,V^\pi(f(s, \pi(s)))$$

where $f(s, a)$ is your deterministic transition function (the env step). This is an *implicit* equation — $V^\pi$ appears on both sides. Solving it is the job of the critic.

**Why not just simulate?** You could estimate $V^\pi(s)$ by running many episodes from $s$ and averaging the returns. This works but requires enormous amounts of data. The Bellman equation lets you estimate $V^\pi$ *online* using just single-step transitions, which is far more efficient. This leads to **Temporal Difference (TD) learning** in Section 10.

---

### 8. The Action-Value Function $Q^\pi(s, a)$

The $Q$-function answers a slightly different question: "if I am in state $s$, I take action $a$ right now (even if it is not what $\pi$ would normally choose), and then follow $\pi$ for all subsequent steps — what total return do I expect?"

$$Q^\pi(s, a) = \mathbb{E}_\pi\left[G_t \;\middle|\; s_t = s, a_t = a\right] = r(s, a) + \gamma \mathbb{E}_{s' \sim P}\left[V^\pi(s')\right]$$

**The relationship between $Q^\pi$ and $V^\pi$:**

$$V^\pi(s) = \mathbb{E}_{a \sim \pi(\cdot|s)}\left[Q^\pi(s, a)\right]$$

The value of a state is the expected $Q$-value when you average over the actions the policy would take. For a deterministic policy: $V^\pi(s) = Q^\pi(s, \pi(s))$.

**Why $Q^\pi$ is more useful than $V^\pi$ for learning.** To improve a policy, you need to know whether taking a *specific action* is better or worse than average. $V^\pi(s)$ only tells you the average; $Q^\pi(s, a)$ tells you the value of each specific action. You can improve the policy greedily: $\pi'(s) = \arg\max_a Q^\pi(s, a)$.

**The Bellman equation for $Q^\pi$:**

$$Q^\pi(s, a) = r(s, a) + \gamma \mathbb{E}_{s' \sim P}\left[\mathbb{E}_{a' \sim \pi(\cdot|s')}\left[Q^\pi(s', a')\right]\right]$$

For continuous action spaces, $\arg\max_a Q^\pi(s, a)$ is intractable — you cannot enumerate all actions. This is why we need policy gradient methods rather than pure Q-learning for continuous control.

---

### 9. The Advantage Function $A^\pi(s, a)$

The advantage function is a beautifully simple idea: it measures how much better action $a$ is compared to the average action the policy would take in state $s$:

$$A^\pi(s, a) = Q^\pi(s, a) - V^\pi(s)$$

**Interpretation.** If $A^\pi(s, a) > 0$, action $a$ is better than average — the policy should take it more often. If $A^\pi(s, a) < 0$, action $a$ is worse than average — take it less. If $A^\pi(s, a) = 0$, action $a$ is exactly average.

**Why the advantage is crucial for stable learning.** The raw $Q$-value can be very large (even for a simple environment, $Q$ values might be in the hundreds). Using raw $Q$ values in gradient updates causes high variance because small policy changes can lead to wildly different $Q$ estimates. The advantage $A$ is centred around zero by construction, which dramatically reduces variance.

**Example in your null-space env.** Suppose in some state, the manipulability is currently 0.3. The policy produces action $a_1$ (null-space vector pointing toward higher-manipulability configuration) with $Q^\pi(s, a_1) = 45$, and the average $V^\pi(s) = 38$. Then $A^\pi(s, a_1) = +7$ — this action is 7 units of return better than average. The policy should be strengthened to take this action more often in similar states.

---

## Part IV — Policy Gradient Methods

---

### 10. Temporal Difference Learning — The Core Learning Mechanism

Before learning policies, let's understand how to learn value functions efficiently. This is the foundation every modern algorithm builds on.

**The TD target.** Given a single transition $(s_t, a_t, r_t, s_{t+1})$, we have two estimates of $V^\pi(s_t)$:

- **Monte Carlo estimate:** run the episode to completion and compute $G_t = r_t + \gamma r_{t+1} + \ldots$ This is unbiased (it's the true return) but has high variance (many random steps contribute).
- **TD estimate (bootstrapped):** use $r_t + \gamma V^\pi(s_{t+1})$ — the immediate reward plus the value of the next state. This is biased (because $V^\pi$ is an estimate), but has low variance.

The **TD error** $\delta_t$ is the difference between the TD estimate and the current estimate:

$$\delta_t = \underbrace{r_t + \gamma V_\phi(s_{t+1})}_{\text{TD target}} - V_\phi(s_t)$$

This is the "surprise" — by how much was your current prediction of the value wrong, given one step of actual experience?

**TD learning update.** The critic (value function approximator) is updated to reduce the TD error:

$$\phi \leftarrow \phi + \alpha_\text{critic}\, \delta_t\, \nabla_\phi V_\phi(s_t)$$

This is gradient descent on $\frac{1}{2}\delta_t^2$ with respect to the critic parameters $\phi$. The loss is $\mathcal{L}_\text{critic} = \frac{1}{2}\left(r_t + \gamma V_\phi(s_{t+1}) - V_\phi(s_t)\right)^2$.

**Important subtlety: stop the gradient through the target.** When computing the TD target $r_t + \gamma V_\phi(s_{t+1})$, you treat $V_\phi(s_{t+1})$ as a constant (detach it from the computational graph). If you didn't, you'd be chasing a moving target with your gradients, which causes instability. This is why modern implementations use a **separate target network** $V_{\phi_\text{target}}$ that is updated slowly.

---

### 11. The Policy Gradient Theorem

Now for the core theoretical result that underpins all policy gradient methods. How do you take the gradient of the RL objective — which involves an expectation over trajectories generated by the policy — with respect to the policy parameters $\theta$?

Define the objective as the expected return from the initial state distribution:

$$J(\theta) = \mathbb{E}_{s_0 \sim \rho_0, \tau \sim \pi_\theta}\left[G_0\right]$$

where $\tau = (s_0, a_0, r_0, s_1, a_1, r_1, \ldots)$ is a trajectory sampled by running $\pi_\theta$.

**The problem.** $J(\theta)$ involves an expectation over trajectories that depend on $\theta$ in a complex way — through both the action distribution and the state visitation distribution (which states the policy visits). Differentiating through the expectation is non-trivial.

**The log-derivative trick.** For any probability distribution $p_\theta(x)$:

$$\nabla_\theta p_\theta(x) = p_\theta(x)\, \nabla_\theta \log p_\theta(x)$$

This allows us to write: $\nabla_\theta \mathbb{E}_{x \sim p_\theta}[f(x)] = \mathbb{E}_{x \sim p_\theta}[f(x)\, \nabla_\theta \log p_\theta(x)]$. The expectation can now be estimated by sampling — we sample trajectories, compute the log-probability gradient, and weight by the return.

**The Policy Gradient Theorem** (Sutton et al., 2000):

$$\nabla_\theta J(\theta) = \mathbb{E}_{\tau \sim \pi_\theta}\left[\sum_{t=0}^{T} \nabla_\theta \log \pi_\theta(a_t \mid s_t)\, G_t\right]$$

**Intuitive interpretation.** For each step $t$ in the trajectory: if the return $G_t$ was high (good episode), increase the probability of the action taken, $\nabla_\theta \log \pi_\theta(a_t \mid s_t) > 0$. If the return was low, decrease it. The gradient pushes the policy toward actions that led to high returns and away from actions that led to low returns.

**The baseline trick.** The policy gradient is unbiased for any choice of baseline $b(s_t)$ subtracted from $G_t$:

$$\nabla_\theta J(\theta) = \mathbb{E}_\tau\left[\sum_t \nabla_\theta \log \pi_\theta(a_t \mid s_t)\, (G_t - b(s_t))\right]$$

Using $b(s_t) = V^\pi(s_t)$ gives us the **advantage** $(G_t - V^\pi(s_t)) \approx A^\pi(s_t, a_t)$. This reduces variance without introducing bias, which is essential for stable training.

---

### 12. REINFORCE — The Simplest Policy Gradient Algorithm

The simplest implementation of the policy gradient theorem is REINFORCE:

```
For each episode:
    1. Run the policy to get a full trajectory τ = (s_0, a_0, r_0, ..., s_T)
    2. For each step t, compute G_t = Σ_{k≥t} γ^{k-t} r_k  (return from step t)
    3. Update: θ ← θ + α Σ_t ∇_θ log π_θ(a_t|s_t) · G_t
```

**Ball-in-bowl example with REINFORCE.** Suppose the policy is a Gaussian: $\pi_\theta(a \mid s) = \mathcal{N}(\theta_0 s, \sigma^2)$ — push with force proportional to position. The log-probability is:

$$\log \pi_\theta(a \mid s) = -\frac{(a - \theta_0 s)^2}{2\sigma^2} - \log(\sigma\sqrt{2\pi})$$

$$\nabla_{\theta_0} \log \pi_\theta(a \mid s) = \frac{(a - \theta_0 s) \cdot s}{\sigma^2}$$

The gradient pushes $\theta_0$ in the direction that made the action more likely, weighted by how good the return was. If $\theta_0 = 0$ (random policy) and by chance the ball moved toward centre (positive return), the gradient would nudge $\theta_0$ toward negative (push back toward centre when $s > 0$). Over many episodes, this converges.

**Why REINFORCE is not enough for your problem.** It requires complete episodes before updating (on-policy), has extremely high variance because $G_t$ reflects many future random steps, and is very sample-inefficient. For a 200-step episode with 11D state and 4D action, you'd need millions of episodes. This motivates the Actor-Critic family.

---

### 13. Actor-Critic — Combining Policy and Value Learning

The key insight is that we can replace the high-variance return $G_t$ in the policy gradient with the lower-variance advantage estimate $A^\pi(s_t, a_t)$, computed using the critic.

**The actor-critic update:**

$$\nabla_\theta J(\theta) \approx \mathbb{E}\left[\nabla_\theta \log \pi_\theta(a_t \mid s_t)\,\cdot\, A^\pi(s_t, a_t)\right]$$

**How to estimate the advantage** from a single transition? Use the TD error as a proxy:

$$A^\pi(s_t, a_t) \approx \delta_t = r_t + \gamma V_\phi(s_{t+1}) - V_\phi(s_t)$$

This is a one-step advantage estimate. It is biased (because $V_\phi$ is an approximation) but has much lower variance than $G_t$.

**The actor-critic loop:**

```
Observe state s_t
Actor selects action a_t ~ π_θ(·|s_t)
Environment returns r_t, s_{t+1}

# Critic update
δ_t = r_t + γ V_φ(s_{t+1}) - V_φ(s_t)   ← TD error
L_critic = (1/2) δ_t²
φ ← φ - α_critic ∇_φ L_critic

# Actor update
L_actor = - log π_θ(a_t|s_t) · δ_t
θ ← θ - α_actor ∇_θ L_actor
```

Notice the minus sign in the actor loss — we're doing gradient *ascent* on the log-probability weighted by advantage. In PyTorch, this is written as a minimisation with a negative sign.

**Intuition for why this works.** The critic is constantly answering: "given where the arm is, how good a situation is this?" The actor is constantly answering: "given the critic's evaluation, what action should I take?" The critic's evaluation trains the actor, and the actor generates new experiences that train the critic. They co-evolve.

---

## Part V — Deep Dive into SAC

---

### 14. The Problem with Basic Actor-Critic — Why We Need SAC

Standard actor-critic has several painful failure modes for continuous control problems like yours.

**Overestimation of $Q$ values.** When training a $Q$-network, the actor update tries to maximise $Q(s, a)$ over the actor. But the critic learned on a finite dataset and will have incorrectly high $Q$ estimates for some out-of-distribution actions. The actor then exploits these spuriously high values, taking actions the critic hasn't seen, creating a feedback loop of overestimation and exploitation. This is called **deadly triad** behaviour and can lead to completely unstable training.

**Sample inefficiency from on-policy learning.** Standard actor-critic methods like A2C/A3C discard data after each update — they only learn from experiences collected by the *current* policy. For your 200-step episode, you throw away 200 transitions after one gradient step. This is wasteful.

**Premature convergence in under-constrained spaces.** Your null-space has 2 degrees of freedom for a 4-DOF arm — many different null-space actions are similarly valid. Without a mechanism to stay exploratory, the policy could collapse to a narrow region of the null-space, missing better configurations.

**Brittle hyperparameter sensitivity.** Standard actor-critic requires careful tuning of learning rates, entropy coefficients, and network sizes. Small changes can completely destabilise training.

SAC (Soft Actor-Critic, Haarnoja et al. 2018) addresses all four of these problems systematically.

---

### 15. The Maximum Entropy Framework — The Central Idea Behind SAC

SAC operates in the **maximum entropy RL** framework. The idea is elegant: instead of just maximising expected return, maximise expected return *plus* the entropy of the policy.

**Entropy of a distribution** $\pi(\cdot \mid s)$ measures how spread out (uncertain/exploratory) the policy is:

$$\mathcal{H}(\pi(\cdot \mid s)) = -\mathbb{E}_{a \sim \pi(\cdot|s)}\left[\log \pi(a \mid s)\right]$$

For a Gaussian with standard deviation $\sigma$, the entropy is $\frac{1}{2}\log(2\pi e \sigma^2)$ — higher $\sigma$ means higher entropy. For a completely uniform distribution, entropy is maximised. For a delta function (deterministic), entropy is $-\infty$.

**The maximum entropy RL objective:**

$$J^\text{SAC}(\pi) = \mathbb{E}_{\tau \sim \pi}\left[\sum_{t=0}^T \gamma^t \left(r(s_t, a_t) + \alpha\, \mathcal{H}(\pi(\cdot \mid s_t))\right)\right]$$

The temperature $\alpha > 0$ controls the trade-off between reward maximisation and entropy maximisation. You can think of $\alpha$ as "how much does the agent value staying uncertain/exploratory?"

**Why add entropy to the objective?**

The intuition is that an optimal *deterministic* policy is a special case — it only makes sense when you are completely confident about the consequences of actions. When there is uncertainty about the environment, or when multiple actions are similarly good (as in your null-space), maintaining a spread of probability over good actions is genuinely better than committing to one.

Mathematically, the entropy bonus prevents the policy from over-fitting to any particular action. It keeps the policy robust: even if the environment changes slightly, a more spread-out policy can adapt quickly.

**Connection to information theory.** The maximum entropy principle says: "given what you know (the reward signal), be as uncertain as possible about everything else." This is Jaynes' maximum entropy principle applied to RL. It has the effect of finding the *simplest* (least committed) policy that achieves good rewards.

---

### 16. Soft Bellman Equations — The Math of SAC

The maximum entropy framework changes the Bellman equations. Define the **soft Q-function** $Q_\text{soft}^\pi(s, a)$ as the expected return under the entropy-augmented reward:

$$Q_\text{soft}^\pi(s, a) = r(s, a) + \gamma\, \mathbb{E}_{s' \sim P}\left[V_\text{soft}^\pi(s')\right]$$

where the **soft value function** is:

$$V_\text{soft}^\pi(s) = \mathbb{E}_{a \sim \pi(\cdot|s)}\left[Q_\text{soft}^\pi(s, a) - \alpha\,\log\pi(a \mid s)\right]$$

Notice the extra $-\alpha\,\log\pi(a \mid s)$ term. This is the entropy contribution. When $\pi(a \mid s)$ is small (low probability action), $-\log\pi(a \mid s)$ is large — the agent gets a bonus for taking unlikely actions, which maintains exploration.

**The soft Bellman equation** combining these:

$$Q_\text{soft}^\pi(s, a) = r(s, a) + \gamma\, \mathbb{E}_{s'}\left[\mathbb{E}_{a' \sim \pi}\left[Q_\text{soft}^\pi(s', a') - \alpha\,\log\pi(a' \mid s')\right]\right]$$

The **optimal soft policy** (the one that maximises $J^\text{SAC}$) has a beautiful closed form:

$$\pi^*(a \mid s) = \frac{\exp\!\left(\frac{1}{\alpha} Q_\text{soft}^*(s, a)\right)}{Z(s)}$$

where $Z(s) = \int \exp\!\left(\frac{1}{\alpha} Q_\text{soft}^*(s, a)\right) da$ is a normalisation constant. This is a **Boltzmann/softmax distribution** over $Q$-values. Actions with high $Q$-values get high probability, but every action gets at least some probability. $\alpha$ controls the "temperature" of this softmax — high $\alpha$ means near-uniform distribution; low $\alpha$ means near-deterministic.

In practice, $Z(s)$ is intractable for continuous actions, which is why we parameterise the policy explicitly as a Gaussian and train it to approximate this optimal distribution.

---

### 17. The SAC Architecture — Three Networks in Detail

SAC maintains five networks (yes, five — two critics, two target critics, one actor), which sounds like a lot but each has a specific, necessary purpose.

**The Actor Network** $\pi_\theta(a \mid s)$ outputs a Gaussian distribution over actions, parameterised by mean $\mu_\theta(s)$ and log standard deviation $\log\sigma_\theta(s)$:

$$\pi_\theta(a \mid s) = \mathcal{N}(\mu_\theta(s),\, \text{diag}(\sigma_\theta(s)^2))$$

Actions are sampled as $\tilde{a} = \mu_\theta(s) + \sigma_\theta(s) \odot \epsilon$ where $\epsilon \sim \mathcal{N}(0, I)$ (the **reparameterisation trick** — more on this in a moment). The final action is squashed: $a = \tanh(\tilde{a})$, bounding it to $(-1, 1)^4$.

For your arm: the network is `MLP: [11] → [256] → [256] → [8]`, outputting 4 means and 4 log-stds.

**The Critic Networks** $Q_{\phi_1}(s, a)$ and $Q_{\phi_2}(s, a)$ each estimate the soft $Q$-value for a (state, action) pair. Using two independent critics is the **clipped double-Q** trick: when computing the TD target, take $\min(Q_{\phi_1}, Q_{\phi_2})$ — this prevents overestimation. For your arm: `MLP: [11+4] → [256] → [256] → [1]`, taking state+action concatenated.

**The Target Critic Networks** $Q_{\phi_1^\text{target}}$ and $Q_{\phi_2^\text{target}}$ are copies of the critic networks updated via **soft (exponential moving average) update**:

$$\phi_i^\text{target} \leftarrow \tau\, \phi_i + (1-\tau)\, \phi_i^\text{target}, \qquad \tau = 0.005$$

At each step, the target network weights move only 0.5% toward the current critic weights. This means the target network changes extremely slowly — it provides a stable learning target for the critic update, preventing the instability that comes from "chasing a moving target."

---

### 18. The Reparameterisation Trick — How SAC Gradients Flow

This is a technical but crucial piece. The actor update requires computing:

$$\nabla_\theta \mathbb{E}_{a \sim \pi_\theta(\cdot|s)}\left[Q(s, a) - \alpha\log\pi_\theta(a \mid s)\right]$$

The expectation is over $a \sim \pi_\theta$, which *depends on* $\theta$. You cannot simply take the gradient inside the expectation when the distribution itself depends on the parameter.

**The reparameterisation trick** sidesteps this by writing the sample as a deterministic function of the parameters and a noise variable that does not depend on $\theta$:

$$a = \tanh(\mu_\theta(s) + \sigma_\theta(s) \odot \epsilon), \quad \epsilon \sim \mathcal{N}(0, I)$$

Now $\mathbb{E}_{\epsilon \sim \mathcal{N}(0,I)}\left[Q(s, \tanh(\mu_\theta(s) + \sigma_\theta(s) \odot \epsilon))\right]$ — the expectation is over $\epsilon$, which does *not* depend on $\theta$. Gradients can flow from $Q$ directly back through the $\tanh$ and $\sigma_\theta$ into $\theta$. PyTorch's autograd handles this automatically.

**Why this is better than REINFORCE for continuous actions.** REINFORCE estimates the gradient as $\mathbb{E}[\log\pi_\theta(a|s) \cdot Q(s,a)]$ — a product of two quantities that can be noisy. The reparameterisation trick gives gradients that flow directly through $Q$ via the chain rule, which has much lower variance. For SAC with continuous actions, reparameterisation is what makes training stable.

---

### 19. The SAC Update Equations — Step by Step

Now let's put it all together. Here is one complete SAC training step, with every equation explained.

**Step 1: Sample a batch.** Sample a mini-batch of transitions $\{(s_i, a_i, r_i, s_i')\}_{i=1}^B$ from the replay buffer $\mathcal{D}$ (batch size $B = 256$ typically).

**Step 2: Update the critics.** For each critic $j \in \{1, 2\}$:

First compute the TD target. Sample a new action from the current policy at the next state:

$$\tilde{a}' \sim \pi_\theta(\cdot \mid s_i'), \qquad \tilde{a}' = \tanh(\mu_\theta(s_i') + \sigma_\theta(s_i') \odot \epsilon), \quad \epsilon \sim \mathcal{N}(0,I)$$

Compute the target $Q$ value using the soft Bellman equation with clipped double-Q:

$$y_i = r_i + \gamma \left[\min_{j=1,2} Q_{\phi_j^\text{target}}(s_i', \tilde{a}') - \alpha\,\log\pi_\theta(\tilde{a}' \mid s_i')\right]$$

The entropy term $-\alpha\,\log\pi_\theta$ increases the target when the policy is uncertain about $s_i'$ (high entropy → more exploratory future states are worth more). The $\min$ over two critics prevents overestimation.

Update each critic by minimising the Huber/MSE loss:

$$\mathcal{L}_\text{critic}(\phi_j) = \frac{1}{B}\sum_{i=1}^B \left(Q_{\phi_j}(s_i, a_i) - y_i\right)^2$$

$$\phi_j \leftarrow \phi_j - \alpha_\text{critic}\, \nabla_{\phi_j}\, \mathcal{L}_\text{critic}(\phi_j)$$

Note that $y_i$ is computed using the *target* networks and detached from the gradient graph. The original actions $a_i$ from the replay buffer (not freshly sampled) are used here.

**Step 3: Update the actor.** Sample fresh actions from the current policy at the *current* states (not next states):

$$\tilde{a}_i = \tanh(\mu_\theta(s_i) + \sigma_\theta(s_i) \odot \epsilon_i), \quad \epsilon_i \sim \mathcal{N}(0,I)$$

The actor loss is the negative of the objective to be maximised (since we do gradient descent):

$$\mathcal{L}_\text{actor}(\theta) = \frac{1}{B}\sum_{i=1}^B \left[\alpha\,\log\pi_\theta(\tilde{a}_i \mid s_i) - \min_{j=1,2} Q_{\phi_j}(s_i, \tilde{a}_i)\right]$$

The first term pushes the policy toward higher entropy (more exploration). The second term pushes the policy toward actions with higher $Q$-values. The temperature $\alpha$ balances these.

$$\theta \leftarrow \theta - \alpha_\text{actor}\, \nabla_\theta\, \mathcal{L}_\text{actor}(\theta)$$

**Step 4: Update the temperature.** If using automatic entropy tuning, update $\log\alpha$ to maintain target entropy $\mathcal{H}_\text{target} = -|\mathcal{A}| = -4$ (negative action dimension):

$$\mathcal{L}(\alpha) = -\mathbb{E}_{a \sim \pi_\theta}\left[\alpha \left(\log\pi_\theta(a \mid s) + \mathcal{H}_\text{target}\right)\right]$$

$$\log\alpha \leftarrow \log\alpha - \alpha_\text{temp}\, \nabla_{\log\alpha}\, \mathcal{L}(\alpha)$$

If the current entropy is below target (policy too deterministic), $\alpha$ increases → more entropy bonus → policy spreads out. If above target (policy too random), $\alpha$ decreases → policy sharpens. This self-correction is what makes SAC so robust to hyperparameter choice.

**Step 5: Soft update target networks:**

$$\phi_j^\text{target} \leftarrow \tau\, \phi_j + (1-\tau)\, \phi_j^\text{target}, \quad \tau = 0.005$$

This is done every step, not every episode. The slow update ensures stable TD targets.

---

### 20. The Replay Buffer — Why Off-Policy Learning Is Powerful

The replay buffer $\mathcal{D}$ is a circular queue of transitions $(s, a, r, s')$ with capacity $N$ (typically $10^5$ to $10^6$). Every time the agent takes a step, the transition is added. When $\mathcal{D}$ is full, the oldest entry is overwritten.

**Why does reusing old data work?** SAC is an *off-policy* algorithm — its theoretical guarantees hold for any data distribution in the buffer, not just data from the current policy. This is because the Bellman equation is a property of the *environment*, not the policy that collected the data. Any transition $(s, a, r, s')$ from any policy gives valid information about $Q^*(s, a)$, as long as the data covers the state-action space adequately.

**What this means practically for your null-space env.** On your first few thousand steps, the policy is random — it takes random null-space actions. These random-policy transitions are stored in the buffer. As the policy improves, good null-space behaviours start appearing — these are also stored. When training, each mini-batch samples from *all* of this history. The good transitions from later in training get mixed with the random transitions from early on.

The key advantage: each good null-space transition (one where the arm happened to achieve high manipulability and good joint positioning simultaneously) is not discarded after one gradient step — it stays in the buffer and is used for potentially hundreds of updates. This is why SAC is dramatically more sample-efficient than on-policy methods for your task.

**Buffer capacity choice.** Too small: good experiences get overwritten before being fully used. Too large: old data from a very different policy dominates. For a 200-step episode with 4D actions, $N = 10^5$ (500 episodes) is a good starting point.

---

### 21. The Log-Probability of a Squashed Gaussian — The Tricky Calculation

When SAC computes $\log\pi_\theta(a \mid s)$ for the entropy term, there is a subtlety that catches many people. The policy samples $\tilde{a}$ from a Gaussian, then applies $a = \tanh(\tilde{a})$. The log-probability of the *squashed* action involves a Jacobian correction.

**The change of variables formula.** If $a = \tanh(\tilde{a})$ and $\tilde{a} \sim \mathcal{N}(\mu, \sigma^2)$:

$$\log\pi(a \mid s) = \log \mathcal{N}(\tilde{a};\, \mu, \sigma^2) - \sum_{d=1}^{D}\log\left(1 - \tanh^2(\tilde{a}_d)\right)$$

The second term is the log-Jacobian of the $\tanh$ transformation. Since $\frac{d}{d\tilde{a}}\tanh(\tilde{a}) = 1 - \tanh^2(\tilde{a}) = \text{sech}^2(\tilde{a})$, this correction accounts for the fact that $\tanh$ squeezes probability mass near the boundaries.

**Why this matters.** Near $a_d \approx \pm 1$ (near the action bounds), $\log(1 - \tanh^2(\tilde{a}_d))$ becomes very negative — the log-probability is very low. This implicitly penalises actions near the boundary. For your null-space actions, this is actually helpful: it gently prevents the policy from saturating the null-space velocity at the boundary values, which could cause numerical issues.

---

### 22. Applied to Your System — What SAC is Actually Learning

Let's be concrete about what the five numbers output by the actor network mean for your arm.

At each step, the actor receives the 11-dimensional state $\boldsymbol{s} = [\boldsymbol{q}/\pi, \dot{\boldsymbol{q}}_\text{prev}, \boldsymbol{e}_{ee}, z_\text{phase}]$ and outputs a 4-dimensional action $\boldsymbol{a} \in [-1, 1]^4$.

This action then gets projected: $\boldsymbol{a}_\text{null} = \mathbf{N}(\boldsymbol{q})\,\boldsymbol{a}$, giving the actual null-space joint velocity. The key insight is that the policy never sees the projection — it just learns to output raw 4D vectors. Over training, it discovers that certain raw vectors, when projected, consistently lead to good null-space configurations. The policy is learning to "aim" into the null-space in a way that achieves high composite reward.

**What the Q-function is encoding.** The value $Q_\phi(s, a)$ is answering: "given that the arm is in configuration $\boldsymbol{q}$ with EE error $\boldsymbol{e}_{ee}$ and task phase $z$, and I apply raw null-space action $\boldsymbol{a}$ right now, what total discounted null-space reward can I expect for the rest of this episode?" This is a long-horizon prediction about how a current null-space decision affects future manipulability, joint-limit avoidance, and smoothness.

**What the critic is learning vs. what the actor is learning.** The critic is learning the consequences of null-space actions — it builds a model of "what happens to manipulability and joint limits over the next 100 steps if I take this action." The actor is learning to take actions that the critic thinks are good. The critic trains on real experience (from the replay buffer); the actor trains on the critic's evaluations.

---

## Part VI — Common Failure Modes and Diagnostics

---

### 23. How to Know if SAC Training is Working

The most important diagnostic is **not** the episode reward — it's the combination of several signals together.

**The Q-value should grow steadily, then plateau.** If $Q$ grows without bound, your critic is overestimating (deadly triad). Try reducing the learning rate or increasing the target network update interval.

**The actor loss should trend downward, then stabilise.** If it explodes, the actor is exploiting spurious Q-values. If it never decreases, the Q-function is not learning meaningful signals.

**The entropy should start high (near $\mathcal{H}_\text{target}$) and may decrease slightly over training** as the policy finds good null-space behaviours and becomes more committed. If entropy collapses to near zero early in training, the temperature $\alpha$ is too low — the policy has gone deterministic before finding good strategies.

**EE error should stay low and not be correlated with training progress.** If EE error increases as null-space reward increases, something is wrong with your null-space projection implementation (the CLIK and RL contributions are interfering).

**For your specific task:** watch manipulability as the primary indicator of learning. It should increase monotonically on average over training. A good trained policy for a 4-DOF planar arm should achieve mean manipulability above 0.6 (compared to ~0.2-0.3 for a random policy).

---

### 24. The Connection Between RL and Your Paper's Claims

To close the loop on theory: here is exactly what each part of the SAC algorithm contributes to your paper's experimental claims.

**Claim H3** ("learned policy beats fixed baselines on composite reward") relies on the $Q$-function learning a *multi-step, multi-objective* prediction. The classical baselines use single-step gradients of individual objectives. SAC's $Q$-function implicitly models all three reward components (manipulability, joint limits, smoothness) simultaneously and over 100+ steps. This is why it can find configurations that the single-objective gradients cannot.

**Claim H4** ("task conditioning helps") relies on the state input $z_\text{phase}$ being visible to both the actor and critic. The $Q$-function $Q_\phi(s, a)$ with $z_\text{phase}$ in $s$ learns that the same null-space action has different long-term value depending on the phase. During APPROACH, moving toward high manipulability is valuable (phase changes soon, need good grasp pose). During HOLD, it's less valuable (manipulability doesn't matter as much for holding). The critic learns this automatically from the reward signal.

**Claim H5** ("null-space RL is more sample-efficient than end-to-end") is a consequence of the constrained action space. An end-to-end RL agent has to simultaneously learn to track the EE *and* manage joint configurations. These are competing signals early in training. Your null-space agent only has to learn to use the null-space — the EE is handled by CLIK and never appears in the reward. Fewer goals = faster convergence.

---

## Quick Reference — All Key RL Formulas

This table collects every formula from the primer in one place. The third column connects each formula to your specific system.

| Formula | Name | In Your System |
|---|---|---|
| $G_t = \sum_{k\geq 0} \gamma^k r_{t+k}$ | Discounted return | Total null-space quality from step $t$ |
| $G_t = r_t + \gamma G_{t+1}$ | Bellman recursion | The "bootstrap" that enables TD learning |
| $V^\pi(s) = \mathbb{E}_\pi[G_t \mid s_t = s]$ | State value | Expected null-space quality from configuration $s$ |
| $Q^\pi(s,a) = \mathbb{E}_\pi[G_t \mid s_t = s, a_t = a]$ | Action-value | Expected quality if I take null-space action $a$ from $s$ |
| $A^\pi(s,a) = Q^\pi(s,a) - V^\pi(s)$ | Advantage | How much better is $a$ than what $\pi$ normally does? |
| $\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)$ | TD error | Surprise: did this step beat or disappoint expectations? |
| $\nabla_\theta J = \mathbb{E}[\nabla_\theta\log\pi_\theta(a\mid s)\cdot A]$ | Policy gradient | Push policy toward good null-space actions |
| $\mathcal{H}(\pi) = -\mathbb{E}[\log\pi(a\mid s)]$ | Entropy | Spread of policy over null-space — keep this non-zero |
| $J^\text{SAC} = \mathbb{E}[\sum_t r_t + \alpha\mathcal{H}(\pi(\cdot\mid s_t))]$ | SAC objective | Null-space reward + exploration bonus |
| $y_i = r_i + \gamma[\min_j Q_{\phi_j^\text{target}}(s',\tilde{a}') - \alpha\log\pi(\tilde{a}'\mid s')]$ | SAC TD target | Bootstrap target with entropy, clipped for stability |
| $\mathcal{L}_\text{critic} = (Q_\phi(s,a) - y)^2$ | Critic loss | MSE between predicted and target Q-value |
| $\mathcal{L}_\text{actor} = \alpha\log\pi(\tilde{a}\mid s) - \min_j Q_{\phi_j}(s,\tilde{a})$ | Actor loss | Entropy - Q-value (minimise to maximise Q + entropy) |
| $\phi^\text{target} \leftarrow \tau\phi + (1-\tau)\phi^\text{target}$ | Soft target update | Slowly track critic weights for stable TD targets |
| $a = \tanh(\mu_\theta(s) + \sigma_\theta(s)\odot\epsilon)$ | Reparameterisation | Differentiable sampling for actor gradient |

---

*End of RL Primer. The next step is to read Section 22 once more while looking at the SB3 SAC source code — match every variable name in the code to a formula in this document. Once you can do that, you understand SAC fully.*