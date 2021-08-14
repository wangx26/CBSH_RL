import cbsrl
import gym
from gym import spaces

class CBSEnv(gym.Env):

    def __init__(self) -> None:
        super().__init__()
        self.action_space = spaces.Tuple([spaces.Discrete(2),spaces.Discrete(20)])
        self.observation_space = spaces.Box(low=-1, high=20, shape=(1, 135), dtype=int)
        self.cbs = cbsrl.CBSHRL()
        self.state = self.cbs.getstate()
        if self.cbs.isdone():
            self.reward = self.cbs.getreward()
        else:
            self.reward = 0
        print("init: ", self.cbs.getreward())
        self.done = self.cbs.isdone()

    def reset(self):
        return self.state
    def step(self, action):
        agent = action // 20
        loc = action % 20
        self.reward = self.cbs.step(agent, loc)
        print("agent: ", agent, "loc: ", loc)
        print("reward", self.reward)
        self.state = self.cbs.getstate()
        self.done = self.cbs.isdone()
        return self.state, self.reward, self.done, {}
