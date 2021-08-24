import cbsrl
import gym
from gym import spaces
import numpy

class CBSEnv(gym.Env):

    def __init__(self) -> None:
        super().__init__()
        #self.action_space = spaces.Tuple([spaces.Discrete(2),spaces.Discrete(20)])
        self.action_space = spaces.Discrete(40)
        self.observation_space = spaces.Box(low=-1, high=20, shape=(3, 100), dtype=int)
        self.cbs = cbsrl.CBSHRL()
        self.state = numpy.array(self.cbs.getstate()).reshape(1, 3, 10, 10).astype(numpy.float32)
        #if self.cbs.isdone():
        self.reward = -self.cbs.getreward()
        #else:
        #    self.reward = 0
        print("init: ", self.reward)
        self.done = self.cbs.isdone()

    def reset(self):
        self.cbs.reset()
        return self.state

    def step(self, action):
        agent = action // 20
        loc = action % 20
        self.reward = -self.cbs.step(agent, loc)
        print("agent: ", agent, "loc: ", loc)
        print("reward", self.reward)
        self.state = numpy.array(self.cbs.getstate()).reshape(1, 3, 10, 10).astype(numpy.float32)
        self.done = self.cbs.isdone()
        #if self.reward > -45:
        #    print(self.state)
        #    wait()
        return self.state, self.reward, self.done, {}

    def getvalidaction(self):
        return self.cbs.getvalidaction()
