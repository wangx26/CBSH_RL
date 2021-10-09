import cbsrl
import gym
from gym import spaces
import numpy

class CBSEnv(gym.Env):

    def __init__(self) -> None:
        super().__init__()
        self.action_space = spaces.Discrete(2)
        self.observation_space = spaces.Box(low=-1, high=20, shape=(11, 1024), dtype=int)
        self.cbs = cbsrl.CBSHRL()
        self.state = numpy.array(self.cbs.getstate()).reshape(1, 11, 32, 32).astype(numpy.float32)
        self.reward = -self.cbs.getreward()
        print("init: ", self.reward)
        self.done = self.cbs.isdone()

    def reset(self, reload):
        self.cbs.reset(reload)
        self.state = numpy.array(self.cbs.getstate()).reshape(1, 11, 32, 32).astype(numpy.float32)
        return self.state

    def step(self, action):
        self.reward = -self.cbs.stepLorR(action)
        self.state = numpy.array(self.cbs.getstate()).reshape(1, 11, 32, 32).astype(numpy.float32)
        self.done = self.cbs.isdone()
        return self.state, self.reward, self.done, {}

    def getvalidaction(self):
        return self.cbs.getvalidaction()

    def isdone(self):
        return self.cbs.isdone()


def create_train_env():
    env = CBSEnv()
    print("action len: ", env.action_space.n)
    return env, env.observation_space.shape[0], env.action_space.n