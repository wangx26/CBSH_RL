import gym

def create_train_env():
    env = gym.make("cbs_rl:cbs-v0")
    print("action len: ", env.action_space.n)
    return env, env.observation_space.shape[0], env.action_space.n