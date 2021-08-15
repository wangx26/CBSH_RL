import gym, torch, numpy as np, torch.nn as nn
from torch.utils.tensorboard import SummaryWriter
from tianshou.utils import BasicLogger
import tianshou as ts

task = 'cbs_rl:cbs-v0'
#task = 'CartPole-v0'
lr = 5e-4
gamma = 0.9
n_step = 4
eps_train, eps_test = 0.1, 0.05
epoch = 10
step_per_epoch = 10000
step_per_collect = 10
target_freq = 320
batch_size = 64
train_num, test_num = 10, 100
buffer_size = 20000
writer = SummaryWriter('log/dqn')
logger = BasicLogger(writer)

# 也可以用 SubprocVectorEnv
train_envs = ts.env.DummyVectorEnv([
    lambda: gym.make(task) for _ in range(train_num)])
test_envs = ts.env.DummyVectorEnv([
    lambda: gym.make(task) for _ in range(test_num)])

class Net(nn.Module):
    def __init__(self, state_shape, action_shape):
        super().__init__()
        self.model = nn.Sequential(*[
            nn.Linear(np.prod(state_shape), 256),
            nn.ReLU(inplace=True),
            nn.Linear(256, 256), nn.ReLU(inplace=True),
            nn.Linear(256, 256), nn.ReLU(inplace=True),
            nn.Linear(256, np.prod(action_shape))
        ])
    def forward(self, s, state=None, info={}):
        if not isinstance(s, torch.Tensor):
            s = torch.tensor(s, dtype=torch.float)
        print("s: ", s)
        batch = s.shape[0]
        logits = self.model(s.view(batch, -1))
        return logits, state

env = gym.make(task)
state_shape = env.observation_space.shape #or env.observation_space.n
action_shape = (env.action_space[0].n, env.action_space[1].n)
#action_shape = env.action_space.shape or env.action_space.n
net = Net(state_shape, action_shape)
optim = torch.optim.Adam(net.parameters(), lr=lr)

policy = ts.policy.DQNPolicy(
    net, optim, gamma, n_step,
    target_update_freq=target_freq)
train_collector = ts.data.Collector(
    policy, train_envs, ts.data.VectorReplayBuffer(buffer_size, train_num),
    exploration_noise=True)
test_collector = ts.data.Collector(policy, test_envs, exploration_noise=True)

result = ts.trainer.offpolicy_trainer(
    policy, train_collector, test_collector, max_epoch=epoch,
    step_per_epoch=step_per_epoch, step_per_collect=step_per_collect,
    update_per_step=1 / step_per_collect, episode_per_test=test_num,
    batch_size=batch_size, logger=ts.utils.BasicLogger(writer),
    train_fn=lambda epoch, env_step: policy.set_eps(0.1),
    test_fn=lambda epoch, env_step: policy.set_eps(0.05),
    stop_fn=lambda mean_rewards: mean_rewards >= env.spec.reward_threshold)
print(f'Finished training! Use {result["duration"]}')

torch.save(policy.state_dict(), 'dqn.pth')
policy.load_state_dict(torch.load('dqn.pth'))

policy.eval()
policy.set_eps(0.05)
collector = ts.data.Collector(policy, env, exploration_noise=True)
collector.collect(n_episode=1, render=1 / 35)