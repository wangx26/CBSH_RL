import gym, torch, numpy as np, torch.nn as nn
from torch.utils.tensorboard import SummaryWriter
import tianshou as ts

task = 'cbs_rl:cbs-v0'
#task = 'CartPole-v0'
lr = 1e-3
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

# 也可以用 SubprocVectorEnv
train_envs = ts.env.DummyVectorEnv([
    lambda: gym.make(task) for _ in range(train_num)])
test_envs = ts.env.DummyVectorEnv([
    lambda: gym.make(task) for _ in range(test_num)])

class Recurrent(nn.Module):
    def __init__(self, state_shape, action_shape):
        super().__init__()
        self.fc1 = nn.Linear(np.prod(state_shape), 128)
        self.nn = nn.LSTM(input_size=128, hidden_size=128,
                          num_layers=3, batch_first=True)
        self.fc2 = nn.Linear(128, np.prod(action_shape))

    def forward(self, s, state=None, info={}):
        if not isinstance(s, torch.Tensor):
            s = torch.tensor(s, dtype=torch.float)
        # s [bsz, len, dim] (training)
        # or [bsz, dim] (evaluation)
        if len(s.shape) == 2:
            bsz, dim = s.shape
            length = 1
        else:
            bsz, length, dim = s.shape
        s = self.fc1(s.view([bsz * length, dim]))
        s = s.view(bsz, length, -1)
        self.nn.flatten_parameters()
        if state is None:
            s, (h, c) = self.nn(s)
        else:
            # we store the stack data with [bsz, len, ...]
            # but pytorch rnn needs [len, bsz, ...]
            s, (h, c) = self.nn(s, (
                state['h'].transpose(0, 1).contiguous(),
                state['c'].transpose(0, 1).contiguous()))
        s = self.fc2(s[:, -1])
        # make sure the 0-dim is batch size: [bsz, len, ...]
        return s, {'h': h.transpose(0, 1).detach(),
                   'c': c.transpose(0, 1).detach()}

env = gym.make(task)
state_shape = env.observation_space.shape #or env.observation_space.n
action_shape = (env.action_space[0].n, env.action_space[1].n)
#action_shape = env.action_space.shape or env.action_space.n
net = Recurrent(state_shape, action_shape)
optim = torch.optim.Adam(net.parameters(), lr=lr)

policy = ts.policy.DQNPolicy(
    net, optim, gamma, n_step,
    target_update_freq=target_freq)
train_collector = ts.data.Collector(
    policy, train_envs,
    ts.data.VectorReplayBuffer(buffer_size, train_num, stack_num=4),
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