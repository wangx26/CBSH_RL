from src.env import create_train_env
from src.parameters import *
import torch
import timeit
from tensorboardX import SummaryWriter
from src.model import ActorCritic
import torch.nn.functional as F
from torch.distributions import Categorical
from collections import deque

def local_train(index, global_model, optimizer, save=False):
    print("Process {} build".format(index))
    torch.manual_seed(123 + index)
    if save:
        start_time = timeit.default_timer()
    writer = SummaryWriter(log_path)
    env, num_state, num_actions = create_train_env()
    local_model = ActorCritic(num_state, num_actions)
    if use_gpu:
        local_model.cuda()
    local_model.train()
    state = torch.from_numpy(env.reset())
    if use_gpu:
        state = state.cuda()
    done = True
    curr_step = 0
    curr_episode = 0
    while True:
        if save:
            if curr_episode % save_interval == 0 and curr_episode > 0:
                torch.save(global_model.state_dict(), "{}/CBSRL".format(save_path))
                print("Process {}. Episode {}".format(index, curr_episode))
            curr_episode += 1
            local_model.load_state_dict(global_model.state_dict())
            if done:
                h_0 = torch.zeros((1, 512), dtype=torch.float)
                c_0 = torch.zeros((1, 512), dtype=torch.float)
            else:
                h_0 = h_0.detach()
                c_0 = c_0.detach()
            if use_gpu:
                h_0 = h_0.cuda()
                c_0 = c_0.cuda()
            log_policies = []
            values = []
            rewards = []
            entropies = []
            for _ in range(num_local_steps):
                curr_step += 1
                logits, value, h_0, c_0 = local_model(state, h_0, c_0)
                policy = F.log_softmax(logits, dim=1)
                log_policy = F.log_softmax(logits, dim=1)
                entropy = -(policy * log_policy).sum(1, keepdim=True)

                m = Categorical(policy)
                action = m.sample().item()
                print("Process {}. Episode {}, action {}".format(index, curr_episode, action))

                state, reward, done, _ = env.step(action)
                state = torch.from_numpy(state)
                if use_gpu:
                    state = state.cuda()
                if curr_step > num_global_steps:
                    done = True

                if done:
                    curr_step = 0
                    state = torch.from_numpy(env.reset())
                    if use_gpu:
                        state = state.cuda()

                values.append(value)
                log_policies.append(log_policy[0, action])
                rewards.append(reward)
                entropies.append(entropy)

                if done:
                    break

            R = torch.zeros((1, 1), dtype=torch.float)
            if use_gpu:
                R = R.cuda()
            if not done:
                _, R, _, _ = local_model(state, h_0, c_0)

            gae = torch.zeros((1, 1), dtype=torch.float)
            if use_gpu:
                gae = gae.cuda()
            actor_loss = 0
            critic_loss = 0
            entropy_loss = 0
            next_value = R

            for value, log_policy, reward, entropy in list(zip(values, log_policies, rewards, entropies))[::-1]:
                gae = gae * gamma * tau
                gae = gae + reward + gamma * next_value.detach() - value.detach()
                next_value = value
                actor_loss = actor_loss + log_policy * gae
                R = R * gamma + reward
                critic_loss = critic_loss + (R - value) ** 2 / 2
                entropy_loss = entropy_loss + entropy

            total_loss = -actor_loss + critic_loss - beta * entropy_loss
            writer.add_scalar("Train_{}/loss".format(index), total_loss, curr_episode)
            optimizer.zero_grad()
            total_loss.backward()

            for local_param, global_param in zip(local_model.parameters(), global_model.parameters()):
                if global_param.grad is not None:
                    break
                global_param._grad = local_param.grad

            optimizer.step()

            if curr_episode == int(num_global_steps / num_local_steps):
                print("Training process {} terminated".format(index))
                if save:
                    end_time = timeit.default_timer()
                    print("The code runs for %.2f s " % (end_time - start_time))
                return

def local_test(index, global_model):
    torch.manual_seed(123 + index)
    env, num_states, num_actions = create_train_env()
    local_model = ActorCritic(num_states, num_actions)
    local_model.eval()
    state = torch.from_numpy(env.reset())
    done = True
    curr_step = 0
    actions = deque(maxlen=max_actions)
    while True:
        curr_step += 1
        if done:
            local_model.load_state_dict(global_model.state_dict())
        with torch.no_grad():
            if done:
                h_0 = torch.zeros((1, 512), dtype=torch.float)
                c_0 = torch.zeros((1, 512), dtype=torch.float)
            else:
                h_0 = h_0.detach()
                c_0 = c_0.detach()

        logits, value, h_0, c_0 = local_model(state, h_0, c_0)
        policy = F.softmax(logits, dim=1)
        action = torch.argmax(policy).item()
        state, reward, done, _ = env.step(action)
        #env.render()
        actions.append(action)
        if curr_step > num_global_steps or actions.count(actions[0]) == actions.maxlen:
            done = True
        if done:
            print("Test done, reward: ", reward)
            curr_step = 0
            actions.clear()
            state = env.reset()
        state = torch.from_numpy(state)