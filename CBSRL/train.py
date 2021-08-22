from multiprocessing import process

from torch import optim
from src.parameters import *
import torch
import os
import shutil
import torch.multiprocessing as _mp
from src.env import create_train_env
from src.model import ActorCritic
from src.optimizer import GlobalAdam
from src.process import local_train, local_test

def train():
    torch.manual_seed(123)
    if os.path.isdir(log_path):
        shutil.rmtree(log_path)
    os.makedirs(log_path)
    if not os.path.isdir(save_path):
        os.makedirs(save_path)
    mp = _mp.get_context("spawn")
    env, num_state, num_actions = create_train_env()
    global_model = ActorCritic(num_state, num_actions)
    if use_gpu:
        global_model.cuda()
    global_model.share_memory()
    if load_from_pre:
        pass
    optimizer = GlobalAdam(global_model.parameters(), lr=LR)
    processes = []
    for index in range(num_processes):
        if index == 0:
            process = mp.Process(target=local_train, args=(index, global_model, optimizer, True))
        else:
            process = mp.Process(target=local_train, args=(index, global_model, optimizer))
        process.start()
        processes.append(process)
    process = mp.Process(target=local_test, args=(num_processes, global_model))
    process.start()
    processes.append(process)
    for process in processes:
        process.join()


if __name__ == "__main__":
    train()
