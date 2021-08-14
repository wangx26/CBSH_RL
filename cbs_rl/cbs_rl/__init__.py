from gym.envs.registration import register

register(
    id='cbs-v0',
    entry_point='cbs_rl.envs:CBSEnv',
    max_episode_steps = 1000,
    reward_threshold = 30.0,
)
register(
    id='cbs-extrahard-v0',
    entry_point='cbs_rl.envs:CBSExtraHardEnv',
)