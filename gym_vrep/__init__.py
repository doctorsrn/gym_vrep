from gym.envs.registration import register

register(
    id='vrep-v0',
    entry_point='gym_vrep.envs:Vrep_Env',
)
