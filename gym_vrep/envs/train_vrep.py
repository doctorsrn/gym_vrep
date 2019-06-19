import argparse

import gym
import numpy as np

from stable_baselines.deepq import DQN, MlpPolicy
from vrep_env import Vrep_Env


def callback(lcl, _glb):
    """
    The callback function for logging and saving

    :param lcl: (dict) the local variables
    :param _glb: (dict) the global variables
    :return: (bool) is solved
    """
    # stop training if reward exceeds 199
    if len(lcl['episode_rewards'][-101:-1]) == 0:
        mean_100ep_reward = -np.inf
    else:
        mean_100ep_reward = round(float(np.mean(lcl['episode_rewards'][-101:-1])), 1)
    is_solved = lcl['self'].num_timesteps > 100 and mean_100ep_reward >= 199
    return not is_solved


def main(args):
    """
    Train and save the DQN model, for the cartpole problem

    :param args: (ArgumentParser) the input arguments
    """
    
    # env = gym.make("CartPole-v0")
    # model = DQN(
    #     env=env,
    #     policy=MlpPolicy,
    #     verbose=1,
    #     learning_rate=1e-3,
    #     buffer_size=50000,
    #     exploration_fraction=0.1,
    #     exploration_final_eps=0.02,
    #     tensorboard_log='./log',
    # )
    # model.learn(total_timesteps=args.max_timesteps, callback=callback)

    # print("Saving model to cartpole_model.pkl")
    # model.save("cartpole_model.pkl")

    # env = Vrep_Env()
    env = gym.make('vrep-v0')

    model = DQN(
        env=env,
        policy=MlpPolicy,
        verbose=1,
        learning_rate=1e-3,
        buffer_size=3,
        batch_size=1,
        checkpoint_freq=2,
        checkpoint_path='./model',
        prioritized_replay=True,
        exploration_fraction=0.1,
        exploration_final_eps=0.02,
        tensorboard_log='./log',
    )
    model.learn(total_timesteps=args.max_timesteps)



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Train DQN on cartpole")
    parser.add_argument('--max-timesteps', default=20, type=int, help="Maximum number of timesteps")
    args = parser.parse_args()
    main(args)
