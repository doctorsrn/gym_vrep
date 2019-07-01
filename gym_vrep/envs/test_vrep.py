import argparse

import gym
import numpy as np

from stable_baselines.deepq import DQN, MlpPolicy
from vrep_env import Vrep_Env
from gym import logger

def main(args):
    """
    Run a trained model for the vrep

    :param args: (ArgumentParser) the input arguments
    """
    env = gym.make('vrep-v0')
    model = DQN.load("./model/bk1/cartpole_model3000.pkl", env)

    for _ in range(10):
        obs, done = env.reset(), False
        episode_rew = 0
        while not done:
            action, _ = model.predict(obs)
            obs, rew, done, _ = env.step(action)
            print('action: %d, reward: %f' %(action, rew))
            episode_rew += rew
        print("Episode reward", episode_rew)
        # No render is only used for automatic testing
        if args.no_render:
            break
   

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Enjoy trained DQN on V-rep")
    parser.add_argument('--no-render', default=False, action="store_true", help="Disable rendering")
    args = parser.parse_args()
    main(args)
