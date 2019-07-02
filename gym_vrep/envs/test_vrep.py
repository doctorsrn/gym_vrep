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
    model = DQN.load("./model/cartpole_model3000.pkl", env)
    model_index = [x for x in range(3000,54001,3000)]
    #model = DQN.load("./model/bk1/cartpole_model3000.pkl", env, tensorboard_log='./log')
    for index in model_index:
        model = DQN.load("./model/cartpole_model"+str(index)+".pkl", env)

        all_espisode_rewards = []
        all_try_steps = []
        all_success = []

        for i in range(100):
            obs, done = env.reset(), False
            episode_rew = 0
            try_steps = 0
            while not done:
                action, _ = model.predict(obs)
                obs, rew, done, _ = env.step(action)
                # print('action: %d, reward: %f, counts: %d' %(action, rew, env.counts))
                episode_rew += rew
                try_steps += 1
                if done and rew > 0:
                    success = 1
                else:
                    success = 0
            print("Episode reward: %d, try_steps:%d, if success:%d "%(episode_rew, try_steps, success))
            all_espisode_rewards.append(episode_rew)
            if success:
                all_try_steps.append(try_steps)
            else:
                all_try_steps.append(30)
            all_success.append(success)

            # No render is only used for automatic testing
            if args.no_render:
                break
        # average reward, steps, and success rate
        ave_rew = sum(all_espisode_rewards) / len(all_espisode_rewards)
        ave_step = sum(all_try_steps) / len(all_try_steps)
        success_rate = sum(all_success) / len(all_success)
        print("ave_rew:%f, ave_step:%f, success_rate:%f "%(ave_rew, ave_step, success_rate))
        res = ','.join([str(ave_rew), str(ave_step), str(success_rate)])
        with open('./log/test_results.txt', 'a') as f:
            f.write(res+'\n')
        
        env.reset_simulation()
    env.close()
   

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Enjoy trained DQN on V-rep")
    parser.add_argument('--no-render', default=False, action="store_true", help="Disable rendering")
    args = parser.parse_args()
    main(args)
