import argparse

import gym
import numpy as np

from stable_baselines.deepq import DQN, MlpPolicy
from vrep_env import Vrep_Env
from gym import logger

from stable_baselines.deepq.policies import FeedForwardPolicy

# Custom MLP policy of two layers of size 32 each
class CustomPolicy(FeedForwardPolicy):
    def __init__(self, *args, **kwargs):
        super(CustomPolicy, self).__init__(*args, **kwargs,
                                           layers=[64, 64,64],
                                           layer_norm=False,
                                           feature_extraction="mlp")

def callback(lcl, _glb):
    """
    The callback function for logging and saving

    :param lcl: (dict) the local variables
    :param _glb: (dict) the global variables
    :return: (bool) is solved
    """
    #print(lcl['self'])
    # stop training if reward exceeds 199
    # if len(lcl['episode_rewards'][-101:-1]) == 0:
    #     mean_100ep_reward = -np.inf
    # else:
    #     mean_100ep_reward = round(float(np.mean(lcl['episode_rewards'][-101:-1])), 1)
    # is_solved = lcl['self'].num_timesteps > 100 and mean_100ep_reward >= 199
    # logger.set_level(20)
    # logger.info('num timesteps: %d'%(lcl['self'].num_timesteps))

    if lcl['self'].num_timesteps % lcl['self'].checkpoint_freq == 0:
        lcl['self'].save(lcl['self'].checkpoint_path+'cartpole_model'+str(lcl['self'].num_timesteps)+'.pkl')
    return True


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
        gamma=0.95,
        policy=MlpPolicy,
        #policy=CustomPolicy,
        verbose=1,
        learning_rate=1e-4,
        buffer_size=50000,  #5000
        train_freq=1,
        learning_starts=100,
        batch_size=64, # 32
        checkpoint_freq=3000,
        checkpoint_path='./model/',
        target_network_update_freq=300,
        prioritized_replay=True,
        exploration_fraction=0.1,
        exploration_final_eps=0.02,
        tensorboard_log='./log',
    )
    # path = './model/'
    # model = DQN.load(path+'bk2_16/cartpole_model6000.pkl', env, tensorboard_log='./log')
    model.learn(total_timesteps=args.max_timesteps, callback=callback, log_interval=30)

    print("Saving model to slab_installing_model.pkl")
    model.save("slab_installing_model.pkl")



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Train DQN on cartpole")
    parser.add_argument('--max-timesteps', default=60000, type=int, help="Maximum number of timesteps")
    args = parser.parse_args()
    main(args)
