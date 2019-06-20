## Introduction
Using robot simulator V-REP to creat a reinforcement environment for dual-arm robot like Gym based on b0RemoteApi from V-rep, and can use OpenAI [baseline](https://github.com/openai/baselines/) or [stable-baseline](https://github.com/hill-a/stable-baselines) to train RL model.

Reference web: 
+ https://github.com/openai/gym/blob/master/docs/creating-environments.md
+ https://zhuanlan.zhihu.com/p/33553076
+ https://github.com/openai/gym/blob/master/gym/envs/classic_control/cartpole.py
+ https://github.com/apoddar573/Tic-Tac-Toe-Gym_Environment/blob/master/gym-tictac4/gym_tictac4/envs/tictac4_env.py
+ https://github.com/openai/gym-soccer/blob/master/gym_soccer/envs/soccer_env.py


## Environment
+ V-rep 3.6.1 rev3
  
other requirements:
+ dynamics libraries `*.dll` files from v-rep directory.

## Install
1) `cd gym_vrep`
2) `pip install -e .`


## Usage
a usage: [train_vrep.py](https://github.com/doctorsrn/gym_vrep/blob/master/gym_vrep/envs/train_vrep.py)


```python
import argparse
import gym
import numpy as np

def main(args):
    env = gym.make('vrep-v0')
    model = DQN(
        env=env,
        policy=MlpPolicy,
        verbose=1,
        learning_rate=1e-5,
        buffer_size=6000,
        train_freq=1,
        learning_starts=100,
        batch_size=32,
        checkpoint_freq=300,
        checkpoint_path='./model',
        target_network_update_freq=300,
        prioritized_replay=True,
        exploration_fraction=0.1,
        exploration_final_eps=0.02,
        tensorboard_log='./log',
    )
    model.learn(total_timesteps=args.max_timesteps)

    print("Saving model to slab_installing_model.pkl")
    model.save("slab_installing_model.pkl")
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Train DQN on cartpole")
    parser.add_argument('--max-timesteps', default=15000, type=int, help="Maximum number of timesteps")
    args = parser.parse_args()
    main(args)
```

## More
not finished yet...