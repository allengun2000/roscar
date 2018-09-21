#!/usr/bin/env python

import numpy as np
from car_env import CarEnv
from DDPG import DDPG
import matplotlib.pyplot as plt
import time
import math


DISPLAY_REWARD_THRESHOLD=200
env = CarEnv()
s_dim = 2+env.O_LC
a_dim = 1
a_bound = env.action_bound[1]
ddpg = DDPG(a_dim, s_dim, a_bound)
ON_TRAIN=False
t1 = time.time()
GLOBAL_RUNNING_R=[]
def veiw_Qtable():
    q_table=np.zeros((10,10))
    for i in range(10):
        for j in range(10):
            s=np.hstack((np.array([j*40,360-i*40],dtype=np.float32)/400,\
                         np.array(env.obs_line([j*40,360-i*40]),dtype=np.float32)/400))
            a=ddpg.choose_action(s)
            q_table[i][j]=math.degrees(a)
    print(np.round(q_table,1))
    
def train():
 
    RENDER=False
    var = 1
    MEMORY_CAPACITY = 10000
    MAX_EPISODES = 1000
    MAX_EP_STEPS = 100
    goood_job=0
    ddpg.restore()
    for i in range(MAX_EPISODES):
        s=env.reset()/400
        ep_reward = 0
        for j in range(MAX_EP_STEPS):
            if RENDER:
                env.render()
    
            # Add exploration noise
            a = ddpg.choose_action(s)
            a = np.clip(np.random.normal(a, var), *env.action_bound) 
            s_, r, done  = env.step(a)
            ddpg.store_transition(s, a, r / 10, s_)
            if ddpg.pointer > MEMORY_CAPACITY:
                var *= .9995    # decay the action randomness
                ddpg.learn()
    
            s = s_.copy()
            
            ep_reward += r
            if j == MAX_EP_STEPS-1 or done==True :
                print('Episode:', i, ' Reward: %i' % int(ep_reward), 'Explore: %.2f' % var, )
                if len(GLOBAL_RUNNING_R) == 0: GLOBAL_RUNNING_R.append(ep_reward)
                else: GLOBAL_RUNNING_R.append(GLOBAL_RUNNING_R[-1]*0.9+ep_reward*0.1)
    #            veiw_Qtable()
                if ep_reward>30 and i>500 :
                    RENDER = False
                    var=0
                #    veiw_Qtable()
                    goood_job+=1
                    print(goood_job)
                else:
                    if goood_job>0:
                        goood_job-=1
                break
#        if goood_job>20:
#            break
    plt.plot(np.arange(len(GLOBAL_RUNNING_R)), GLOBAL_RUNNING_R)
    plt.xlabel('Episode'); plt.ylabel('Moving reward'); plt.ion(); plt.show()
    print('Running time: ', time.time() - t1)
    ddpg.save()

def eval_():
    ddpg.restore()
    env.render()
    env.viewer.set_vsync(True)
    s = env.reset()
    while True:
        s=env.reset()/400
        ep_reward = 0
        for j in range(100):
            env.render()
            a = ddpg.choose_action(s)
            s, r, done = env.step(a)
            ep_reward += r
            if j == 99 or done==True :
                    print(' Reward: %i' % int(ep_reward) )
                  #  veiw_Qtable()
                    break

if ON_TRAIN:
    train()
else:
    eval_()
