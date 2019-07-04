#!/usr/bin/env python
import numpy as np
# from vehicle import CarEnv
from DDPG import DDPG
import matplotlib.pyplot as plt
import time
import math


###ROS library
import rospy
#import math
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose2D

# DISPLAY_REWARD_THRESHOLD=200
#s_dim = 5+env.O_LC
s_dim = 4
a_dim = 1
a_bound =1
ddpg = DDPG(a_dim, s_dim, a_bound)
ON_TRAIN=False
#ON_TRAIN=True
t1_ = time.time()
GLOBAL_RUNNING_R=[]
state_=np.array([0,0,0,0])
state =np.array([0,0,0,0])
reward=0
done=0
var=1
def callback(data):
     global state
     global state_
     global reward
     global done
    #  global a
     state=state_
     msg_input=np.array((data.data[0:4]),dtype=np.float64)
     reward=data.data[4]
     done=data.data[5]
     state_=np.array([msg_input[0]/5,msg_input[1]/60,msg_input[2]/28,msg_input[3]/28])
    #  a=msg_input[3]
    #  a=msg_input[3]/28
    #  print(reward)

def ros_robot():
    pub = rospy.Publisher('/cmd', Pose2D, queue_size=10)
    ddpg.restore()
    cmd_x_y=Pose2D()
    rospy.Subscriber("/state_reward", Float64MultiArray, callback)
    rospy.init_node('car_matlab', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    wheel=0
    var=1
    MEMORY_CAPACITY = 1000
    ep_reward=0
    ep_reward_b=0
    good_time=0
    a=np.zeros(1)
    while not rospy.is_shutdown():


            a = ddpg.choose_action(state)
            print(state_[3],a)
            a = np.random.normal(a*28, var)
            a = np.clip(a, -28,28)
            ddpg.store_transition(state, a/28, reward , state_)
            # ddpg.store_transition_sp(state_, state_[3])
            print(state, a/28, reward , state_)
            if ddpg.pointer > MEMORY_CAPACITY:
                var *= .995    # decay the action randomness
                ddpg.learn()
                # print(ddpg.s_learn())
            cmd_x_y.x=10
            if a<wheel:
                wheel=wheel-0.3
            elif a>wheel:
                wheel=wheel+0.3
            if math.isnan(wheel):
                cmd_x_y.theta=0
            else:
                cmd_x_y.theta=math.radians(wheel)
            pub.publish(cmd_x_y)
            rate.sleep()
            if reward==-5 :
                wheel=0
                print(good_time, ' Reward: %i' % int(ep_reward), 'Explore: %.2f' % var)
                ep_reward=0
                good_time=0
    ddpg.save()


        
        
def train():
    var = 1
    ep_reward_b=0
    MEMORY_CAPACITY = 1000
    goood_job=0
    ddpg.restore() ########important
    while not rospy.is_shutdown():

            # Add exploration noise
            a = ddpg.choose_action(state)
            a = np.random.normal(a, var)
            s_, r, done  = env.step(a)
            
            ddpg.store_transition(s, a, r , s_)
            if ddpg.pointer > MEMORY_CAPACITY:
                var *= .9995    # decay the action randomness
                ddpg.learn()
            s = s_.copy()
            ep_reward += r
            if j == MAX_EP_STEPS-1 or done==True :
                print('Episode:', i, ' Reward: %i' % int(ep_reward), 'Explore: %.2f' % var,'Running time: ', time.time() - t1 )
                if len(GLOBAL_RUNNING_R) == 0: GLOBAL_RUNNING_R.append(ep_reward)
                else: GLOBAL_RUNNING_R.append(GLOBAL_RUNNING_R[-1]*0.9+ep_reward*0.1)
                if int(ep_reward_b*100)==int(ep_reward*100):
                    var=0.5
                ep_reward_b=ep_reward
                if i==100 or i==150 or i==200 or done==1:
                    RENDER = True
                else:
                    RENDER = False
                break

    plt.plot(np.arange(len(GLOBAL_RUNNING_R)), GLOBAL_RUNNING_R)
    plt.xlabel('Episode'); plt.ylabel('Moving reward'); plt.ion(); plt.show()
    print('Running time: ', time.time() - t1_)
    ddpg.save()



# def eval_():
#     ddpg.restore()
#     env.render()
#     env.viewer.set_vsync(True)
#     s = env.reset()
#     while True:
#         s=env.reset_easy()
#         ep_reward = 0
#         for j in range(300):
#             env.render()
#             a = ddpg.choose_action(s)
#             s, r, done = env.step_easy(a)
#             # s, r, done,wheel_yam,v  = env.step_easy(a)
# #            print(-wheel_yam*50/0.5235+100,v*6/100+189)
#             ep_reward += r
            
#             if j == 299 or done==True :
#                     print(' Reward: %i' % int(ep_reward) )
#                     break

if ON_TRAIN:
    train()
else:
    ros_robot()

