#!/usr/bin/env python
import numpy as np
from vehicle import CarEnv
from DDPG import DDPG
import matplotlib.pyplot as plt
import time



###ROS library
import rospy
#import math
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray

DISPLAY_REWARD_THRESHOLD=200
env = CarEnv()
#s_dim = 5+env.O_LC
s_dim = 2+env.O_LC
a_dim = 2
a_bound =env.action_bound
ddpg = DDPG(a_dim, s_dim, a_bound)
ON_TRAIN=False
#ON_TRAIN=True
t1_ = time.time()
GLOBAL_RUNNING_R=[]

def train():
 
    RENDER=False
    var = 1
    ep_reward_b=0
    MEMORY_CAPACITY = 10000
    MAX_EPISODES =150
    MAX_EP_STEPS = 300
    goood_job=0
    ddpg.restore() ########important
    for i in range(MAX_EPISODES):
        t1 = time.time()
        s=env.reset()
        ep_reward = 0
        for j in range(MAX_EP_STEPS): 
            if RENDER:
                env.render()
    
            # Add exploration noise
            a = ddpg.choose_action(s)
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
#                    var=0
#                    goood_job+=1
#                    print(goood_job)
                else:
                    RENDER = False
#                    if goood_job>0:
#                        goood_job-=1
                break
#        if ep_reward>30:
#        if goood_job>10:
#            break
    plt.plot(np.arange(len(GLOBAL_RUNNING_R)), GLOBAL_RUNNING_R)
    plt.xlabel('Episode'); plt.ylabel('Moving reward'); plt.ion(); plt.show()
    print('Running time: ', time.time() - t1_)
    ddpg.save()

def callback(data):
     global state
     line=np.array((data.data),dtype=np.float32) #road w in real is 80 and sim is 40
     
#      main_vec=rospy.get_param('/AvoidChallenge/GoodAngle') s=np.hstack((self.obs_l/1000,self.car_info['yam'][1]/self.wheel_yam_bound[1],self.v/100))
#      num_change=main_vec*3-180
#      go_where_x=math.cos(num_change*math.pi/180)
#      go_where_y=math.sin(num_change*math.pi/180)
#      print(main_vec)
# #     line=np.array(data.data)
#      #print(line[0:120:6])
     state=[]
     state[0:9]=line/1000
     state.append(a[0])
     state.append(a[1])
     state=np.array(state)
     
    #  print(state)

def ros_robot():
    global a
    a=np.zeros(2)
    pub = rospy.Publisher('/car_wheel', Float32, queue_size=10)
    pub1 = rospy.Publisher('/car_speed', Int32, queue_size=10)
    ddpg.restore()
    env.render()
    env.viewer.set_vsync(True)
    env.reset_easy()
    rospy.Subscriber("/roadDis", Float64MultiArray, callback)
    rospy.init_node('car_strage', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    env.real_reset()
    while not rospy.is_shutdown():
            env.render()
            a = ddpg.choose_action(state)
            wheel_yam,v=env.state_reward_easy(state,a,2000)
            done=0
            pub.publish(-wheel_yam*50/0.5235+100)
            pub1.publish(int(v*6/100+189))
            rate.sleep()

        
        




def eval_():
    ddpg.restore()
    env.render()
    env.viewer.set_vsync(True)
    s = env.reset()
    while True:
        s=env.reset_easy()
        ep_reward = 0
        for j in range(300):
            env.render()
            a = ddpg.choose_action(s)
            s, r, done = env.step_easy(a)
            # s, r, done,wheel_yam,v  = env.step_easy(a)
#            print(-wheel_yam*50/0.5235+100,v*6/100+189)
            ep_reward += r
            
            if j == 299 or done==True :
                    print(' Reward: %i' % int(ep_reward) )
                    break

if ON_TRAIN:
    train()
else:
    # ros_robot()
    eval_()
