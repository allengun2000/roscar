#!/usr/bin/env python
import tensorflow as tf
import numpy as np
import gym
import time

tf.reset_default_graph()
#####################  hyper parameters  ####################

MAX_EPISODES = 2000
MAX_EP_STEPS = 200
LR_A = 0.001    # learning rate for actor
LR_C = 0.002    # learning rate for critic
GAMMA = 0.9     # reward discount
TAU = 0.01      # soft replacement
MEMORY_CAPACITY = 1000
BATCH_SIZE = 100

import matplotlib.pyplot as plt
import math


###ROS library
import rospy
#import math
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from std_msgs.msg import Float64MultiArray
from vision_pro.msg import line_inform
from geometry_msgs.msg import Pose2D


###############################  DDPG  ####################################


class DDPG(object):
    def __init__(self, a_dim, s_dim, a_bound,):
        self.memory = np.zeros((MEMORY_CAPACITY, s_dim * 2 + a_dim + 1), dtype=np.float32)
        self.memory_sp = np.zeros((MEMORY_CAPACITY, s_dim + a_dim ), dtype=np.float32)
        self.pointer_sp = 0
        self.pointer = 0
        self.sess = tf.Session()

        self.a_dim, self.s_dim, self.a_bound = a_dim, s_dim, a_bound,
        self.S = tf.placeholder(tf.float32, [None, s_dim], 's')
        self.S_ = tf.placeholder(tf.float32, [None, s_dim], 's_')
        self.a_s = tf.placeholder(tf.float32, [None, a_dim], 'a_p')
        self.R = tf.placeholder(tf.float32, [None, 1], 'r')

        self.a = self._build_a(self.S,)
        q = self._build_c(self.S, self.a, )
        a_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope='Actor')
        c_params = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope='Critic')
        ema = tf.train.ExponentialMovingAverage(decay=1 - TAU)          # soft replacement

        def ema_getter(getter, name, *args, **kwargs):
            return ema.average(getter(name, *args, **kwargs))

        target_update = [ema.apply(a_params), ema.apply(c_params)]      # soft update operation
        a_ = self._build_a(self.S_, reuse=True, custom_getter=ema_getter)   # replaced target parameters
        q_ = self._build_c(self.S_, a_, reuse=True, custom_getter=ema_getter)

        a_loss = - tf.reduce_mean(q)  # maximize the q
        self.atrain = tf.train.AdamOptimizer(LR_A).minimize(a_loss, var_list=a_params)

        self.ap_loss = tf.losses.mean_squared_error(labels=self.a_s, predictions=self.a)  # maximize the q
        self.aptrain = tf.train.AdamOptimizer(LR_A).minimize(self.ap_loss, var_list=a_params)

        with tf.control_dependencies(target_update):    # soft replacement happened at here
            q_target = self.R + GAMMA * q_
            td_error = tf.losses.mean_squared_error(labels=q_target, predictions=q)
            self.ctrain = tf.train.AdamOptimizer(LR_C).minimize(td_error, var_list=c_params)

        self.sess.run(tf.global_variables_initializer())

    def choose_action(self, s):
        return self.sess.run(self.a, {self.S: s[np.newaxis, :]})[0]

    def learn(self):
        indices = np.random.choice(MEMORY_CAPACITY, size=BATCH_SIZE)
        bt = self.memory[indices, :]
        bs = bt[:, :self.s_dim]
        ba = bt[:, self.s_dim: self.s_dim + self.a_dim]
        br = bt[:, -self.s_dim - 1: -self.s_dim]
        bs_ = bt[:, -self.s_dim:]

        self.sess.run(self.atrain, {self.S: bs})
        self.sess.run(self.ctrain, {self.S: bs, self.a: ba, self.R: br, self.S_: bs_})

    def s_learn(self):
        indices = np.random.choice(MEMORY_CAPACITY, size=BATCH_SIZE)
        bt = self.memory_sp[indices, :]
        bs = bt[:, :self.s_dim]
        ba = bt[:, self.s_dim: self.s_dim + self.a_dim]
        self.sess.run(self.aptrain, {self.S: bs,self.a_s:ba})
        return self.sess.run(self.ap_loss ,{self.S: bs,self.a_s:ba})

    def store_transition(self, s, a, r, s_ ):
        transition = np.hstack((s, a, r, s_))
        index = self.pointer % MEMORY_CAPACITY  # replace the old memory with new memory
        self.memory[index, :] = transition
        self.pointer += 1
    def store_transition_sp(self, s, a):
        transition = np.hstack((s, a))
        index = self.pointer_sp % MEMORY_CAPACITY  # replace the old memory with new memory
        self.memory_sp[index, :] = transition
        self.pointer_sp += 1
    def _build_a(self, s, reuse=None, custom_getter=None):
        trainable = True if reuse is None else False
        net=tf.layers.batch_normalization(s,trainable=trainable)
        with tf.variable_scope('Actor', reuse=reuse, custom_getter=custom_getter):
            net = tf.layers.dense(net, 50, activation=tf.nn.relu, name='l1', trainable=trainable)
            net = tf.layers.dense(net, 500, activation=tf.nn.relu, name='l2', trainable=trainable)
            tf.layers.dropout(net,rate=0.2,noise_shape=None,seed=None,training=trainable,name='drop')
            net = tf.layers.dense(net, 20, activation=tf.nn.relu, name='l3', trainable=trainable)
            # net = tf.layers.dense(net, 500, activation=tf.nn.relu, name='l3', trainable=trainable)
            # net = tf.layers.dense(net, 500, activation=tf.nn.relu, name='l4', trainable=trainable)
            a = tf.layers.dense(net, self.a_dim, activation=tf.nn.tanh, name='a', trainable=trainable)
            return tf.multiply(a, self.a_bound, name='scaled_a')

    def _build_c(self, s, a, reuse=None, custom_getter=None):
        trainable = True if reuse is None else False
        with tf.variable_scope('Critic', reuse=reuse, custom_getter=custom_getter):
            n_l1 = 500
            s_n1 = tf.layers.dense(s, 500, activation=tf.nn.relu, name='l1_s', trainable=trainable)
            w1_s = tf.get_variable('w1_s', [n_l1, n_l1], trainable=trainable)
            w1_a = tf.get_variable('w1_a', [self.a_dim, n_l1], trainable=trainable)
            b1 = tf.get_variable('b1', [1, n_l1], trainable=trainable)
            net = tf.nn.relu(tf.matmul(s_n1, w1_s) + tf.matmul(a, w1_a) + b1)
            return tf.layers.dense(net, 1, trainable=trainable)  # Q(s,a)
    def save(self):
        saver = tf.train.Saver()
        saver.save(self.sess, './spmodel', write_meta_graph=False)

    def restore(self):
        saver = tf.train.Saver()
        saver.restore(self.sess, './spmodel')
    
###############################  training  ####################################
s_dim = 34
a_dim = 1
a_bound =1
ddpg = DDPG(a_dim, s_dim, a_bound)
ON_TRAIN=False
state=np.zeros(s_dim)
reward=0
done=0
cmd=0
cmd_angle=0
var=1
rospy.set_param("/statego",0)
def standardization(data):
    mu = np.mean(data, axis=0)
    sigma = np.std(data, axis=0)
    return (data - mu) / sigma
    
def call_state(data):
     global state
    #  global a\
     state_=np.zeros(17)
     state_[:]=state[0:17]
     if data.state!=3:
        state=np.hstack([standardization(data.mid_y[0:5]),standardization(data.line1_y[0:5]),standardization(data.line2_y[0:5]),data.offest,data.angle_re,state_])
        
        
    
def callcmd(data):
     global cmd_angle
     cmd_angle=np.array(data.data/28)

if __name__ == '__main__':
    pub = rospy.Publisher('/cmd', Pose2D, queue_size=10)
    # ddpg.restore()
    cmd_x_y=Pose2D()
    rospy.Subscriber("/line_info", line_inform, call_state)
    rospy.Subscriber("/wheelFB", Float32, callcmd)
    rospy.init_node('car_sptrain', anonymous=True)
    rate = rospy.Rate(30) # 10hz
    MEMORY_CAPACITY = 1000
    a=np.zeros(1)
    while not rospy.is_shutdown():
            # ddpg.store_transition(state, a/28, reward , state_)
            # ddpg.store_transition_sp(state, cmd_angle)
            # print(state, cmd_angle)
            # print(state, a/28, reward , state_)
            ac_state=rospy.get_param("/statego")
            if ac_state==2:
                ddpg.store_transition_sp(state, cmd_angle)
                print(state, cmd_angle)
            if ddpg.pointer_sp > MEMORY_CAPACITY:
                # ddpg.learn()
                print(ddpg.s_learn())
            if ac_state==1:
                a = ddpg.choose_action(state)
                a = np.clip(a*28, -28,28)
                print(state, cmd_angle,ddpg.s_learn(),a[0])
                cmd_x_y.theta=(a +35)* (720+ 720)/(35 +35) -720
                print(cmd_x_y.theta)
                cmd_x_y.y=2
                pub.publish(cmd_x_y)
                
            rate.sleep()

    ddpg.save()
