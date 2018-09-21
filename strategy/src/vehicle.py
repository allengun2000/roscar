# -*- coding: utf-8 -*-
"""
Created on Mon Sep 17 10:28:14 2018

@author: USER
"""



import numpy as np
import time
import sys
import math
import pyglet


UNIT = 10   # pixels
MAZE_H = 4  # grid height
MAZE_W = 4  # grid width
car_r=25
O_LC=20
dt=0.01
###################################################################
class CarEnv(object):
    viewer = None
    action_bound = [-math.pi/6, math.pi/6]
    point_bound =[30,370]
    v = 60
    a=0
    goal_point=np.array([980,543])

    def __init__(self):
        self.car_info=np.zeros(2, dtype=[('a', np.float32), ('l', np.float32),('w', np.float32),('yam',np.float32),('main_vec',np.float32)])
        self.car_info['a']=(30,30)  #x座標點 y座標點
        self.car_info['l']=(20,20)  #Lf=20 Lr=20 重心到車頭的長度 重心到車尾長度
        self.car_info['w']=(3,15)  #車頭寬度及車車尾寬度
        self.car_info['yam']=(0,0) #重心角度 車輪旋轉角度
        self.car_info['main_vec']=(0,0)
        self.o1_point=np.array([200,280])
        self.o2_point=np.array([150,150])
        self.ran=math.pi
        self.obs_l=np.zeros(O_LC,dtype=np.float32)
        self.O_LC=O_LC
        self.q=0
        self.flag=0
        self.raceline=self.race_line()
    def step(self, action):
        done = False
        r = 0.
        action = np.clip(action, *self.action_bound)
        self.car_info['yam'][1]=action
        beta = math.atan((self.car_info['l'][1] / (self.car_info['l'][0] +self.car_info['l'][1])) * math.tan(action))
        self.car_info['main_vec'][0]=self.v * math.cos(self.car_info['yam'][0] + beta)
        self.car_info['main_vec'][1]=self.v * math.sin(self.car_info['yam'][0] + beta)
        self.car_info['a'][0]+=  self.v * math.cos(self.car_info['yam'][0] + beta)*dt
        self.car_info['a'][1]+=  self.v * math.sin(self.car_info['yam'][0] + beta)*dt
        self.car_info['yam'][0] += (self.v / self.car_info['l'][0]) * math.sin(beta)*dt
        self.v = self.v + self.a*dt
        self.car_info['a']=np.clip(self.car_info['a'],*self.point_bound)

        
        
        # state

        v_goal=(self.goal_point-self.car_info['a'])
        self.obs_l[:]=self.obs_line()
        s=np.hstack((v_goal/400,self.obs_l/400))
#        s = self.car_info['a']/400
        # done and reward
        goal_car_dis=np.hypot(*(self.car_info['a']-self.goal_point)) 
        ob1_car_dis=np.hypot(*(self.car_info['a']-self.o1_point)) 
        ob2_car_dis=np.hypot(*(self.car_info['a']-self.o2_point)) 
        r=-goal_car_dis/4000
        if goal_car_dis<25+car_r:
#            done = True
            r += 1.
        if ob1_car_dis<25+car_r:
#            done = True
            r += -1.
        if ob2_car_dis<25+car_r:
#            done = True
            r += -1.
        if self.car_info['a'][0]==30 or self.car_info['a'][0]==370 \
             or self.car_info['a'][1]==30 or self.car_info['a'][1]==370:
#                done = True
                r += -1.
                
            
        return s, r, done
    def race_line(self):
        race_line=[]
        for i in np.linspace(0,500,50,endpoint=False):
           y=math.cos(i/100)*150+400
           race_line.append(i)
           race_line.append(y)
        for i in np.linspace(501,1000,50,endpoint=False):
           y=i/5+ 343
           race_line.append(i)
           race_line.append(y)
        return race_line
    
    def obs_line(self,car_=None):
        obs_line=[]
        for i in np.linspace(-math.pi/2, math.pi/2,O_LC,endpoint=False):
            if car_ is None:
                car_point=self.car_info['a'].copy()
                car_=self.car_info['a'].copy()
            else:
                car_point=car_.copy()
                
            for j in np.linspace(1,400,20):
                car_point[0]=car_[0]+j*math.cos(i)
                car_point[1]=car_[1]+j*math.sin(i)
                car_point=np.clip(car_point,1,800)
                ob1_car_dis=np.hypot(*(car_point-self.o1_point)) 
                ob2_car_dis=np.hypot(*(car_point-self.o2_point)) 
                if ob1_car_dis<25 or ob2_car_dis<25 \
                    or car_point[1]==1 or car_point[1]==399:
#                    or car_point[0]==1 or car_point[0]==399 \
                        break
            obs_line.append(j)
        
        return obs_line
            
    def reset(self):
#        self.car_info['a']=(30,300)
        self.o1_point[:]=np.random.rand(2)*(100,200)+(100,30)
        self.o2_point[:]=np.random.rand(2)*(100,150)+(100,200)
#        self.car_info['a']=np.random.rand(2)*(1,340)+30
        self.car_info['a']=(30,380)
        self.obs_l[:]=self.obs_line()
        s=np.hstack((self.car_info['a']/400,self.obs_l/400))
        v_goal=(self.goal_point-self.car_info['a'])
        self.obs_l[:]=self.obs_line()
        s=np.hstack((v_goal/400,self.obs_l/400))

        return s
    
    def render(self):
        if self.viewer is None:
            self.viewer = Viewer(self.goal_point,self.car_info,self.raceline,self.o2_point,self.obs_l)
        self.viewer.render()
    def sample_action(self):
#        self.ran-=0.1
#        if self.ran<-math.pi:
#            self.ran=math.pi
#        return np.random.rand(2)+2  # two radians
        if self.q>-60 and self.flag==0:
            self.flag=0
            self.q-=1
        elif self.q<60 :
#            self.flag=1
            self.q+=1
        else:
            self.flag=0
        self.ran=math.radians(self.q)
        return self.ran
        

class Viewer(pyglet.window.Window):
    bar_thc = 5

    def __init__(self,goal_point,car_point,car_line_point,o2_point,obs_line):
        # vsync=False to not use the monitor FPS, we can speed up training
        super(Viewer, self).__init__(width=1000, height=800, resizable=True, caption='gooood_car', vsync=False)
        self.car_point_=car_point['a']
        self.car_point_l=car_point['l']
        self.car_point_w=car_point['w']
        self.car_yam=car_point['yam']
        self.car_main_v=car_point['main_vec']
        self.car_line_dot=car_line_point
        self.o2_point=o2_point
        self.count=0
        vehicle_v,wheel_v,main_vec_v=self.makevehicle(*self.car_yam)
        goal_v=np.hstack([goal_point-25,goal_point[0]-25,goal_point[1]+25,goal_point+25,goal_point[0]+25,goal_point[1]-25])
        
        pyglet.gl.glClearColor(1, 1, 1, 1)
        #        GL_POINTS  GL_LINES GL_LINE_STRIP GL_LINE_LOOP GL_POINTS
        background = pyglet.graphics.OrderedGroup(0)
        foreground = pyglet.graphics.OrderedGroup(1)

        self.batch = pyglet.graphics.Batch()    # display whole batch at once
        self.goal = self.batch.add(
            4, pyglet.gl.GL_QUADS, None,    # 4 corners
            ('v2f', goal_v),
            ('c3B', (0, 255, 255) * 4))    # color
        self.wheel = self.batch.add(
            4, pyglet.gl.GL_QUADS, None,
            ('v2f', wheel_v ),
            ('c3B', (0, 0, 0) * 4,)) 
        self.vehicle = self.batch.add(
            4, pyglet.gl.GL_QUADS, None,
            ('v2f', vehicle_v),
            ('c3B', (249, 86, 86) * 4,))
        self.main_vec=self.batch.add(
            2, pyglet.gl.GL_LINES, foreground,
            ('v2f', main_vec_v), ('c3B', (0, 0, 0) * 2))
   # color
        car_dot=self.makeCircle(200,car_r,*self.car_point_)
        self.car = self.batch.add(
            int(len(car_dot)/2), pyglet.gl.GL_LINE_LOOP, foreground,
            ('v2f', car_dot), ('c3B', (0, 0, 0) * int(len(car_dot)/2)))

#        line_dot=self.linedot()
        car_line_dot,car_line_dot2=self.car_line()
        self.o_line=self.batch.add(
            int(len(car_line_dot)/2), pyglet.gl.GL_LINE_STRIP, background,
            ('v2f', car_line_dot), ('c3B', (0, 0, 0) * int(len(car_line_dot)/2)))
        self.o_line2=self.batch.add(
            int(len(car_line_dot2)/2), pyglet.gl.GL_LINE_STRIP, foreground,
            ('v2f', car_line_dot2), ('c3B', (0, 0, 0) * int(len(car_line_dot2)/2)))
    def makevehicle(self,yam,w_yam):
        lh_p=(-self.car_point_l[1],self.car_point_w[1])
        ll_p=(-self.car_point_l[1],-self.car_point_w[1])
        rh_p=(+self.car_point_l[0],self.car_point_w[0])
        rl_p=(+self.car_point_l[0],-self.car_point_w[0])

        lh=(self.car_point_[0]+lh_p[0]*math.cos(yam)+lh_p[1]*math.sin(yam),self.car_point_[1]+lh_p[0]*math.sin(yam)-lh_p[1]*math.cos(yam))
        ll=(self.car_point_[0]+ll_p[0]*math.cos(yam)+ll_p[1]*math.sin(yam),self.car_point_[1]+ll_p[0]*math.sin(yam)-ll_p[1]*math.cos(yam))
        rh=(self.car_point_[0]+rh_p[0]*math.cos(yam)+rh_p[1]*math.sin(yam),self.car_point_[1]+rh_p[0]*math.sin(yam)-rh_p[1]*math.cos(yam))
        rl=(self.car_point_[0]+rl_p[0]*math.cos(yam)+rl_p[1]*math.sin(yam),self.car_point_[1]+rl_p[0]*math.sin(yam)-rl_p[1]*math.cos(yam))
        wheel_l=5
        wheel_w=8
#        rh_p=(+self.car_point_l[0],self.car_point_w[1])
#        rw_p=(self.car_point_[0]+rh_p[0]*math.cos(yam)+rh_p[1]*math.sin(yam),self.car_point_[1]+rh_p[0]*math.sin(yam)-rh_p[1]*math.cos(yam))
        rw_p=((rh[0]+rl[0])/2,(rh[1]+rl[1])/2)
        w_yam=yam+w_yam
        wlh=(rw_p[0]-wheel_l*math.cos(w_yam)+wheel_w*math.sin(w_yam),rw_p[1]-wheel_l*math.sin(w_yam)-wheel_w*math.cos(w_yam))
        wll=(rw_p[0]-wheel_l*math.cos(w_yam)-wheel_w*math.sin(w_yam),rw_p[1]-wheel_l*math.sin(w_yam)+wheel_w*math.cos(w_yam))
        wrh=(rw_p[0]+wheel_l*math.cos(w_yam)+wheel_w*math.sin(w_yam),rw_p[1]+wheel_l*math.sin(w_yam)-wheel_w*math.cos(w_yam))
        wrl=(rw_p[0]+wheel_l*math.cos(w_yam)-wheel_w*math.sin(w_yam),rw_p[1]+wheel_l*math.sin(w_yam)+wheel_w*math.cos(w_yam))

        
        return np.hstack([ll,lh,rh,rl]) ,np.hstack([wll,wlh,wrh,wrl]) ,np.hstack([self.car_point_,self.car_point_+self.car_main_v])
    def car_line(self):
        line_dot_v=[]
        line_dot_v2=[]
        carline_w=40
        line_dot_v.append(self.car_line_dot)
        line_dot_v2.append(self.car_line_dot)
        line_dot_v=np.hstack(line_dot_v)
        line_dot_v2=np.hstack(line_dot_v2)
        for i in range(0,len(line_dot_v)-2,2):
            line_dot_v[i+1]-=(self.car_line_dot[i+2]-self.car_line_dot[i])*carline_w/np.hypot(*(self.car_line_dot[i+2]-self.car_line_dot[i],self.car_line_dot[i+3]-self.car_line_dot[i+1]))
            line_dot_v[i]+=(self.car_line_dot[i+3]-self.car_line_dot[i+1])*carline_w/np.hypot(*(self.car_line_dot[i+2]-self.car_line_dot[i],self.car_line_dot[i+3]-self.car_line_dot[i+1]))
        for i in range(0,len(line_dot_v2)-2,2):            
            line_dot_v2[i+1]+=(self.car_line_dot[i+2]-self.car_line_dot[i])*carline_w /np.hypot(*(self.car_line_dot[i+2]-self.car_line_dot[i],self.car_line_dot[i+3]-self.car_line_dot[i+1]))
            line_dot_v2[i]-=(self.car_line_dot[i+3]-self.car_line_dot[i+1])*carline_w /np.hypot(*(self.car_line_dot[i+2]-self.car_line_dot[i],self.car_line_dot[i+3]-self.car_line_dot[i+1]))
        return line_dot_v ,line_dot_v2
        
        
    def makeCircle(self,numPoints,r,c_x,c_y):
        verts = []
        for i in range(numPoints):
            angle = math.radians(float(i)/numPoints * 360.0)
            x = r*math.cos(angle) + c_x
            y = r*math.sin(angle) + c_y
            verts += [x,y]
        return verts
    
    def linedot(self):
        line_dot_v=[]
        for i, j in zip(np.linspace(-math.pi, math.pi,O_LC,endpoint=False),range(O_LC)):
            l_dot=self.car_point_.copy()
            line_dot_v.append(l_dot.copy())
            l_dot[0]+=self.obs_line[j]*math.cos(i)
            l_dot[1]+=self.obs_line[j]*math.sin(i)
            line_dot_v.append(l_dot)
        return np.hstack(line_dot_v)
    
    def render(self):
        self._update_car()
        self.switch_to()
        self.dispatch_events()
        self.dispatch_event('on_draw')
        self.flip()
        
    def on_draw(self):
        self.clear()
        self.batch.draw()
        

    def _update_car(self):
        car_dot=self.makeCircle(200,car_r,*self.car_point_)
        self.car.vertices = car_dot 
#        line_dot=self.linedot()
#        self.o_line.vertices=line_dot
        vehicle_v,wheel_v,main_vec_v=self.makevehicle(*self.car_yam)
        self.vehicle.vertices=vehicle_v
        self.wheel.vertices=wheel_v
        self.main_vec.vertices=main_vec_v
    def on_close(self):
        self.close()
        
if __name__ == '__main__':
    env = CarEnv()
    s=env.reset()
    while True:
#        s=env.reset()
        for i in range(1000):
            env.render()
            env.step(env.sample_action())
#            env.step(10)
#    pyglet.on_close()