#!/usr/bin/env python
# -*- coding: UTF-8 -*-   


import numpy as np
import time
import sys
import math
import pyglet


UNIT = 10   # pixels
MAZE_H = 4  # grid height
MAZE_W = 4  # grid width
car_r=25
O_LC=10
dt=0.1
carline_w=40
###################################################################
class CarEnv(object):
    viewer = None
    wheel_yam_bound = [-math.pi/6, math.pi/6]
    v_bound = [0, 100]
    point_bound =[30,1100]    
    goal_point=np.array([980,543])    
    def __init__(self):
        self.car_info=np.zeros(2, dtype=[('a', np.float32), ('l', np.float32),('w', np.float32),('yam',np.float32),('main_vec',np.float32)])
        self.car_info['a']=(30,30)  #x座標點 y座標點
        self.car_info['l']=(20,20)  #Lf=20 Lr=20 重心到車頭的長度 重心到車尾長度
        self.car_info['w']=(3,15)  #車頭寬度及車車尾寬度 24.5  34.5
        self.car_info['yam']=(0,0) #重心角度 車輪旋轉角度
        self.car_info['main_vec']=(0,0) #主向量x y
        self.o1_point=np.array([200,280])
        self.o2_point=np.array([150,150])
        self.ran=math.pi
        self.obs_l=np.zeros(O_LC,dtype=np.float32)
        self.O_LC=O_LC
        self.raceline_1 ,self.raceline_2 ,self.raceline_XY=self.race_line()
        self.wheel_yam_a=0
        self.wheel_yam=math.radians(0)
        self.v = 3
        self.a=0
        self.action_bound=np.array([1.,1.])
        self.max_x=0
    def step(self, action):
        done = False
        r = 0.
        self.wheel_yam_a=action[0]
#        print(self.wheel_yam_a)
        self.wheel_yam=self.wheel_yam+self.wheel_yam_a*dt
        self.wheel_yam = np.clip(self.wheel_yam, *self.wheel_yam_bound)
        self.car_info['yam'][1]=self.wheel_yam
        self.a=action[1]*30
        self.v = self.v + self.a*dt
        self.v = np.clip(self.v, *self.v_bound)
        beta = math.atan((self.car_info['l'][1] / (self.car_info['l'][0] +self.car_info['l'][1])) * math.tan(self.wheel_yam))
        self.car_info['main_vec'][0]=self.v * math.cos(self.car_info['yam'][0] + beta)
        self.car_info['main_vec'][1]=self.v * math.sin(self.car_info['yam'][0] + beta)
        self.car_info['a'][0]+=  self.v * math.cos(self.car_info['yam'][0] + beta)*dt
        self.car_info['a'][1]+=  self.v * math.sin(self.car_info['yam'][0] + beta)*dt
        self.car_info['yam'][0] += (self.v / self.car_info['l'][0]) * math.sin(beta)*dt
        if self.car_info['yam'][0] > math.pi:
            self.car_info['yam'][0]-=2*math.pi
        elif self.car_info['yam'][0]<-math.pi:
            self.car_info['yam'][0]+=2*math.pi
        self.car_info['a']=np.clip(self.car_info['a'],*self.point_bound)
        # state

        v_goal=(self.goal_point-self.car_info['a'])
        self.obs_l[:],crash=self.obs_line()
        car_goal_vec=math.atan(v_goal[1]/v_goal[0])-self.car_info['yam'][0]
        s=np.hstack((v_goal/1000,self.obs_l/1000,self.car_info['yam'][1]*2,self.v/100,car_goal_vec)) #goal obsline wheelyam v car_goal_vec
        
        
        # done and reward
##conut go on road
        if self.max_x<self.car_info['a'][0] and crash==0:
            self.max_x=self.car_info['a'][0]
            r=abs(self.v/20)
##road middle
        if self.car_info['a'][0]<500:
           y=math.cos(self.car_info['a'][0]/100)*150+400
        if self.car_info['a'][0]>500:
           y=self.car_info['a'][0]/5+ 343
        r-=abs(self.car_info['a'][1]-y)/5000 
#action change a lot
        r-=np.sum(np.absolute(action))/1000
#goal_dis
        goal_car_dis=np.hypot(*(self.car_info['a']-self.goal_point)) 
        r-=goal_car_dis/40000
#        print([abs(self.car_info['a'][1]-y)/5000,self.v/20,goal_car_dis/40000,np.sum(np.absolute(action))/100])
        if goal_car_dis<25+car_r:
#            done = True
            r += 10.
        if crash==1:
#            done = True
            r += -1.
#            print('crash')
        if self.car_info['a'][0]==30 or self.car_info['a'][0]>1000 :
#                done = True
                r += -1.
#        print(r)
  #      return s, r, done ,self.wheel_yam,self.v
        return s, r, done 

    def race_line(self):
        race_line=[]
        race_lineXY=np.zeros((1100,2), dtype=np.float32)

        for i in np.linspace(0,500,1000,endpoint=False):
           y=math.cos(i/100)*150+400
           race_line.append(i)
           race_line.append(y)
#           count+=1
        for i in np.linspace(500,1000,1000,endpoint=False):
           y=i/5+ 343
           race_line.append(i)
           race_line.append(y)
#           count+=1
        line_dot_v=[]
        line_dot_v2=[]
        line_dot_v.append(race_line)
        line_dot_v2.append(race_line)
        line_dot_v=np.hstack(line_dot_v)
        line_dot_v2=np.hstack(line_dot_v2)
        for i in range(0,len(line_dot_v)-2,2):
            line_dot_v[i+1]-=(race_line[i+2]-race_line[i])*carline_w/np.hypot(*(race_line[i+2]-race_line[i],race_line[i+3]-race_line[i+1]))
            line_dot_v[i]+=(race_line[i+3]-race_line[i+1])*carline_w/np.hypot(*(race_line[i+2]-race_line[i],race_line[i+3]-race_line[i+1]))
            race_lineXY[int(line_dot_v[i])][0]=line_dot_v[i+1]
        for i in range(0,len(line_dot_v2)-2,2):            
            line_dot_v2[i+1]+=(race_line[i+2]-race_line[i])*carline_w /np.hypot(*(race_line[i+2]-race_line[i],race_line[i+3]-race_line[i+1]))
            line_dot_v2[i]-=(race_line[i+3]-race_line[i+1])*carline_w /np.hypot(*(race_line[i+2]-race_line[i],race_line[i+3]-race_line[i+1]))
            race_lineXY[int(line_dot_v2[i])][1]=line_dot_v2[i+1]
        for i in range(0,1039,1):
            if race_lineXY[i+1][0]==0:
                j=i
                j+=1
                while race_lineXY[j][0]==0:
                    j=j+1
                    if j>self.point_bound[1]-1:
                        j=i-2
                        break
                j+=1
                a=(race_lineXY[j][0]-race_lineXY[i][0])/(j-i)
                b=race_lineXY[i][0]-a*i
                for k in range(i,j,1):
                    race_lineXY[int(k)][1]=a*k+b
            if race_lineXY[i+1][1]==0:
                j=i
                j+=1
                while race_lineXY[j][1]==0:
                    j=j+1
                    if j>self.point_bound[1]-1:
                        j=i-2
                        break
                j+=1
                a=(race_lineXY[j][1]-race_lineXY[i][1])/(j-i)
                b=race_lineXY[i][1]-a*i
                for k in range(i,j,1):
                    race_lineXY[int(k)][1]=a*k+b
        return line_dot_v,line_dot_v2 ,race_lineXY

    def obs_line(self,car_=None):
        obs_line=[]
        crash=0
        for i in np.linspace(-math.pi/2, math.pi/2,O_LC,endpoint=False):
            if car_ is None:
                car_point=self.car_info['a'].copy()
                car_=self.car_info['a'].copy()
            else:
                car_point=car_.copy()
                
            for j in np.linspace(1,400,200):
                car_point[0]=car_[0]+j*math.cos(i+self.car_info['yam'][0])
                car_point[1]=car_[1]+j*math.sin(i+self.car_info['yam'][0])
                car_point=np.clip(car_point,1,1040)
                if self.raceline_XY[int(car_point[0])][0]>car_point[1] or self.raceline_XY[int(car_point[0])][1]<car_point[1]:
                        if j<25:
                            crash=1
                        break
            obs_line.append(j)
        
        return obs_line,crash
            
    def reset(self):
#        self.o1_point[:]=np.random.rand(2)*(100,200)+(100,30)
        self.car_info['a']=(30,550)
        self.wheel_yam_a=0
        self.wheel_yam=math.radians(0)
        self.car_info['yam']=(math.radians(-20),0)
        self.v = 1
        self.a=0
        self.max_x=0
        v_goal=(self.goal_point-self.car_info['a'])
        self.obs_l[:],_=self.obs_line()
        car_goal_vec=math.atan(v_goal[1]/v_goal[0])-self.car_info['yam'][0]
        s=np.hstack((v_goal/1000,self.obs_l/1000,self.car_info['yam'][1]*2,self.v/100,car_goal_vec)) #goal obsline wheelyam v car_goal_vec
        
        return s
    
    def render(self):
        if self.viewer is None:
            self.viewer = Viewer(self.goal_point,self.car_info,self.raceline_1,self.raceline_2,self.obs_l)
        self.viewer.render()
    def sample_action(self):
#        self.ran-=0.1
#        if self.ran<-math.pi:
#            self.ran=math.pi
#        return np.random.rand(2)  # two radians

        return np.array([0.3,0])
        

class Viewer(pyglet.window.Window):
    bar_thc = 5

    def __init__(self,goal_point,car_point,car_line_dot,car_line_dot2,obs_line):
        # vsync=False to not use the monitor FPS, we can speed up training
        super(Viewer, self).__init__(width=1100, height=800, resizable=True, caption='gooood_car', vsync=False)
        self.car_point_=car_point['a']
        self.car_point_l=car_point['l']
        self.car_point_w=car_point['w']
        self.car_yam=car_point['yam']
        self.car_main_v=car_point['main_vec']
        self.obs_line=obs_line
        self.count=0
        vehicle_v,wheel_v,main_vec_v=self.makevehicle(*self.car_yam)
        goal_v=np.hstack([goal_point-25,goal_point[0]-25,goal_point[1]+25,goal_point+25,goal_point[0]+25,goal_point[1]-25])
        pyglet.gl.glClearColor(1, 1, 1, 1)
        #        GL_POINTS  GL_LINES GL_LINE_STRIP GL_LINE_LOOP GL_POINTS
        background = pyglet.graphics.OrderedGroup(0)
        foreground = pyglet.graphics.OrderedGroup(1)
        foreground1 = pyglet.graphics.OrderedGroup(2)
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


        self.o_line=self.batch.add(
            int(len(car_line_dot)/2), pyglet.gl.GL_LINE_STRIP, background,
            ('v2f', car_line_dot), ('c3B', (0, 0, 0) * int(len(car_line_dot)/2)))
        self.o_line2=self.batch.add(
            int(len(car_line_dot2)/2), pyglet.gl.GL_LINE_STRIP, foreground,
            ('v2f', car_line_dot2), ('c3B', (0, 0, 0) * int(len(car_line_dot2)/2)))
        line_dot=self.linedot()
        self.o_line=self.batch.add(
            int(len(line_dot)/2), pyglet.gl.GL_LINE_STRIP, foreground1,
            ('v2f', line_dot), ('c3B', (0, 0, 0) * int(len(line_dot)/2)))

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
        rw_p=((rh[0]+rl[0])/2,(rh[1]+rl[1])/2)
        w_yam=yam+w_yam
        wlh=(rw_p[0]-wheel_l*math.cos(w_yam)+wheel_w*math.sin(w_yam),rw_p[1]-wheel_l*math.sin(w_yam)-wheel_w*math.cos(w_yam))
        wll=(rw_p[0]-wheel_l*math.cos(w_yam)-wheel_w*math.sin(w_yam),rw_p[1]-wheel_l*math.sin(w_yam)+wheel_w*math.cos(w_yam))
        wrh=(rw_p[0]+wheel_l*math.cos(w_yam)+wheel_w*math.sin(w_yam),rw_p[1]+wheel_l*math.sin(w_yam)-wheel_w*math.cos(w_yam))
        wrl=(rw_p[0]+wheel_l*math.cos(w_yam)-wheel_w*math.sin(w_yam),rw_p[1]+wheel_l*math.sin(w_yam)+wheel_w*math.cos(w_yam))
        return np.hstack([ll,lh,rh,rl]) ,np.hstack([wll,wlh,wrh,wrl]) ,np.hstack([self.car_point_,self.car_point_+self.car_main_v])

        
        
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
        for i, j in zip(np.linspace(-math.pi/2, math.pi/2 ,O_LC,endpoint=False),range(O_LC)):
            l_dot=self.car_point_.copy()
            line_dot_v.append(l_dot.copy())
            l_dot[0]+=self.obs_line[j]*math.cos(i+self.car_yam[0])
            l_dot[1]+=self.obs_line[j]*math.sin(i+self.car_yam[0])
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
        line_dot=self.linedot()
        self.o_line.vertices=line_dot
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
