import argparse
import configparser
import os
import rospy
#from draca_planner.srv import draca_planner
from scripts.handle_draca_planner import HandleDraca_planner 
import time

class Temp:
    _a = 0
    def __init__(self,x,y):
        self.y = y
        self.__b = x
    def modifyA(self):
        Temp._a = 1 
    def printX(self):
        print(self.__b)
    def printA(self):
        print(self._a)

def initialise():
    parser = argparse.ArgumentParser('Parse configuration file')
    parser.add_argument('output_dir', type=str)
    args = parser.parse_args()
    config_file = os.path.join(args.output_dir, 'model.config')
    env_file = os.path.join(args.output_dir, 'env.config')
    weight_file = os.path.join(args.output_dir, 'trained_model.pth')
    model_config = configparser.RawConfigParser()
    model_config.read(config_file)
    env_config = configparser.RawConfigParser()
    env_config.read(env_file)

    state_dim = model_config.getint('model', 'state_dim')
    kinematic = env_config.getboolean('agent', 'kinematic')
    gamma = model_config.getfloat('model', 'gamma')
    
    return HandleDraca_planner(state_dim, gamma, weight_file, env_config,kinematic)

t1 = time.time()*1000
h = initialise()
t2 = time.time()*1000

print("time to initialise: ",(t2-t1))

px=-6
py=0
vx=0
vy=0
radius=0.8
pgx=6
pgy=0
v_pref=1.0
theta=0
px1=6
py1=0
vx1=0
vy1=0
radius1=0.8


#s = h.calcVelocity(px,py,vx,vy,radius,pgx,pgy, v_pref, theta, px1,py1,vx1,vy1,radius1)
n2 = time.time()*1000
#print(n2-n1)

for i in range(20):
    n1 = time.time()*1000
    print(n1 - n2)
    n2=n1
    s = h.calcVelocity(px,py,vx,vy,radius,pgx,pgy, v_pref, theta, px1,py1,vx1,vy1,radius1)
    #print(s)
    px=s.px
    py=s.py
    vx=s.vx
    vy=s.vy
    radius=s.radius
    pgx=s.pgx
    pgy=s.pgy
    v_pref=s.v_pref
    theta=s.theta
    px1=s.px1
    py1=s.py1
    vx1=s.py1
    vy1=s.vy1
    radius1=s.radius1


#t = Temp(5,6)
#t.modifyA()
#t._a = 4
#t.printA()


