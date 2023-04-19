import math
import random
from utils import JointState

class Agent:
    def __init__(self, px, py, vx, vy, radius, pgx=None, pgy=None, v_pref=None, theta=None, kinematic=True):
        self.px = px
        self.py = py
        self.vx = vx
        self.vy = vy
        self.radius = radius
        self.pgx = pgx
        self.pgy = pgy
        self.v_pref = v_pref
        self.theta = theta
        self.kinematic = kinematic
        self.done = False

    def update_state(self, action, time):
        self.px, self.py = self.compute_position(time=time, action=action)
        if self.kinematic:
            self.theta += action.r
            self.vx = math.cos(self.theta) * action.v
            self.vy = math.sin(self.theta) * action.v
        else:
            self.vx = math.cos(action.r) * action.v
            self.vy = math.sin(action.r) * action.v
            self.theta = 0

    def get_full_state(self):
        return self.px, self.py, self.vx, self.vy, self.radius, self.pgx, self.pgy, self.v_pref, self.theta

    def get_observable_state(self):
        return self.px, self.py, self.vx, self.vy, self.radius

    def compute_position(self, time, action=None):
        if action is None:
            # assume the agent travels in original speed
            x = self.px + time * self.vx
            y = self.py + time * self.vy
        else:
            if self.kinematic:
                x = self.px + time * math.cos(self.theta + action.r) * action.v
                y = self.py + time * math.sin(self.theta + action.r) * action.v
            else:
                x = self.px + time * math.cos(action.r) * action.v
                y = self.py + time * math.sin(action.r) * action.v
        return x, y

class ENV:
    def __init__(self, config, phase):
        self.radius = config.getfloat('agent', 'radius')
        self.v_pref = config.getfloat('agent', 'v_pref')
        self.kinematic = config.getboolean('agent', 'kinematic')
        self.agent_num = config.getint('sim', 'agent_num')
        self.xmin = config.getfloat('sim', 'xmin')
        self.xmax = config.getfloat('sim', 'xmax')
        self.ymin = config.getfloat('sim', 'ymin')
        self.ymax = config.getfloat('sim', 'ymax')
        self.crossing_radius = config.getfloat('sim', 'crossing_radius')
        self.max_time = config.getint('sim', 'max_time')
        self.agents = [None, None]
        self.counter = 0
        assert phase in ['train', 'test']
        self.phase = phase
        self.test_counter = 0

    def compute_joint_state(self, agent_idx):
        if self.agents[agent_idx].done:
            return None
        else:
            return JointState(*(self.agents[agent_idx].get_full_state() +
                              self.agents[1-agent_idx].get_observable_state()))

    
    def reset(self, px, py, vx, vy, radius, pgx, pgy, v_pref, theta, px1, py1, vx1, vy1, radius1):
        self.agents[0] = Agent(px, py, vx, vy, radius, pgx, pgy, v_pref, theta)
        self.agents[1] = Agent(px1, py1, vx1, vy1, radius1)

        return self.compute_joint_state(0)

    def compute_reward(self, action):
        agent = self.agents[0]
        other_agent = self.agents[1]

        dmin = float('inf')
        dmin_time = 1

        for time in [0, 0.5, 1]:
            pos = agent.compute_position(time, action)
            other_pos = other_agent.compute_position(time, None)
            distance = math.sqrt((pos[0]-other_pos[0])**2 + (pos[1]-other_pos[1])**2)
            if distance < dmin:
                dmin = distance
                dmin_time = time
        final_pos = agent.compute_position(1, action)
        reached_goal = math.sqrt((final_pos[0] - agent.pgx)**2 + (final_pos[1] - agent.pgy)**2) < self.radius

        if dmin < self.radius * 2:
            reward = -0.25
            end_time = dmin_time
        else:
            end_time = 1
            if dmin < self.radius * 2 + 0.2:
                reward = -0.1 - dmin/2
            elif reached_goal:
                reward = 1
            else:
                reward = 0

        return reward, end_time

    def check_boundary(self):
        agent = self.agents[0]
        return self.xmin < agent.px < self.xmax and self.ymin < agent.py < self.ymax

    def step(self, action):
        reward, end_time = self.compute_reward(action)
        self.agents[0].update_state(action, end_time)
        state = self.compute_joint_state(0)

        agent = self.agents[0]
        if not agent.done:
            if reward == 1:
                agent.done = 1
            elif reward == -0.25:
                agent.done = 2
            elif not self.check_boundary():
                agent.done = 3
            elif self.counter > self.max_time:
                agent.done = 4
            else:
                agent.done = False
        
        self.counter+=1

        return state, reward, agent.done
