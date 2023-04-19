import torch
import random
import itertools
import math
from collections import defaultdict
from model import ValueNetwork
from env import ENV
from utils import *


class HandleDraca_planner:
    def __init__(self, state_dim, gamma, weight_path, env_config, kinematic='true', phase='test', device='cpu'):
        self.gamma = gamma
        self.kinematic = kinematic
        self.phase = phase
        self.device = torch.device(device)
        self.test_env = ENV(config=env_config,phase=phase)
        self.model = ValueNetwork(state_dim=state_dim, fc_layers=[150, 100, 100], kinematic=kinematic)
        self.model.load_state_dict(torch.load(weight_path, map_location=lambda storage, loc: storage))

    def build_action_space(self, v_pref):
        if self.kinematic:
            velocities = [(i + 1) / 5 * v_pref for i in range(5)]
            rotations = [i/4*math.pi/3 - math.pi/6 for i in range(5)]
            actions = [Action(*x) for x in itertools.product(velocities, rotations)]
            for i in range(25):
                random_velocity = random.random() * v_pref
                random_rotation = random.random() * math.pi/3 - math.pi/6
                actions.append(Action(random_velocity, random_rotation))
            actions.append(Action(0, 0))
        else:
            velocities = [(i + 1) / 5 * v_pref for i in range(5)]
            rotations = [i / 4 * 2 * math.pi for i in range(5)]
            actions = [Action(*x) for x in itertools.product(velocities, rotations)]
            for i in range(25):
                random_velocity = random.random() * v_pref
                random_rotation = random.random() * 2 * math.pi
                actions.append(Action(random_velocity, random_rotation))
            actions.append(Action(0, 0))

        return actions
    
    def filter_velocity(self, joint_state):

        # TODO: filter velocity and avoid oscillation
        filtered_v = Velocity(joint_state.vx1, joint_state.vy1)

        return filtered_v

    def propagate(self, state, v_est, delta_t=1):
        """
        Compute approximate next state with estimated velocity/action

        """
        if isinstance(state, ObservableState) and isinstance(v_est, Velocity):
            # propagate state of the other agent
            new_px = state.px + v_est.x * delta_t
            new_py = state.py + v_est.y * delta_t
            state = ObservableState(new_px, new_py, v_est.x, v_est.y, state.radius)
        elif isinstance(state, FullState) and isinstance(v_est, Action):
            # propagate state of current agent
            # perform action without rotation
            if self.kinematic:
                new_px = state.px + math.cos(state.theta + v_est.r) * v_est.v * delta_t
                new_py = state.py + math.sin(state.theta + v_est.r) * v_est.v * delta_t
            else:
                new_px = state.px + math.cos(v_est.r) * v_est.v * delta_t
                new_py = state.py + math.sin(v_est.r) * v_est.v * delta_t
            state = FullState(new_px, new_py, state.vx, state.vy, state.radius,
                            state.pgx, state.pgy, state.v_pref, state.theta)
        else:
            raise ValueError('Type error')

        return state        

    def calcVelocity(self, px, py, vx, vy, radius, pgx, pgy, v_pref, theta, px1, py1, vx1, vy1, radius1):
        state = self.test_env.reset(px, py, vx, vy, radius, pgx, pgy, v_pref, theta, px1, py1, vx1, vy1, radius1)
        
        state_sequences = [] #defaultdict(list)
        state_sequences.append(state)

        action_space = self.build_action_space(state.v_pref)

        other_v_est = self.filter_velocity(state)
        other_sn_est = self.propagate(ObservableState(*state[9:]), other_v_est)
        max_value = float('-inf')
        best_action = None

        for action in action_space:            
            reward, _ = self.test_env.compute_reward(action)
            sn_est = self.propagate(FullState(*state[:9]), action)
            sn_est = torch.Tensor([sn_est + other_sn_est]).to(self.device)
            value = reward + pow(self.gamma, state.v_pref) * self.model(sn_est, self.device).data.item()
            if value > max_value:
                max_value = value
                best_action = action

        new_state, rewards, done = self.test_env.step(best_action)
            

        return new_state.vx, new_state.vy, new_state.theta