#!/usr/bin/env python3
import argparse
import configparser
import os

from torch import angle
import rospy
from draca_planner.srv import ddrl_service_planning, ddrl_service_planningResponse
from handle_draca_planner import HandleDraca_planner 

def handle_draca_planner(req, handler):
    x_velocity, y_velocity, angle = handler.calcVelocity(req.px, req.py, req.vx, req.vy, req.radius, req.pgx, req.pgy, req.v_pref, req.theta, req.px1, req.py1, req.vx1, req.vy1, req.radius1)
    return ddrl_service_planningResponse(x_velocity, y_velocity, angle)

def initialise():
    parser = argparse.ArgumentParser('Parse configuration file')
    parser.add_argument('--output_dir', type=str, default='data')
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

    
    return HandleDraca_planner(state_dim, gamma, weight_file, env_config, kinematic)
    

def draca_planner_server():
    rospy.init_node('draca_planner_server')

    print("Initialising model and environment..")
    h = initialise()

    # to preserve OOP and not use global variables to pass handler
    handle_draca_planner_lambda = lambda req : handle_draca_planner(req, h)

    print("Starting planning service for draca_planner..")
    s = rospy.Service('ddrl_service_planning', ddrl_service_planning, handle_draca_planner_lambda)
    
    print("Ready to serve flight!")
    rospy.spin()


if __name__ == "__main__":
    draca_planner_server()