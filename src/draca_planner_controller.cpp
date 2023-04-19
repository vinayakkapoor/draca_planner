/* include header file of this class */
#include "draca_planner_controller.h"

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>


#define ALTITUDE 5

namespace draca_planner
{
/* initialize() //{ */

void Draca_plannerController::initialize(const ros::NodeHandle& parent_nh, const std::string& name,
                                        const std::string& ros_name_space,
                                        std::shared_ptr<mrs_lib::Transformer> transformer)
{
  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forget to come back here
  _name_ = name;

  // | ------------------------ Tf ------------------------------ |

  transformer_ = transformer;

  /* obtain node handle */
  ros::NodeHandle nh(parent_nh, ros_name_space);
  ros::NodeHandle nh_g("/");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */
  mrs_lib::ParamLoader pl_parent(parent_nh, "Draca_plannerController");

  pl_parent.loadParam("uav_name", _uav_name_);
  pl_parent.loadParam("uav_frame_id", _uav_frame_id_);
  pl_parent.loadParam("origin_frame_id", _origin_frame_id_);

  std::cout << _uav_name_ << std::endl;

  mrs_lib::ParamLoader pl_child(nh, "Draca_plannerController");

  /*serv.request.px = pl_child.loadParam2<double>("p_x");
  serv.request.py = pl_child.loadParam2<double>("p_y");
  serv.request.vx = pl_child.loadParam2<double>("v_x");
  serv.request.vy = pl_child.loadParam2<double>("v_y");*/
  
  serv.request.v_pref = pl_child.loadParam2<double>("v_pref");
  //serv.request.theta = pl_child.loadParam2<double>("theta");
  serv.request.radius = pl_child.loadParam2<double>("radius");

  //loading the correct goal positions 
  if(_uav_name_.find('1') != std::string::npos ){
    serv.request.pgx = pl_child.loadParam2<double>("goal_x");
    serv.request.pgy = pl_child.loadParam2<double>("goal_y");    
  }
  else{
    serv.request.pgx = pl_child.loadParam2<double>("goal_x1");
    serv.request.pgy = pl_child.loadParam2<double>("goal_y1");
  }
  std::cout << "initialised goal as: (" << serv.request.pgx << "," << serv.request.pgy << ")" << std::endl;
  serv.request.radius1 = serv.request.radius;
  serviceClient = nh.serviceClient<draca_planner::draca_service_planning>("/draca_service_planning",true);
  std::cout<< "planning service available: " << serviceClient.exists() << std::endl;

  // | ------------------ initialize subscribers ----------------- |

  // | ------------------ initialize publishers ----------------- |

  /* Check if all parameters were loaded successfully */
  if (!pl_parent.loadedSuccessfully() || !pl_child.loadedSuccessfully())
  {
    /* If not, alert the user and shut the node down */
    ROS_ERROR("[Draca_plannerController] parameter loading failure");
    ros::shutdown();
  }

  // | ---------- initialize dynamic reconfigure server --------- |

  /* reconfig_srv_.reset(new dynamic_reconfigure::Server<multi_uav_dynreconfig::DynReconfigConfig>(mutex_dyn_reconfig_,
   * nh)); */
  /* dynamic_reconfigure::Server<multi_uav_dynreconfig::DynReconfigConfig>::CallbackType f =
   * boost::bind(&Draca_plannerController::callbackDynReconfig, this, _1, _2); */
  /* reconfig_srv_->setCallback(f); */

  is_init_ = true;
  ROS_INFO_ONCE("[Draca_plannerController] Initialized");
}
//}

/* activate() //{ */

void Draca_plannerController::activate()
{
  is_active_ = true;
  ROS_INFO_STREAM("[Draca_plannerController] Activated");
}

//}

/* deactivate() //{ */

void Draca_plannerController::deactivate()
{
  is_active_ = false;
  ROS_INFO_STREAM("[Draca_plannerController] Deactivated");
}

//}

/* update() //{ */
std::optional<std::any> Draca_plannerController::update(std::shared_ptr<swm_ctrl::SwarmCommonDataHandler> common_data)
{
  if (!is_init_ || !is_active_)
  {
    return nullopt;
  }
  // std::cout << "in the loop, helpppp" << std::endl;
  swm_utils::IdStateStampedConstPtr self_state = common_data->getSelfState();
  swm_utils::IdStateArrayStampedConstPtr neighbor_states = common_data->getNeighborStates();

  mrs_msgs::VelocityReferenceStamped vel_cmd;
  vel_cmd.header = swm_r_utils::createHeader(self_state->header.frame_id, ros::Time::now());

  // self_state_global.header.seq = self_state.header.seq;
  // self_state_global.header.stamp = self_state.header.stamp;
  // self_state_global.header.frame_id = "global_origin";

  geometry_msgs::Pose globalPose_self =
      *(transformer_->transformSingle(self_state->header.frame_id, self_state->state.pose, "global_origin"));
  geometry_msgs::Vector3 globalLinearVel_self =
      *(transformer_->transformSingle(self_state->header.frame_id, self_state->state.velocity.linear, "global_origin"));
  //auto globalHeading_self = 
  //    *(transformer_->transformSingle(self_state->header.frame_id, self_state->state.heading, "global_origin"));

  // std::cout << (*(neighbor_states->states.begin())).pose<< std::endl;

  geometry_msgs::Point globalPosition_neighbor = *(transformer_->transformSingle(
      self_state->header.frame_id, (*(neighbor_states->states.begin())).pose.position, "global_origin"));
  geometry_msgs::Vector3 globalLinearVel_neighbor = *(transformer_->transformSingle(
      self_state->header.frame_id, (*(neighbor_states->states.begin())).velocity.linear, "global_origin"));
  // std::cout << globalLinearVel_neighbor << std::endl;

  geometry_msgs::Vector3 finalReqVel;
  


  finalReqVel.x = 0.0;
  finalReqVel.y = 0.0;
  finalReqVel.z = 0.0;

  if (!correctAlt)
  {
    finalReqVel.z = 0.0;
    std::cout << "attaining proper altitude first" << std::endl;
    if (globalPose_self.position.z > ALTITUDE)
    {
      correctAlt = true;
    }
  }
  else
  {
    if (globalPose_self.position.z < ALTITUDE - 1)
    {
      correctAlt = false;
    }

    serv.request.px = globalPose_self.position.x;
    serv.request.py = globalPose_self.position.y;
    serv.request.vx = globalLinearVel_self.x;
    serv.request.vy = globalLinearVel_self.y;

    // std::cout << std::endl << std::endl << std::endl << "inside draca_controller " << std::endl;

    /*double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;    
    tf2::Quaternion q(globalPose_self.orientation.w, globalPose_self.orientation.x, globalPose_self.orientation.y,
                      globalPose_self.orientation.z);
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);*/

    

    //std::cout << "self state: " << self_state->state << std::endl;
    //std::cout << "globalHeading_self: " << globalHeading_self << std::endl;
    //std::cout << "neighbor state: " << (*(neighbor_states->states.begin())).pose << std::endl << std::endl;

    serv.request.theta = self_state->state.heading; //globalHeading_self;

    serv.request.px1 = globalPosition_neighbor.x;
    serv.request.py1 = globalPosition_neighbor.y;
    serv.request.vx1 = globalLinearVel_neighbor.x;
    serv.request.vy1 = globalLinearVel_neighbor.y;

    // std::cout << self_state->state.velocity << std::endl;

    auto start_time = chrono::high_resolution_clock::now();
    try
    {
      serviceClient.call(serv);
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
    }       
    auto end_time = chrono::high_resolution_clock::now();
    auto duration = chrono::duration_cast<chrono::milliseconds>(end_time - start_time);

    std::cout << "Took " << duration.count() << " ms to call service" << std::endl;


    //std::cout << "request: " << serv.request << std::endl;
    //std::cout << "response: " << serv.response << std::endl;
    
    geometry_msgs::Vector3 tempReqVel;
    tempReqVel.x = serv.response.x_velocity;
    tempReqVel.y = serv.response.y_velocity;
    tempReqVel.z = 0.0;     
    
    geometry_msgs::Vector3 ReqLinearVel_self = *(transformer_->transformSingle("global_origin", tempReqVel, self_state->header.frame_id));

    finalReqVel.x = ReqLinearVel_self.x;
    finalReqVel.y = ReqLinearVel_self.y;
  }

  
  vel_cmd.reference.velocity = swm_r_utils::vector3FromEigen(e::Vector3d(finalReqVel.x, finalReqVel.y, finalReqVel.z));
  vel_cmd.reference.use_heading = true;
  vel_cmd.reference.heading = serv.response.angle;
  vel_cmd.reference.use_heading_rate = false;
  vel_cmd.reference.use_altitude = true;
  vel_cmd.reference.altitude = ALTITUDE;
  vel_cmd.reference.heading_rate = 0.0;

  return std::any{ vel_cmd };

}
}; // namespace draca_planner

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(draca_planner::Draca_plannerController, swarm_control_manager::SwarmController);
