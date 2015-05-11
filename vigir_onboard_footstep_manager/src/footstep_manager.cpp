/*=================================================================================================
// Copyright (c) 2015, David Conner, TORC Robotics
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Team ViGIR or TORC Robotics nor the names of
//       its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================
*/
#include "footstep_manager.h"

#include <vigir_footstep_planning_msgs/StepPlanRequest.h>
#include <vigir_footstep_planning_msgs/Step.h>
#include <vigir_footstep_planning_msgs/Foot.h>
#include <vigir_footstep_planning_msgs/EditStepService.h>

namespace onboard_footstep
{
void FootstepManager::onInit()
{
    ros::NodeHandle& nh         = getNodeHandle();

    foot_pose_transformer_.reset(new vigir_footstep_planning::FootPoseTransformer(nh));

    // TODO: Get them from footstep planner
    nh.param("foot/size/x", foot_size.x, 0.26);
    nh.param("foot/size/y", foot_size.y, 0.13);
    nh.param("foot/size/z", foot_size.z, 0.05);

    nh.param("upper_body/size/x", upper_body_size.x, 0.7);
    nh.param("upper_body/size/y", upper_body_size.y, 1.1);
    nh.param("upper_body/size/z", upper_body_size.z, 0.0);
    nh.param("upper_body/origin_shift/x", upper_body_origin_shift.x, 0.0);
    nh.param("upper_body/origin_shift/y", upper_body_origin_shift.y, 0.0);
    nh.param("upper_body/origin_shift/z", upper_body_origin_shift.z, 0.0);

    // Load default parameters for planner
    ros::NodeHandle nhp("~");
    double  dval;
    int32_t ival;

    nhp.param("max_planning_time"    , dval, 20.0); planner_config_.max_time         = dval;
    nhp.param("max_number_steps"     , ival, 100);  planner_config_.max_steps        = ival;
    nhp.param("max_path_length_ratio", dval, 5.0);  planner_config_.path_length_ratio= dval;
    nhp.param("edit_mode",             ival, 0 );   planner_config_.edit_mode        = ival;
    nhp.param("use_3d_planning",       ival, 0);    planner_config_.use_3d_planning  = ival;
    nhp.param("default_planning_parameter_set_name",selected_footstep_parameter_set_, std::string("drc_step_2D"));

    ROS_INFO("Loaded default planner_config_:  max_planning_time=%f, max_number_steps=%d, max_path_length_ratio=%f,  edit mode=%d, use_3d_planning=%d",
            planner_config_.max_time,
            planner_config_.max_steps,
            planner_config_.path_length_ratio,
            planner_config_.edit_mode,
            planner_config_.use_3d_planning);
    ROS_INFO("  Using %s as the default parameter set", selected_footstep_parameter_set_.c_str());

    // planner will always create new plans if this is unchanged
    start_step_index_ = -1;

    // creates publishers and subscribers for the interaction loop
    onboard_footstep_plan_pub_          = nh.advertise<vigir_footstep_planning_msgs::StepPlan>("current_step_plan", 1, false);
    onboard_planner_status_pub_         = nh.advertise<flor_ocs_msgs::OCSFootstepStatus>("step_planner_status",1,false);
    onboard_planner_parameter_set_pub_  = nh.advertise<std_msgs::String>("active_parameter_set",1,false);
    onboard_planner_configuration_pub_  = nh.advertise<flor_ocs_msgs::OCSFootstepPlanParameters>("planner_configuration",1,false);
    onboard_footstep_plan_request_pub_  = nh.advertise<flor_ocs_msgs::OCSFootstepPlanRequest    >("replan_request", 1, false );


    ocs_footstep_update_sub_             = nh.subscribe<flor_ocs_msgs::OCSFootstepUpdate         >("step_update",              1, &FootstepManager::processFootstepPoseUpdate      , this);
    ocs_footstep_plan_goal_update_sub_   = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanGoalUpdate >("update_step_plan_goal",    1, &FootstepManager::processFootstepPlanGoalUpdate  , this);
    ocs_footstep_plan_goal_sub_          = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanGoal       >("plan_goal",                1, &FootstepManager::processFootstepPlanGoal        , this);
    ocs_footstep_plan_parameters_sub_    = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanParameters >("plan_parameters",          1, &FootstepManager::processFootstepPlanParameters  , this);
    ocs_footstep_set_parameter_set_sub_  = nh.subscribe<std_msgs::String                         >("set_active_parameter_set", 1, &FootstepManager::processFootstepParamSetSelected, this);
    ocs_footstep_plan_sub_               = nh.subscribe<vigir_footstep_planning_msgs::StepPlan   >("updated_step_plan",        1, &FootstepManager::processFootstepPlanUpdate      , this);
    ocs_footstep_plan_request_sub_       = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanRequest    >("replan_request",           1, &FootstepManager::processFootstepPlanRequest     , this);

    // Start the action servers that can be triggered by OCS or behaviors
    step_plan_request_server_ = new StepPlanRequestServer(nh, "step_plan_request",   false);
    step_plan_request_server_->registerGoalCallback(boost::bind(&FootstepManager::stepPlanRequestGoalCB, this));
    step_plan_request_server_->registerPreemptCallback(boost::bind(&FootstepManager::stepPlanRequestPreemptCB, this));
    step_plan_request_server_->start();

    execute_step_plan_server_ = new ExecuteStepPlanServer(nh, "execute_step_plan",false);
    execute_step_plan_server_->registerGoalCallback(boost::bind(&FootstepManager::executeStepPlanGoalCB, this));
    execute_step_plan_server_->registerPreemptCallback(boost::bind(&FootstepManager::executeStepPlanPreemptCB, this));
    execute_step_plan_server_->start();

    update_feet_server_       = new UpdateFeetServer(nh, "update_feet",false);
    update_feet_server_->registerGoalCallback(boost::bind(&FootstepManager::updateFeetGoalCB, this));
    update_feet_server_->registerPreemptCallback(boost::bind(&FootstepManager::updateFeetPreemptCB, this));
    update_feet_server_->start();

    generate_feet_pose_server_       = new GenerateFeetPoseServer(nh, "generate_feet_pose",false);
    generate_feet_pose_server_->registerGoalCallback(boost::bind(&FootstepManager::generateFeetPoseGoalCB, this));
    generate_feet_pose_server_->registerPreemptCallback(boost::bind(&FootstepManager::generateFeetPosePreemptCB, this));
    generate_feet_pose_server_->start();

    // initialize all ros action clients
    std::string planner_ns, controller_ns;
    nhp.param<std::string>("planner_namespace",   planner_ns,   "/vigir/footstep_planning");
    nhp.param<std::string>("controller_namespace",controller_ns,"/robot_controllers/footstep_controller");

    ROS_INFO(" Connect planner action clients to %s",planner_ns.c_str());
    ROS_INFO(" Connect controller action clients to %s",controller_ns.c_str());

    // client for feet pose generator service
    generate_feet_pose_service_client_ = nh.serviceClient<vigir_footstep_planning_msgs::GenerateFeetPoseService>(planner_ns + "/generate_feet_pose");

    //generate feet considering intial goal
    generate_feet_pose_action_client_  = new GenerateFeetPoseClient(planner_ns+"/generate_feet_pose", true);
    //update feet considering intial goal
    update_feet_client_             = new UpdateFeetClient(planner_ns+"/update_feet", true);
    //plan request
    step_plan_request_client_       = new StepPlanRequestClient(planner_ns+"/step_plan_request", true);
    //edit step
    edit_step_client_               = new EditStepClient(planner_ns+"/edit_step", true);
    //update step plan
    update_step_plan_client_        = new UpdateStepPlanClient(planner_ns+"/update_step_plan", true);
    //get all parameter sets
    get_all_parameter_sets_client_  = new GetAllParameterSetsClient(planner_ns+"/params/get_all_parameter_sets", true);
    //execute step plan
    execute_step_plan_client_       = new ExecuteStepPlanClient(controller_ns+"/execute_step_plan", true);

    //wait for servers to come online
    while(!generate_feet_pose_action_client_->waitForServer(ros::Duration(5.0)) && ros::ok())
    {
        ROS_INFO("Waiting for the generate_feet_pose_action server to come up");
    }
    while(!update_feet_client_->waitForServer(ros::Duration(5.0)) && ros::ok())
    {
        ROS_INFO("Waiting for the update_feet server to come up");
    }
    while(!step_plan_request_client_->waitForServer(ros::Duration(5.0))&& ros::ok())
    {
        ROS_INFO("Waiting for the step_plan_request server to come up");
    }
    while(!edit_step_client_->waitForServer(ros::Duration(5.0))&& ros::ok())
    {
        ROS_INFO("Waiting for the edit_step server to come up");
    }
    while(!update_step_plan_client_->waitForServer(ros::Duration(5.0))&& ros::ok())
    {
        ROS_INFO("Waiting for the update_step_plan server to come up");
    }
    while(!get_all_parameter_sets_client_->waitForServer(ros::Duration(5.0))&& ros::ok())
    {
        ROS_INFO("Waiting for the get_all_parameter_sets server to come up");
    }
    while(!execute_step_plan_client_->waitForServer(ros::Duration(5.0))&& ros::ok())
    {
        ROS_INFO("Waiting for the execute_step_plan server to come up");
    }


    ROS_INFO("Onboard FootstepManager initialized!");

    // request parameter sets
    sendGetAllParameterSetsGoal();
    selected_footstep_parameter_set_ = "";

}

/// This function updates the pose of a single step
void FootstepManager::processFootstepPoseUpdate(const flor_ocs_msgs::OCSFootstepUpdate::ConstPtr& msg)
{
    // find step in the plan
    vigir_footstep_planning_msgs::Step step;

    // if it didn't find a step with the requested step_index, return
    unsigned int step_plan_index;
    if(!findStep(msg->footstep_id, step, step_plan_index))
    {
        ROS_ERROR("Cannot find valid set for this index=%d! Ignore footstep update!",msg->footstep_id);
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_STEP,"Invalid step");
        return;
    }
    if (msg->pose.header.stamp != current_step_plan_.header.stamp)
    {
        ROS_ERROR("The plan time stamps do not match step=%f plan=%f", msg->pose.header.stamp.toSec(), current_step_plan_.header.stamp.toSec());
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_MISMATCHED_TIMES,"Time stamp mismatch");
        return;
    }

    // update pose
    step.foot.pose = msg->pose.pose;

    // send goal
    sendEditStepGoal(current_step_plan_, step);
}

/// This function updates the feet goal poses as 2 6DOF poses
void FootstepManager::processFootstepPlanGoalUpdate(const flor_ocs_msgs::OCSFootstepPlanGoalUpdate::ConstPtr& plan_goal)
{
    // Valid goal for the feet
    ROS_INFO("OBFSM: processFootstepPlanGoalUpdate: Received updated feet goals at (%f, %f, %f) and (%f, %f, %f) with goal pose=(%f, %f, %f)",
             plan_goal->left_foot.pose.position.x,
             plan_goal->left_foot.pose.position.y,
             plan_goal->left_foot.pose.position.z,
             plan_goal->right_foot.pose.position.x,
             plan_goal->right_foot.pose.position.y,
             plan_goal->right_foot.pose.position.z,
             plan_goal->goal_pose.pose.position.x,plan_goal->goal_pose.pose.position.y,plan_goal->goal_pose.pose.position.z);

    {
        boost::recursive_mutex::scoped_lock lock(goal_mutex_);
        goal_.header.stamp = plan_goal->goal_pose.header.stamp;
        goal_.left.pose    = plan_goal->left_foot.pose;  // goal in robot feet frame (ankle)
        goal_.right.pose   = plan_goal->right_foot.pose; // goal in robot feet frame (ankle)
        goal_pose_         = plan_goal->goal_pose;
        active_goal_pose_  = plan_goal->goal_pose;
    }

    flor_ocs_msgs::OCSFootstepPlanRequest plan_request;
    {
        boost::recursive_mutex::scoped_lock lock(goal_mutex_);
        plan_request.plan_time          = goal_.header.stamp;
        plan_request.start_index        = 0;
    }
    {
        boost::recursive_mutex::scoped_lock lock(goal_mutex_);
        plan_request.parameter_set_name = selected_footstep_parameter_set_;
    }
    ROS_INFO("OBFSM: processFootstepPlanGoalUpdate : Invoke planner after getting new goal feet poses with time stamp=%f",plan_request.plan_time.toSec());
    onboard_footstep_plan_request_pub_.publish(plan_request);

}

/// This function sends a single pose, which is used to generate the feet goal poses based on terrain data
void FootstepManager::processFootstepPlanGoal(const flor_ocs_msgs::OCSFootstepPlanGoal::ConstPtr& plan_goal)
{
    // and then the end feet poses
    ROS_INFO("OBFSM: processFootstepPlanGoal : process new plan goal with time=%f",plan_goal->goal_pose.header.stamp.toSec());
    if (calculateGoal(plan_goal->goal_pose))
    {
        flor_ocs_msgs::OCSFootstepPlanRequest plan_request;
        {
            boost::recursive_mutex::scoped_lock lock(goal_mutex_);
            plan_request.plan_time          = goal_.header.stamp;
            plan_request.start_index        = 0;
        }
        {
            boost::recursive_mutex::scoped_lock lock(goal_mutex_);
            plan_request.parameter_set_name = selected_footstep_parameter_set_;
        }
        ROS_INFO("OBFSM: processFootstepPlanGoal : Invoke planner after getting new goal with time stamp=%f",plan_request.plan_time.toSec());
        onboard_footstep_plan_request_pub_.publish(plan_request);

    }
}

void FootstepManager::processFootstepParamSetSelected(const std_msgs::String::ConstPtr& msg)
{
    if(selected_footstep_parameter_set_ != msg->data || selected_footstep_parameter_set_ == "")
    {
        {
          boost::recursive_mutex::scoped_lock lock(param_mutex_);
          selected_footstep_parameter_set_ = msg->data;
        }
        ROS_INFO("Change requested parameter set to %s",msg->data.c_str());
        onboard_planner_parameter_set_pub_.publish(*msg);
    }
}

void FootstepManager::processFootstepPlanParameters(const flor_ocs_msgs::OCSFootstepPlanParameters::ConstPtr& msg)
{
    flor_ocs_msgs::OCSFootstepPlanParameters new_planner_config = *msg;
    if(planner_config_.edit_mode != new_planner_config.edit_mode ||
       planner_config_.max_steps != new_planner_config.max_steps ||
       planner_config_.max_time != new_planner_config.max_time ||
       planner_config_.path_length_ratio != new_planner_config.path_length_ratio ||
       planner_config_.use_3d_planning != new_planner_config.use_3d_planning)
    {
        {
          boost::recursive_mutex::scoped_lock lock(param_mutex_);
          planner_config_ = new_planner_config;
        }
        ROS_INFO("Change planner configuration parameters");
        onboard_planner_configuration_pub_.publish(planner_config_);
    }
}

bool FootstepManager::calculateGoal(const geometry_msgs::PoseStamped& goal_pose)
{
    vigir_footstep_planning_msgs::GenerateFeetPoseService feet_pose_service;
    feet_pose_service.request.request.header = goal_pose.header;
    feet_pose_service.request.request.pose   = goal_pose.pose;
    feet_pose_service.request.request.flags  = vigir_footstep_planning_msgs::FeetPoseRequest::FLAG_CURRENT_Z; // this is for 3D interaction

    if (!generate_feet_pose_service_client_.call(feet_pose_service.request, feet_pose_service.response))
    {
      ROS_ERROR("OBFSM:  calculateGoal: Can't call 'FeetPoseGenerator' service!");
      publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_GOAL,vigir_footstep_planning::toString(feet_pose_service.response.status));
      return false;
    }

    // Valid goal for the feet
    ROS_INFO("OBFSM: calculateGoal: Received updated feet goals at (%f, %f, %f) and (%f, %f, %f) with goal pose=(%f, %f, %f)",
             feet_pose_service.response.feet.left.pose.position.x,
             feet_pose_service.response.feet.left.pose.position.y,
             feet_pose_service.response.feet.left.pose.position.z,
             feet_pose_service.response.feet.right.pose.position.x,
             feet_pose_service.response.feet.right.pose.position.y,
             feet_pose_service.response.feet.right.pose.position.z,
             goal_pose.pose.position.x,goal_pose.pose.position.y,goal_pose.pose.position.z);

    boost::recursive_mutex::scoped_lock lock(goal_mutex_);
    goal_ = feet_pose_service.response.feet; // goal in robot feet frame (ankle)
    goal_.header = goal_pose.header; // retain the goal header
    goal_pose_   = goal_pose;
    active_goal_pose_ = goal_pose_;
    return true;
}

// This function accepts a new plan from the OCS side, ---validates----, and replaces existing plan
void FootstepManager::processFootstepPlanUpdate(const vigir_footstep_planning_msgs::StepPlan::ConstPtr& msg)
{
    ROS_WARN("OBFSM: processFootstepPlanUpdate : just updating the footstep plan without any additional validation! stamp=%f",msg->header.stamp.toSec());

    // Currently not doing any validation on onboard side!
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);
    current_step_plan_ = *msg;

}

// This function requests a new plan to the current goal using designated parameter set
void FootstepManager::processFootstepPlanRequest(const flor_ocs_msgs::OCSFootstepPlanRequest::ConstPtr& plan_request)
{

    if (plan_request->plan_time != goal_.header.stamp)
    {
        ROS_ERROR(" OBFSM:  processFootstepPlanRequest -plan_time=%f does not match goal time=%f!",plan_request->plan_time.toSec(), goal_.header.stamp.toSec());
        // send updated status to ocs
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_MISMATCHED_TIMES, "processFootstepPlanRequest invalid timestamps!");
        return;
    }

    start_step_index_ = plan_request->start_index;

    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    if(current_step_plan_.steps.size() == 0 || start_step_index_ > current_step_plan_.steps.back().step_index)
    {
        ROS_INFO("OBFSM:  Invalid starting index (%d) - plan from beginning", start_step_index_);
        start_step_index_ = -1;
    }


    if(start_step_index_ == -1)
    {
        // request a completely new plan starting from the robot position
        requestStepPlanFromRobot();
    }
    else if(start_step_index_ == current_step_plan_.steps.back().step_index)
    {
        // get last step
        vigir_footstep_planning_msgs::Step step = current_step_plan_.steps.back();
        // request a footstep plan starting from the last step
        requestStepPlanFromStep(step);
    }
    else
    {
        vigir_footstep_planning_msgs::Step step;
        // unlikely, but if start index is the very first step
        if(start_step_index_ == 0)
        {
            if(current_step_plan_.steps.size() > 1)
                step = current_step_plan_.steps[0];
            else
            {
                ROS_ERROR("Invalid starting step index = %d with %ld total steps in current plan", start_step_index_, current_step_plan_.steps.size());
                publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_STEP,"Invalid starting step - ignore plan request");
                return;
            }
        }
        else
        {
            // if it didn't find a step with the requested step_index, return
            unsigned int step_plan_index;
            if(!findStep(start_step_index_-1, step, step_plan_index))
                return;
        }

        requestStepPlanFromStep(step);
    }
}

bool FootstepManager::getStartFeet(vigir_footstep_planning_msgs::Feet& start_feet)
{
    // get start feet pose
    vigir_footstep_planning_msgs::GenerateFeetPoseService feet_pose_service;
    feet_pose_service.request.request.header.frame_id = "/world";
    feet_pose_service.request.request.header.stamp = ros::Time::now();
    feet_pose_service.request.request.flags = vigir_footstep_planning_msgs::FeetPoseRequest::FLAG_CURRENT;

    if (!generate_feet_pose_service_client_.call(feet_pose_service.request, feet_pose_service.response))
    {
      ROS_ERROR("OBFSM:  getStartFeet: Can't call 'FeetPoseGenerator'!");
      publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_START,"OBFSM:  getStartFeet: Can't call 'FeetPoseGenerator'!");
      return false;
    }

    // check result
    if (vigir_footstep_planning::hasError(feet_pose_service.response.status))
    {
      ROS_ERROR("Error occured while requesting current feet pose:\n%s", vigir_footstep_planning::toString(feet_pose_service.response.status).c_str());
      publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_START,vigir_footstep_planning::toString(feet_pose_service.response.status));
      return false;
    }
    else if (vigir_footstep_planning::hasWarning(feet_pose_service.response.status))
    {
      ROS_ERROR("Warning occured while requesting current feet pose:\n%s", vigir_footstep_planning::toString(feet_pose_service.response.status).c_str());
      publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_START,vigir_footstep_planning::toString(feet_pose_service.response.status));
      return false;
    }

    start_feet = feet_pose_service.response.feet;
    return true;
}

void FootstepManager::requestStepPlanFromRobot()
{
    vigir_footstep_planning_msgs::Feet start_feet;
    if (getStartFeet(start_feet))
        sendStepPlanRequestGoal(start_feet, goal_);
}

void FootstepManager::requestStepPlanFromStep(const vigir_footstep_planning_msgs::Step& step)
{
    // then we need to find the next step after the starting one
    vigir_footstep_planning_msgs::Step next_step;
    unsigned int step_plan_index;

    if(!findStep(step.step_index+1, next_step, step_plan_index))
        return;

    // first we get the start feet poses based on the selected step
    vigir_footstep_planning_msgs::Feet start;
    start.header = step.foot.header;
    start.left  = (step.foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT ? step.foot : next_step.foot);
    start.right = (step.foot.foot_index == vigir_footstep_planning_msgs::Foot::RIGHT ? step.foot : next_step.foot);

    // calculate
    unsigned char start_foot;
    if(next_step.foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT)
        start_foot = vigir_footstep_planning_msgs::StepPlanRequest::LEFT;
    else
        start_foot = vigir_footstep_planning_msgs::StepPlanRequest::RIGHT;

    sendStepPlanRequestGoal(start, goal_, next_step.step_index, start_foot);
}

// send updated status to ocs
void FootstepManager::publishPlannerStatus(const flor_ocs_msgs::OCSFootstepStatus::_status_type& status, const std::string& status_msg )
{
    flor_ocs_msgs::OCSFootstepStatus planner_status;
    planner_status.status     = status;
    planner_status.status_msg = status_msg;
    onboard_planner_status_pub_.publish(planner_status);
}

// action callbacks for goal feet pose update request
void FootstepManager::activeUpdateFeet()
{
    ROS_INFO("OBFSM:  activeUpdateFeet : Status changed to active.");
}

void FootstepManager::feedbackUpdateFeet(const vigir_footstep_planning_msgs::UpdateFeetFeedbackConstPtr& feedback)
{
    ROS_INFO("OBFSM:  feedbackUpdateFeet: Feedback received.");
    if (update_feet_server_->isActive())
    {
        update_feet_server_->publishFeedback(feedback);
    }
}

void FootstepManager::doneUpdateFeet(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateFeetResultConstPtr& result)
{
    ROS_INFO("OBFSM:  doneUpdateFeet : Got action response.\n");
    if (update_feet_server_->isActive())
    {
        ROS_INFO(" OBFSM:  doneUpdateFeet - set action result=%d (%s) from client state=%s!",result->status.error,result->status.error_msg.c_str(), state.toString().c_str());
        if (actionlib::SimpleClientGoalState::SUCCEEDED  == state.state_)
        {
            if(vigir_footstep_planning::hasError(result->status) && result->status.error != vigir_footstep_planning_msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL)
            {
                ROS_ERROR("UpdateFeet: Error occured!\n     %d: %s", result->status.error, vigir_footstep_planning::toString(result->status.error_msg).c_str());
                publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_GOAL, vigir_footstep_planning::toString(result->status));
                update_feet_server_->setAborted(*result);
            }
            else
            {
                ROS_INFO("OBFSM:  doneUpdateFeet = Success!\n   %d: %s", result->status.error, vigir_footstep_planning::toString(result->status.error_msg).c_str());
                update_feet_server_->setSucceeded(*result);

                // update the goal feet
                boost::recursive_mutex::scoped_lock lock(goal_mutex_);
                goal_ = result->feet;

            }
        }
        else if (actionlib::SimpleClientGoalState::ABORTED  == state.state_)
        {
            update_feet_server_->setAborted(*result);
        }
        else if (actionlib::SimpleClientGoalState::PREEMPTED  == state.state_)
        {
            update_feet_server_->setAborted(*result);

        }
        else if (actionlib::SimpleClientGoalState::RECALLED  == state.state_)
        {
            update_feet_server_->setAborted(*result);

        }
        else if (actionlib::SimpleClientGoalState::REJECTED  == state.state_)
        {
            update_feet_server_->setAborted(*result);
        }
        else
        {
            ROS_ERROR(" OBFSM: doneUpdateFeet  unhandled state=%s - with result %d: ^%s - abort server goal!",state.toString().c_str(), result->status.error, vigir_footstep_planning::toString(result->status.error_msg).c_str());
            update_feet_server_->setAborted(*result);
        }

    }
    else
    {
        ROS_ERROR(" OBFSM:  doneUpdateFeet - action server was not active! - client result=%d:%s with state=%s!", result->status.error, vigir_footstep_planning::toString(result->status.error_msg).c_str(),state.toString().c_str());

        if(vigir_footstep_planning::hasError(result->status) && result->status.error != vigir_footstep_planning_msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL)
        {
            ROS_ERROR("UpdateFeet: Error occured!\n     %d : %s", result->status.error, vigir_footstep_planning::toString(result->status.error_msg).c_str());
            publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_GOAL, vigir_footstep_planning::toString(result->status));
        }
        else
        {
            // update the goal feet
            ROS_INFO("OBFSM:  doneUpdateFeet = Success with no active action!\n   %d: %s", result->status.error, vigir_footstep_planning::toString(result->status.error_msg).c_str());
            boost::recursive_mutex::scoped_lock lock(goal_mutex_);
            goal_ = result->feet;

        }
    }
}

// action goal for StepPlanRequest (assumes only handling robot (ankle) frame)
void FootstepManager::sendStepPlanRequestGoal(vigir_footstep_planning_msgs::Feet start, vigir_footstep_planning_msgs::Feet goal, const unsigned int start_step_index, const unsigned char start_foot)
{

    if (ros::Time() == goal.header.stamp)
    {
        ROS_ERROR(" OBFSM:  sendStepPlanRequestGoal - invalid timestamp for the goal!");
        // send updated status to ocs
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_GOAL, "StepPlanRequest invalid goal!");
        return;
    }

    vigir_footstep_planning_msgs::StepPlanRequest request;

    request.header.frame_id = "/world";
    request.header.stamp = goal.header.stamp; // goal --> plan --> execute should use consistent time stamps

    request.start = start;
    request.goal  = goal;
    request.start_step_index = start_step_index;

    request.start_foot_selection = start_foot;

    {
        // Update the planner parameters used in this request
        boost::recursive_mutex::scoped_lock lock(param_mutex_);
        // default planning mode is 2D, but will get that from the OCS
        if(planner_config_.use_3d_planning)
            request.planning_mode = vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_3D;
        else
            request.planning_mode = vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_2D;

        // need to get the following from the OCS as well
        request.max_planning_time     = planner_config_.max_time;
        request.max_number_steps      = planner_config_.max_steps;
        request.max_path_length_ratio = planner_config_.path_length_ratio;

        // and then use the selected parameter set
        request.parameter_set_name.data = selected_footstep_parameter_set_;
    }

    // Fill in goal here
    vigir_footstep_planning_msgs::StepPlanRequestGoal action_goal;
    action_goal.plan_request = request;

    // and send it to the server
    if(step_plan_request_client_->isServerConnected())
    {
        ROS_INFO("StepPlanRequest: Sending action goal...");
        step_plan_request_client_->sendGoal(action_goal,
                                            boost::bind(&FootstepManager::doneStepPlanRequest, this, _1, _2),
                                            boost::bind(&FootstepManager::activeStepPlanRequest, this),
                                            boost::bind(&FootstepManager::feedbackStepPlanRequest, this, _1));
    }
    else
    {
        ROS_INFO("StepPlanRequest: Server not connected!");
        // send updated status to ocs
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED, " StepPlanRequest Server not connected!");
    }

    // send updated status to ocs
    publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ACTIVE, "Sending StepPlanRequest action goal...");
}

// action callbacks for step plan request
void FootstepManager::activeStepPlanRequest()
{
    ROS_INFO("OBFSM:  activeStepPlanRequest: Status changed to active.");
    publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ACTIVE, "Footstep planning is active");
}

void FootstepManager::feedbackStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequestFeedbackConstPtr& feedback)
{
    ROS_INFO("OBFSM:  feedbackStepPlanRequest: Feedback received. number visited steps = %ld",feedback->feedback.visited_steps.size()  );
    if (step_plan_request_server_->isActive())
    {
        step_plan_request_server_->publishFeedback(feedback);
    }
}

void FootstepManager::doneStepPlanRequest(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::StepPlanRequestResultConstPtr& result)
{
    //ROS_INFO("OBFSM:  doneStepPlanRequest: Got action response.");

    if(vigir_footstep_planning::hasError(result->status))
    {
        ROS_ERROR("OBFSM:  doneStepPlanRequest: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED, vigir_footstep_planning::toString(result->status));
        if (step_plan_request_server_->isActive())
        {
            step_plan_request_server_->setAborted(*result);
        }
    }
    else
    {
        // send update to glance_hub "done"
        if (step_plan_request_server_->isActive())
        {
            ROS_INFO("OBFSM:  doneStepPlanRequest: - set action result= %d:%s  from client state=%s!",result->status.error, result->status.error_msg.c_str(), state.toString().c_str());

            if (actionlib::SimpleClientGoalState::SUCCEEDED  == state.state_)
            {

                boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

                if(result->step_plan.header.stamp == goal_.header.stamp)
                {
                    step_plan_request_server_->setSucceeded(*result);
                    vigir_footstep_planning_msgs::StepPlan plan = result->step_plan;
                    if (processNewStepPlan(plan))
                    {
                        last_onboard_step_plan_stamp_ = result->step_plan.header.stamp;
                    }

                }
                else
                {
                    step_plan_request_server_->setAborted(*result);
                    ROS_ERROR("OBFSM:  doneStepPlanRequest: Ignoring plan with timestamp that does not match the goal! (plan=%f, goal=%f).",
                              result->step_plan.header.stamp.toSec(), goal_.header.stamp.toSec());
                    publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_MISMATCHED_TIMES, "Ignore mis-matched time stamps");
                }

            }
            else if (actionlib::SimpleClientGoalState::ABORTED  == state.state_)
            {
                step_plan_request_server_->setAborted(*result);
            }
            else if (actionlib::SimpleClientGoalState::PREEMPTED  == state.state_)
            {
                step_plan_request_server_->setAborted(*result);

            }
            else if (actionlib::SimpleClientGoalState::RECALLED  == state.state_)
            {
                step_plan_request_server_->setAborted(*result);

            }
            else if (actionlib::SimpleClientGoalState::REJECTED  == state.state_)
            {
                step_plan_request_server_->setAborted(*result);
            }
            else
            {
                ROS_ERROR("OBFSM:  doneStepPlanRequest:  unhandled state=%s - abort server goal!",state.toString().c_str());
                step_plan_request_server_->setAborted(*result);
            }

        }
        else
        {
            ROS_INFO("OBFSM:  doneStepPlanRequest:Received new step plan with time stamp=%f - must have been triggered by OCS!", result->step_plan.header.stamp.toSec());
            boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

            if(result->step_plan.header.stamp == goal_.header.stamp)
            {
                vigir_footstep_planning_msgs::StepPlan plan = result->step_plan;
                if (processNewStepPlan(plan))
                {
                    last_onboard_step_plan_stamp_ = result->step_plan.header.stamp;
                }
            }
            else
            {
                ROS_ERROR("OBFSM:  doneStepPlanRequest: Ignoring repeated plan (plan=%f, goal=%f).",
                          result->step_plan.header.stamp.toSec(), goal_.header.stamp.toSec());
                publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED, "Ignore repeated plan");
            }
        }

    }
}

// action goal for EditStep
void FootstepManager::sendEditStepGoal(vigir_footstep_planning_msgs::StepPlan step_plan, vigir_footstep_planning_msgs::Step step)
{

    // Fill in action goal here
    vigir_footstep_planning_msgs::EditStepGoal action_goal;

    {
        boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);
        action_goal.step_plan = current_step_plan_;
    }

    if(planner_config_.edit_mode)
        action_goal.edit_step.plan_mode = vigir_footstep_planning_msgs::EditStep::EDIT_MODE_FULL;
    else
        action_goal.edit_step.plan_mode = vigir_footstep_planning_msgs::EditStep::EDIT_MODE_3D;
    action_goal.edit_step.step = step;

    // and send it to the server
    if(edit_step_client_->isServerConnected())
    {
        edit_step_client_->sendGoal(action_goal,
                                    boost::bind(&FootstepManager::doneEditStep, this, _1, _2),
                                    boost::bind(&FootstepManager::activeEditStep, this),
                                    boost::bind(&FootstepManager::feedbackEditStep, this, _1));
    }
    else
    {
        ROS_INFO("OBFSM: sendEditStepGoal: Server not connected!");
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ERROR, "EditStep: Server not connected!");
    }
}

// action callbacks
void FootstepManager::activeEditStep()
{
    ROS_INFO("OBFSM:  EditStep: Status changed to active.");
}

void FootstepManager::feedbackEditStep(const vigir_footstep_planning_msgs::EditStepFeedbackConstPtr& feedback)
{
    ROS_INFO("OBFSM:  EditStep: Feedback received.");
}

void FootstepManager::doneEditStep(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::EditStepResultConstPtr& result)
{
    ROS_INFO("OBFSM:  EditStep: Got action response.");

    if (vigir_footstep_planning::hasError(result->status) && result->status.error != vigir_footstep_planning_msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL)
    {
        ROS_ERROR("OBFSM:  EditStep: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED, vigir_footstep_planning::toString(result->status));
    }
    else
    {
        if(result->step_plans.size() == 0)
        {
            ROS_ERROR("OBFSM:  EditStep: Received no step plan.");
            publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED, vigir_footstep_planning::toString(result->status));
            return;
        }

        if(result->step_plans[0].steps.size() == 0)
        {
            ROS_ERROR("EditStep: Received empty step plan.");
            publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED, vigir_footstep_planning::toString(result->status));
            return;
        }

        boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);
        current_step_plan_ = result->step_plans[0];
        onboard_footstep_plan_pub_.publish(current_step_plan_);
    }
}

// action goal for step plan
void FootstepManager::sendUpdateStepPlanGoal(vigir_footstep_planning_msgs::StepPlan step_plan)
{
    //convert transform to ankle for planner
    foot_pose_transformer_->transformToRobotFrame(step_plan);

    // Fill in goal here
    vigir_footstep_planning_msgs::UpdateStepPlanGoal action_goal;
    action_goal.step_plan = step_plan;
    action_goal.parameter_set_name.data = selected_footstep_parameter_set_;
    action_goal.update_mode.mode = vigir_footstep_planning_msgs::UpdateMode::UPDATE_MODE_REPLAN;

    // and send it to the server
    if(update_step_plan_client_->isServerConnected())
    {
        update_step_plan_client_->sendGoal(action_goal,
                                           boost::bind(&FootstepManager::doneUpdateStepPlan, this, _1, _2),
                                           boost::bind(&FootstepManager::activeUpdateStepPlan, this),
                                           boost::bind(&FootstepManager::feedbackUpdateStepPlan, this, _1));
    }
    else
    {
        ROS_INFO("UpdateStepPlan: Server not connected!");
    }
}

// action callbacks
void FootstepManager::activeUpdateStepPlan()
{
    ROS_INFO("UpdateStepPlan: Status changed to active.");
}

void FootstepManager::feedbackUpdateStepPlan(const vigir_footstep_planning_msgs::UpdateStepPlanFeedbackConstPtr& feedback)
{
    ROS_INFO("UpdateStepPlan: Feedback received.");
}

void FootstepManager::doneUpdateStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateStepPlanResultConstPtr& result)
{
    ROS_INFO("OBFSM:  UpdateStepPlan: Got action response.");

    if(vigir_footstep_planning::hasError(result->status))
    {
        ROS_ERROR("   OBFSM:  UpdateStepPlan: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED, vigir_footstep_planning::toString(result->status));
    }
    else
    {
        ROS_INFO("   OBFSM:  UpdateStepPlan: result %s with %ld steps", vigir_footstep_planning::toString(result->status).c_str(), result->step_plan.steps.size());
        processNewStepPlan(result->step_plan);

    }
}

// action goal for executestep
void FootstepManager::sendExecuteStepPlanGoal()
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    // need to make sure we only have step plan with steps
    if(current_step_plan_.steps.size() < 2 || (ros::Time() == current_step_plan_.header.stamp) )
    {
        // send updated status to ocs
        ROS_ERROR("OBFSM:  sendExecuteStepPlanGoal failed - Can't execute empty plan ");
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_EXECUTION_FAILED, "Can't execute empty plan ");
        return;
    }

    // also need to make sure this plan hasn't been sent before
    if(last_executed_step_plan_stamp_ == current_step_plan_.header.stamp)
    {
        // send updated status to ocs
        ROS_ERROR("OBFSM:  sendExecuteStepPlanGoal failed - Can't execute same plan twice. ");
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_EXECUTION_FAILED, "Can't execute same plan twice.");
        return;
    }


    if(execute_step_plan_client_->isServerConnected())
    {
        // Fill in goal here and send it to the server
        vigir_footstep_planning_msgs::ExecuteStepPlanGoal action_goal;
        action_goal.step_plan = current_step_plan_;

        // save the last plan executed timestamp
        last_executed_step_plan_stamp_ = current_step_plan_.header.stamp;

        execute_step_plan_client_->sendGoal(action_goal,
                                            boost::bind(&FootstepManager::doneExecuteStepPlan, this, _1, _2),
                                            boost::bind(&FootstepManager::activeExecuteStepPlan, this),
                                            boost::bind(&FootstepManager::feedbackExecuteStepPlan, this, _1));
    }
    else
    {
        ROS_INFO("OBFSM:  sendExecuteStepPlanGoal: Footstep Execution Action Server not connected!");
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_EXECUTION_FAILED, "Footstep Execution Action Server not connected");
    }
}

// action callbacks
void FootstepManager::activeExecuteStepPlan()
{
    ROS_INFO("OBFSM:  ExecuteStepPlan: Status changed to active.");
    publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_EXECUTION_ACTIVE, "Execution of footstep plan active");
}

void FootstepManager::feedbackExecuteStepPlan(const vigir_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback)
{
    if (execute_step_plan_server_->isActive())
    {
        execute_step_plan_server_->publishFeedback(feedback);
    }
    else
    {
        ROS_ERROR(" OBFSM:  feedbackExecuteStepPlan - action server was not active!");
    }

    // send updated status to ocs
    flor_ocs_msgs::OCSFootstepStatus planner_status;
    planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_EXECUTION_ACTIVE;
    planner_status.status_msg = "Execution feedback received: Step "+boost::lexical_cast<std::string>(feedback->last_performed_step_index)+" -> Step "+boost::lexical_cast<std::string>(feedback->currently_executing_step_index);

    ROS_INFO("OBFSM: ExecuteStepPlan: %s", planner_status.status_msg.c_str());
    publishPlannerStatus(planner_status.status, planner_status.status_msg);

}

void FootstepManager::doneExecuteStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result)
{
    ROS_INFO("OBFSM:  doneExecuteStepPlan: Got action response.");

    if (execute_step_plan_server_->isActive())
    {
        ROS_INFO(" OBFSM:  doneExecuteStepPlan - set action result=%d  from client state=%s!",result->status.status, state.toString().c_str());
        if (actionlib::SimpleClientGoalState::SUCCEEDED  == state.state_)
        {
            execute_step_plan_server_->setSucceeded(*result);
        }
        else if (actionlib::SimpleClientGoalState::ABORTED  == state.state_)
        {
            execute_step_plan_server_->setAborted(*result);

        }
        else if (actionlib::SimpleClientGoalState::PREEMPTED  == state.state_)
        {
            execute_step_plan_server_->setAborted(*result);

        }
        else if (actionlib::SimpleClientGoalState::RECALLED  == state.state_)
        {
            execute_step_plan_server_->setAborted(*result);

        }
        else if (actionlib::SimpleClientGoalState::REJECTED  == state.state_)
        {
            execute_step_plan_server_->setAborted(*result);
        }
        else
        {
            ROS_ERROR(" OBFSM: doneExecuteStepPlan  unhandled state=%s - abort server goal!",state.toString().c_str());
            execute_step_plan_server_->setAborted(*result);
        }

    }
    else
    {
        ROS_ERROR(" OBFSM:  doneExecuteStepPlan - action server was not active! - client result=%d with state=%s!",result->status.status,state.toString().c_str());
    }

    if(!(result->status.status & vigir_footstep_planning_msgs::FootstepExecutionStatus::REACHED_GOAL))
    {
        ROS_ERROR("ExecuteStepPlan: Error occured!\nExecution error code: %d", result->status.status);

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_EXECUTION_FAILED;
        planner_status.status_msg = "Execution error code: "+boost::lexical_cast<std::string>(result->status.status);
        publishPlannerStatus(planner_status.status, planner_status.status_msg);
    }
    else
    {
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_EXECUTION_SUCCESS;
        planner_status.status_msg = "Succesfully reached goal! Status: "+boost::lexical_cast<std::string>(result->status.status);
        publishPlannerStatus(planner_status.status, planner_status.status_msg);
    }
}

void FootstepManager::sendGetAllParameterSetsGoal()
{
    // Fill in goal here
    vigir_footstep_planning_msgs::GetAllParameterSetsGoal action_goal;

    // and send it to the server
    if(get_all_parameter_sets_client_->isServerConnected())
    {
        get_all_parameter_sets_client_->sendGoal(action_goal,
                                                 boost::bind(&FootstepManager::doneGetAllParameterSets, this, _1, _2),
                                                 boost::bind(&FootstepManager::activeGetAllParameterSets, this),
                                                 boost::bind(&FootstepManager::feedbackGetAllParameterSets, this, _1));
    }
    else
    {
        ROS_INFO("OBFSM:  GetAllParameterSets: Server not connected!");
    }
}

void FootstepManager::activeGetAllParameterSets()
{
    ROS_INFO("OBFSM:  GetAllParameterSets: Status changed to active.");
}

void FootstepManager::feedbackGetAllParameterSets(const vigir_footstep_planning_msgs::GetAllParameterSetsFeedbackConstPtr& feedback)
{
    ROS_INFO("OBFSM:  GetAllParameterSets: Feedback received.");
}

void FootstepManager::doneGetAllParameterSets(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::GetAllParameterSetsResultConstPtr& result)
{
    ROS_INFO("OBFSM:  GetAllParameterSets: Got action response. %s", result->status.error_msg.c_str());

    if(vigir_footstep_planning::hasError(result->status))
    {
        ROS_ERROR("OBFSM:  GetAllParameterSets: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = result->status.error_msg;
        publishPlannerStatus(planner_status.status, planner_status.status_msg);
    }
    else
    {
        boost::recursive_mutex::scoped_lock lock(param_mutex_);

        footstep_parameter_set_list_.clear();
        for(int i = 0; i < result->param_sets.size(); i++)
            footstep_parameter_set_list_.push_back(result->param_sets[i]);

        //this->publishFootstepParameterSetList();
    }
}


bool FootstepManager::processNewStepPlan(const vigir_footstep_planning_msgs::StepPlan& step_plan)
{

    ROS_INFO("OBFSM: processNewStepPlan: Processing new step plan (%d, %d).", step_plan.header.stamp.sec, step_plan.header.stamp.nsec);

    if(step_plan.steps.size() == 0)
    {
        ROS_ERROR("processNewStepPlan: Received empty step plan.");
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED, "Empty step plan!");
        return false;
    }

    // update goal pose
    vigir_footstep_planning_msgs::Feet goal;
    goal.header = step_plan.steps[step_plan.steps.size()-1].header;
    goal.left  = step_plan.steps[step_plan.steps.size()-1].foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT ?
                 step_plan.steps[step_plan.steps.size()-1].foot :
                 step_plan.steps[step_plan.steps.size()-2].foot;
    goal.right = step_plan.steps[step_plan.steps.size()-1].foot.foot_index == vigir_footstep_planning_msgs::Foot::RIGHT ?
                 step_plan.steps[step_plan.steps.size()-1].foot :
                 step_plan.steps[step_plan.steps.size()-2].foot;

    publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_SUCCESS, "Footstep planner done");

    // update the goal feet with final plan data
    boost::recursive_mutex::scoped_lock goal_lock(goal_mutex_);
    goal_ = goal;

    boost::recursive_mutex::scoped_lock plan_lock(step_plan_mutex_);
    current_step_plan_ = step_plan;

    onboard_footstep_plan_pub_.publish(current_step_plan_);
    return true;
}

// utilities

// returns step and the step plan it's contained in based on a step_index
bool FootstepManager::findStep(const unsigned int& step_index, vigir_footstep_planning_msgs::Step &step, unsigned int& step_plan_index)
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    // look for step with step index
    for(int j = 0; j < current_step_plan_.steps.size(); j++)
    {
        if(step_index == current_step_plan_.steps[j].step_index)
        {
            step = current_step_plan_.steps[j];
            step_plan_index = -1;
            return true;
        }
    }
    return false;
}

void FootstepManager::stepPlanRequestGoalCB()
{
    // New goal - invoke the planner and register callbacks
    vigir_footstep_planning_msgs::StepPlanRequestGoal action_goal = *(step_plan_request_server_->acceptNewGoal());

    // trigger execution of client
    if(step_plan_request_client_->isServerConnected())
    {
        // Fill in goal here and send it to the server (only the time stamp and parameters were really relevant)

        // lock the goal
        boost::recursive_mutex::scoped_lock goal_lock(goal_mutex_);

        // Update the planning parameters
        vigir_footstep_planning_msgs::StepPlanRequest& request = action_goal.plan_request;

        if ((request.header.stamp == goal_.header.stamp) || (vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_PATTERN  == request.planning_mode))
        {  // goal --> plan --> execute should use consistent time stamps, but pattern step overrides the current goal/step pattern

            request.header.frame_id = "/world";

            if (!getStartFeet(request.start))
            { // failed to get start feet!
                vigir_footstep_planning_msgs::StepPlanRequestResult result;
                result.status.error = vigir_footstep_planning_msgs::ErrorStatus::ERR_INVALID_START;
                step_plan_request_server_->setAborted(result);

                ROS_INFO("OBFSM:  stepPlanRequestGoalCB: invalid start location for feet!" );
                publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ERROR, "Footstep planning - invalid start location for feet!");

                return;
            }

            request.goal  = goal_;
            request.start_step_index = 0;  // always start from current steps

            request.start_foot_selection = vigir_footstep_planning_msgs::StepPlanRequest::AUTO;

            {
                // Update the planner parameters used in this request
                boost::recursive_mutex::scoped_lock lock(param_mutex_);
                // default planning mode is 2D, but will get that from the OCS
                if (vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_PATTERN  != request.planning_mode)
                { // plan based on setup defined by operator, but retain for pattern requests
                    if(planner_config_.use_3d_planning)
                        request.planning_mode = vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_3D;
                    else
                        request.planning_mode = vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_2D;
                    ROS_INFO(" Using planning mode = %d as specified by operator",request.planning_mode);
                }
                else
                {
                    boost::recursive_mutex::scoped_lock plan_lock(step_plan_mutex_);
                    current_step_plan_.header.stamp = ros::Time(); // reset when requesting a pattern move to force regular replan latter if this fails
                    current_step_plan_.steps.clear();

                    boost::recursive_mutex::scoped_lock lock(goal_mutex_);
                    goal_pose_.header.stamp = request.header.stamp;
                    goal_.header.stamp      = request.header.stamp;
                    ROS_INFO("Reset goal stamp on pattern request to %f",goal_.header.stamp.toSec());
                }

                // need to get the following from the OCS as well
                request.max_planning_time     = planner_config_.max_time;
                request.max_number_steps      = planner_config_.max_steps;
                request.max_path_length_ratio = planner_config_.path_length_ratio;

                // and then use the selected parameter set
                request.parameter_set_name.data = selected_footstep_parameter_set_;
            }

            // Fill in goal here
            action_goal.plan_request = request;

            step_plan_request_client_->sendGoal(action_goal,
                                                boost::bind(&FootstepManager::doneStepPlanRequest, this, _1, _2),
                                                boost::bind(&FootstepManager::activeStepPlanRequest, this),
                                                boost::bind(&FootstepManager::feedbackStepPlanRequest, this, _1));

        }
        else
        {
            vigir_footstep_planning_msgs::StepPlanRequestResult result;
            result.status.error = vigir_footstep_planning_msgs::ErrorStatus::ERR_INCONSISTENT_REQUEST;
            step_plan_request_server_->setAborted(result);

            ROS_INFO("OBFSM:  stepPlanRequestGoalCB: Footstep plan timestamp mismatch %f vs. %f!", action_goal.plan_request.header.stamp.toSec(), current_step_plan_.header.stamp.toSec());
            publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ERROR, "Footstep Execution invalid plan timestamps");
            return;
        }
    }
    else
    {
        vigir_footstep_planning_msgs::StepPlanRequestResult result;
        result.status.error = vigir_footstep_planning_msgs::ErrorStatus::ERR_UNKNOWN;
        step_plan_request_server_->setAborted(result);

        ROS_INFO("OBFSM:  stepPlanRequestGoalCB: Action Server not connected!");
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ERROR, "Footstep Planning Action Server not connected");
    }


}

void FootstepManager::stepPlanRequestPreemptCB()
{
    // Pre-empt any active client requests
    step_plan_request_client_->cancelGoal();

}
void FootstepManager::executeStepPlanGoalCB()
{

    vigir_footstep_planning_msgs::ExecuteStepPlanGoal action_goal = *(execute_step_plan_server_->acceptNewGoal());

    // trigger execution of client
    if(execute_step_plan_client_->isServerConnected())
    {
        // Fill in goal here and send it to the server (only the time stamp and parameters were really relevant)
        // lock the plan
        boost::recursive_mutex::scoped_lock plan_lock(step_plan_mutex_);

        if (action_goal.step_plan.header.stamp == current_step_plan_.header.stamp)
        {
            action_goal.step_plan = current_step_plan_;

            // save the last plan executed timestamp
            last_executed_step_plan_stamp_ = current_step_plan_.header.stamp;

            execute_step_plan_client_->sendGoal(action_goal,
                                            boost::bind(&FootstepManager::doneExecuteStepPlan, this, _1, _2),
                                            boost::bind(&FootstepManager::activeExecuteStepPlan, this),
                                            boost::bind(&FootstepManager::feedbackExecuteStepPlan, this, _1));
        }
        else
        {
            vigir_footstep_planning_msgs::ExecuteStepPlanResult result;
            result.status.status = vigir_footstep_planning_msgs::FootstepExecutionStatus::ERR_INVALID_PLAN;
            execute_step_plan_server_->setAborted(result);
            ROS_INFO("OBFSM:  executeStepPlanGoalCB: Footstep plan timestamp mismatch %f vs. %f!", action_goal.step_plan.header.stamp.toSec(), current_step_plan_.header.stamp.toSec());
            publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_EXECUTION_FAILED, "Footstep Execution invalid plan timestamps");
            return;
        }
    }
    else
    {
        vigir_footstep_planning_msgs::ExecuteStepPlanResult result;
        result.status.status = vigir_footstep_planning_msgs::FootstepExecutionStatus::ERR_CONTROLLER_NOT_READY;
        execute_step_plan_server_->setAborted(result);

        ROS_INFO("OBFSM:  executeStepPlanGoalCB: Footstep Execution Action Server not connected!");
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_EXECUTION_FAILED, "Footstep Execution Action Server not connected");
    }

}
void FootstepManager::executeStepPlanPreemptCB()
{
    // pre-empt any active clients
    ROS_INFO("Cancel any existing step plan executions!");
    execute_step_plan_client_->cancelGoal();
}

void FootstepManager::updateFeetGoalCB()
{

    vigir_footstep_planning_msgs::UpdateFeetGoal action_goal = *(update_feet_server_->acceptNewGoal());

    // trigger execution of client
    if(update_feet_client_->isServerConnected())
    {
        // Fill in goal here and send it to the server (only the time stamp and parameters were really relevant)


        update_feet_client_->sendGoal(action_goal,
                                      boost::bind(&FootstepManager::doneUpdateFeet, this, _1, _2),
                                      boost::bind(&FootstepManager::activeUpdateFeet, this),
                                      boost::bind(&FootstepManager::feedbackUpdateFeet, this, _1));
    }
    else
    {
        vigir_footstep_planning_msgs::UpdateFeetResult result;
        result.status.error =    vigir_footstep_planning_msgs::ErrorStatus::ERR_UNKNOWN;
        result.status.error_msg = "Update Feet  Server not connected";
        update_feet_server_->setAborted(result);

        ROS_INFO("OBFSM:  updateFeetGoalCB: Footstep Update Feet Server not connected!");
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_GOAL, "Footstep Update Feet  Server not connected");
    }

}
void FootstepManager::updateFeetPreemptCB()
{
    // pre-empt any active clients
    ROS_INFO("Cancel any existing footstep goals executions!");
    update_feet_client_->cancelGoal();
}

// action callbacks for goal feet pose update request
void FootstepManager::activeGenerateFeetPose()
{
    ROS_INFO("OBFSM:  activeGenerateFeetPose : Status changed to active.");
}

void FootstepManager::feedbackGenerateFeetPose(const vigir_footstep_planning_msgs::GenerateFeetPoseFeedbackConstPtr& feedback)
{
    ROS_INFO("OBFSM:  feedbackGenerateFeetPose: Feedback received.");
    if (generate_feet_pose_server_->isActive())
    {
        generate_feet_pose_server_->publishFeedback(feedback);
    }
}

void FootstepManager::doneGenerateFeetPose(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::GenerateFeetPoseResultConstPtr& result)
{
    ROS_INFO("OBFSM:  doneGenerateFeetPose : Got action response.\n");
    if (generate_feet_pose_server_->isActive())
    {
        ROS_INFO(" OBFSM:  doneGenerateFeetPose - set action result=%d (%s) from client state=%s!",result->status.error,result->status.error_msg.c_str(), state.toString().c_str());
        if (actionlib::SimpleClientGoalState::SUCCEEDED  == state.state_)
        {
            if(vigir_footstep_planning::hasError(result->status) && result->status.error != vigir_footstep_planning_msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL)
            {
                ROS_ERROR("GenerateFeetPose: Error occured!\n     %d: %s", result->status.error, vigir_footstep_planning::toString(result->status.error_msg).c_str());
                publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_GOAL, vigir_footstep_planning::toString(result->status));
                generate_feet_pose_server_->setAborted(*result);
            }
            else
            {
                ROS_INFO("OBFSM:  doneGenerateFeetPose = Success!\n   %d: %s", result->status.error, vigir_footstep_planning::toString(result->status.error_msg).c_str());
                generate_feet_pose_server_->setSucceeded(*result);

                // update the goal feet
                boost::recursive_mutex::scoped_lock lock(goal_mutex_);
                goal_pose_ = active_goal_pose_;
                goal_ = result->feet;
                goal_.header = result->header;

            }
        }
        else if (actionlib::SimpleClientGoalState::ABORTED  == state.state_)
        {
            generate_feet_pose_server_->setAborted(*result);
        }
        else if (actionlib::SimpleClientGoalState::PREEMPTED  == state.state_)
        {
            generate_feet_pose_server_->setAborted(*result);

        }
        else if (actionlib::SimpleClientGoalState::RECALLED  == state.state_)
        {
            generate_feet_pose_server_->setAborted(*result);

        }
        else if (actionlib::SimpleClientGoalState::REJECTED  == state.state_)
        {
            generate_feet_pose_server_->setAborted(*result);
        }
        else
        {
            ROS_ERROR(" OBFSM: doneGenerateFeetPose  unhandled state=%s - with result %d: ^%s - abort server goal!",state.toString().c_str(), result->status.error, vigir_footstep_planning::toString(result->status.error_msg).c_str());
            generate_feet_pose_server_->setAborted(*result);
        }

    }
    else
    {
        ROS_ERROR(" OBFSM:  doneGenerateFeetPose - action server was not active! - client result=%d:%s with state=%s!", result->status.error, vigir_footstep_planning::toString(result->status.error_msg).c_str(),state.toString().c_str());

        if(vigir_footstep_planning::hasError(result->status) && result->status.error != vigir_footstep_planning_msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL)
        {
            ROS_ERROR("GenerateFeetPose: Error occured!\n     %d : %s", result->status.error, vigir_footstep_planning::toString(result->status.error_msg).c_str());
            publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_GOAL, vigir_footstep_planning::toString(result->status));
        }
        else
        {
            // update the goal feet
            ROS_INFO("OBFSM:  doneGenerateFeetPose = Success with no active action!\n   %d: %s", result->status.error, vigir_footstep_planning::toString(result->status.error_msg).c_str());
            boost::recursive_mutex::scoped_lock lock(goal_mutex_);
            goal_ = result->feet;
            goal_.header = result->header; // retain the goal time stamp

        }
    }
}

void FootstepManager::generateFeetPoseGoalCB()
{

    // trigger execution of client
    if(generate_feet_pose_action_client_->isServerConnected())
    {
        // Fill in goal here and send it to the server
        vigir_footstep_planning_msgs::GenerateFeetPoseGoal action_goal = *(generate_feet_pose_server_->acceptNewGoal());

        generate_feet_pose_action_client_->sendGoal(action_goal,
                                      boost::bind(&FootstepManager::doneGenerateFeetPose, this, _1, _2),
                                      boost::bind(&FootstepManager::activeGenerateFeetPose, this),
                                      boost::bind(&FootstepManager::feedbackGenerateFeetPose, this, _1));
    }
    else
    {
        vigir_footstep_planning_msgs::GenerateFeetPoseResult result;
        result.status.error =    vigir_footstep_planning_msgs::ErrorStatus::ERR_UNKNOWN;
        result.status.error_msg = "Generate Feet Pose  Server not connected";
        generate_feet_pose_server_->setAborted(result);

        ROS_INFO("OBFSM:  generateFeetPoseGoalCB: Footstep Generate Feet Pose Server not connected!");
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_GOAL, "Footstep Update Feet  Server not connected");
    }

}
void FootstepManager::generateFeetPosePreemptCB()
{
    // pre-empt any active clients
    ROS_INFO("Cancel any existing footstep feet pose requests!");
    generate_feet_pose_action_client_->cancelGoal();
}


}

PLUGINLIB_DECLARE_CLASS (vigir_onboard_footstep_manager, FootstepManager, onboard_footstep::FootstepManager, nodelet::Nodelet)
