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

    // planner will always create new plans if this is unchanged
    start_step_index_ = -1;

    // creates publishers and subscribers for the interaction loop
    onboard_footstep_plan_pub_          = nh.advertise<vigir_footstep_planning_msgs::StepPlan>("current_step_plan", 1, false);
    onboard_planner_status_pub_         = nh.advertise<flor_ocs_msgs::OCSFootstepStatus>("step_planner_status",1,false);
    onboard_planner_parameter_set_pub_  = nh.advertise<std_msgs::String>("active_parameter_set",1,false);
    onboard_planner_configuration_pub_  = nh.advertise<flor_ocs_msgs::OCSFootstepPlanParameters>("planner_configuration",1,false);

    ocs_footstep_update_sub_             = nh.subscribe<flor_ocs_msgs::OCSFootstepUpdate         >("step_update",              1, &FootstepManager::processFootstepPoseUpdate      , this);
    ocs_footstep_plan_goal_update_sub_   = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanGoalUpdate >("update_step_plan_goal",    1, &FootstepManager::processFootstepPlanGoalUpdate  , this);
    ocs_footstep_plan_goal_sub_          = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanGoal       >("plan_goal",                1, &FootstepManager::processFootstepPlanGoal        , this);
    ocs_footstep_plan_parameters_sub_    = nh.subscribe<flor_ocs_msgs::OCSFootstepPlanParameters >("plan_parameters",          1, &FootstepManager::processFootstepPlanParameters  , this);
    ocs_footstep_set_parameter_set_sub_  = nh.subscribe<std_msgs::String                         >("set_active_parameter_set", 1, &FootstepManager::processFootstepParamSetSelected, this);
    ocs_footstep_plan_sub_               = nh.subscribe<vigir_footstep_planning_msgs::StepPlan   >("updated_step_plan",        1, &FootstepManager::processFootstepPlanUpdate      , this);
    ocs_footstep_plan_request_sub_       = nh.subscribe<std_msgs::String                         >("replan_request",           1, &FootstepManager::processFootstepPlanRequest     , this);

    // Start the action servers that can be triggered by OCS or behaviors
    step_plan_request_server_ = new StepPlanRequestServer(nh, "replan_request",    boost::bind(&FootstepManager::executeStepPlanRequestCB, this, _1), false);
    execute_step_plan_server_ = new ExecuteStepPlanServer(nh, "execute_step_plan", boost::bind(&FootstepManager::executeExecuteStepPlanCB, this, _1), false);
    step_plan_request_server_->start();
    execute_step_plan_server_->start();

    // client for feet pose generator service
    generate_feet_pose_client = nh.serviceClient<vigir_footstep_planning_msgs::GenerateFeetPoseService>("generate_feet_pose");

    // initialize all ros action clients
    //update feet considering intial goal
    update_feet_client_             = new UpdateFeetClient("update_feet", true);
    //plan request
    step_plan_request_client_       = new StepPlanRequestClient("step_plan_request", true);
    //edit step
    edit_step_client_               = new EditStepClient("edit_step", true);
    //update step plan
    update_step_plan_client_        = new UpdateStepPlanClient("update_step_plan", true);
    //execute step plan
    execute_step_plan_client_       = new ExecuteStepPlanClient("/robot_controllers/footstep_controller/execute_step_plan", true);
    //get all parameter sets
    get_all_parameter_sets_client_  = new GetAllParameterSetsClient("get_all_parameter_sets", true);

    //wait for servers to come online
    while(!update_feet_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the update_feet server to come up");
    }
    while(!step_plan_request_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the step_plan_request server to come up");
    }
    while(!edit_step_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the edit_step server to come up");
    }
    while(!update_step_plan_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the update_step_plan server to come up");
    }
    while(!get_all_parameter_sets_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the get_all_parameter_sets server to come up");
    }
    while(!execute_step_plan_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the execute_step_plan server to come up");
    }


    ROS_INFO("Onboard FootstepManager initialized!");

    // request parameter sets
    sendGetAllParameterSetsGoal();
    selected_footstep_parameter_set_ = "";

//    timer = nh.createTimer(ros::Duration(1), &FootstepManager::timerCallback, this);
}

//void FootstepManager::timerCallback(const ros::TimerEvent& event)
//{
//    // this is just in case some view crashes or a new one opens somewhere.
//    this->publishFootstepList();
//    this->publishFootstepParameterSetList();
//}

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

void FootstepManager::processFootstepPlanGoalUpdate(const flor_ocs_msgs::OCSFootstepPlanGoalUpdate::ConstPtr& plan_goal)
{
    ROS_ERROR(" Not handled yet!");

//    // Fill in goal here
//    vigir_footstep_planning_msgs::UpdateFeetGoal action_goal;
//    action_goal.feet = feet;
//    action_goal.update_mode.mode = vigir_footstep_planning_msgs::UpdateMode::UPDATE_MODE_3D;

//    // and send it to the server
//    if(update_feet_client_->isServerConnected())
//    {
//        update_feet_client_->sendGoal(action_goal,
//                                      boost::bind(&FootstepManager::doneUpdateFeet,     this, _1, _2),
//                                      boost::bind(&FootstepManager::activeUpdateFeet,   this),
//                                      boost::bind(&FootstepManager::feedbackUpdateFeet, this, _1));
//    }
//    else
//    {
//        ROS_WARN("UpdateFeet: Server not connected!");
//    }

}

void FootstepManager::processFootstepPlanGoal(const flor_ocs_msgs::OCSFootstepPlanGoal::ConstPtr& plan_goal)
{
    // and then the end feet poses
    if (calculateGoal(plan_goal->goal_pose))
    {
        ROS_WARN("Need to invoke planner after getting new goal");
    }
}

void FootstepManager::processFootstepParamSetSelected(const std_msgs::String::ConstPtr& msg)
{
    if(selected_footstep_parameter_set_ != msg->data || selected_footstep_parameter_set_ == "")
    {
        selected_footstep_parameter_set_ = msg->data;
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
        planner_config_ = new_planner_config;
        onboard_planner_configuration_pub_.publish(planner_config_);
    }
}

bool FootstepManager::calculateGoal(const geometry_msgs::PoseStamped& goal_pose)
{
    vigir_footstep_planning_msgs::GenerateFeetPoseService feet_pose_service;
    feet_pose_service.request.request.header = goal_pose.header;
    feet_pose_service.request.request.pose = goal_pose.pose;
    feet_pose_service.request.request.flags = vigir_footstep_planning_msgs::FeetPoseRequest::FLAG_CURRENT_Z; // this is for 3D interaction

    if (!generate_feet_pose_client.call(feet_pose_service.request, feet_pose_service.response))
    {
      ROS_ERROR("Can't call 'FeetPoseGenerator' service!");
      publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_GOAL,vigir_footstep_planning::toString(feet_pose_service.response.status));
      return false;
    }

    // Valid goal for the feet
    boost::recursive_mutex::scoped_lock lock(goal_mutex_);
    goal_ = feet_pose_service.response.feet; // goal in robot feet frame (ankle)
    goal_pose_ = goal_pose;
}

void FootstepManager::processFootstepPlanUpdate(const vigir_footstep_planning_msgs::StepPlan::ConstPtr& msg)
{
    ROS_ERROR("Not implemented yet!");

    // Call update footstep to evaluate risk and re-set robot specific parameters

    // If valid, then update current plan

}

void FootstepManager::processFootstepPlanRequest(const std_msgs::Int8::ConstPtr& plan_request)
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    if(current_step_plan_.steps.size() == 0 || start_step_index_ > current_step_plan_.steps.back().step_index)
        start_step_index_ = -1;

    if(start_step_index_ == -1)//if(plan_request->mode == flor_ocs_msgscurrent_step_plan_OCSFootstepPlanRequest::NEW_PLAN)
    {
        // request a completely new plan starting from the robot position
        requestStepPlanFromRobot();
    }
    else if(start_step_index_ == current_step_plan_.steps.back().step_index)
    {
        // get last step
        vigir_footstep_planning_msgs::Step step = getStepPlan().steps.back();
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
                ROS_ERROR("Invalid starting step index = %d with %d total steps in current plan", start_step_index_, current_step_plan_.size());
                publish
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

void FootstepManager::requestStepPlanFromRobot()
{
    // get start feet pose
    vigir_footstep_planning_msgs::GenerateFeetPoseService feet_pose_service;
    feet_pose_service.request.request.header.frame_id = "/world";
    feet_pose_service.request.request.header.stamp = ros::Time::now();
    feet_pose_service.request.request.flags = vigir_footstep_planning_msgs::FeetPoseRequest::FLAG_CURRENT;

    if (!generate_feet_pose_client.call(feet_pose_service.request, feet_pose_service.response))
    {
      ROS_ERROR("Can't call 'FeetPoseGenerator'!");
      return;
    }

    // check result
    if (vigir_footstep_planning::hasError(feet_pose_service.response.status))
    {
      ROS_ERROR("Error occured while requesting current feet pose:\n%s", vigir_footstep_planning::toString(feet_pose_service.response.status).c_str());
      publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_GOAL,vigir_footstep_planning::toString(feet_pose_service.response.status));
      return;
    }
    else if (vigir_footstep_planning::hasWarning(feet_pose_service.response.status))
    {
      ROS_ERROR("Warning occured while requesting current feet pose:\n%s", vigir_footstep_planning::toString(feet_pose_service.response.status).c_str());
      publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_GOAL,vigir_footstep_planning::toString(feet_pose_service.response.status));
      return;
    }

    // since feet is reported in robot frame (ankle), need to transform it to planner frame
    foot_pose_transformer_->transformToPlannerFrame(feet_pose_service.response.feet);

    sendStepPlanRequestGoal(feet_pose_service.response.feet, goal_);
}

void FootstepManager::requestStepPlanFromStep(vigir_footstep_planning_msgs::Step& step)
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

void FootstepManager::cleanMarkerArray(visualization_msgs::MarkerArray& old_array, visualization_msgs::MarkerArray& new_array)
{
    // update visualization message of the new step plan removing unused markers
    for(int i = 0; i < old_array.markers.size(); i++)
    {
        // add delete action to the marker if it doesn't exist in the new plan, ignore if it was already deleted
        if(old_array.markers[i].action != visualization_msgs::Marker::DELETE && new_array.markers.size() == i)
        {
            visualization_msgs::Marker marker;
            marker = old_array.markers[i];
            marker.action = visualization_msgs::Marker::DELETE;
            new_array.markers.push_back(marker);
        }
    }
}

// action callbacks for goal feet pose update request
void FootstepManager::activeUpdateFeet()
{
    ROS_INFO("UpdateFeet: Status changed to active.");
}

void FootstepManager::feedbackUpdateFeet(const vigir_footstep_planning_msgs::UpdateFeetFeedbackConstPtr& feedback)
{
    ROS_INFO("UpdateFeet: Feedback received.");
}

// send updated status to ocs
void FootstepManager::publishPlannerStatus(const flor_ocs_msgs::OCSFootstepStatus::_status_type& status, const std::string& status_msg )
{
    flor_ocs_msgs::OCSFootstepStatus planner_status;
    onboard_planner_status_pub_.status = status;
    onboard_planner_status_pub_.status_msg = status_msg;
    onboard_planner_status_pub_.publish(planner_status);
}
void FootstepManager::doneUpdateFeet(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateFeetResultConstPtr& result)
{
    ROS_INFO("UpdateFeet: Got action response.\n");

    if(vigir_footstep_planning::hasError(result->status) && result->status.error != vigir_footstep_planning_msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL)
    {
        ROS_ERROR("UpdateFeet: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_INVALID_GOAL, vigir_footstep_planning::toString(result->status));
    }
    else
    {
        // update the goal feet
        boost::recursive_mutex::scoped_lock lock(goal_mutex_);
        goal_ = result->feet;

    }
}

// action goal for StepPlanRequest
void FootstepManager::sendStepPlanRequestGoal(vigir_footstep_planning_msgs::Feet start, vigir_footstep_planning_msgs::Feet goal, const unsigned int start_step_index, const unsigned char start_foot)
{
    vigir_footstep_planning_msgs::StepPlanRequest request;

    //convert transform to ankle for planner, but only transform start pose if start is 0
    foot_pose_transformer_->transformToRobotFrame(start);
    foot_pose_transformer_->transformToRobotFrame(goal);

    request.header.frame_id = "/world";
    request.header.stamp = ros::Time::now();

    request.start = start;
    request.goal = goal;
    request.start_step_index = start_step_index;

    request.start_foot_selection = start_foot;

    // default planning mode is 2D, but will get that from the OCS
    if(planner_config_.use_3d_planning)
        request.planning_mode = vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_3D;
    else
        request.planning_mode = vigir_footstep_planning_msgs::StepPlanRequest::PLANNING_MODE_2D;

    // need to get the following from the OCS as well
    request.max_planning_time = planner_config_.max_time;
    request.max_number_steps = planner_config_.max_steps;
    request.max_path_length_ratio = planner_config_.path_length_ratio;

    // and then use the selected parameter set
    request.parameter_set_name.data = selected_footstep_parameter_set_;

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
    }

    // send updated status to ocs
    flor_ocs_msgs::OCSFootstepStatus planner_status;
    planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ACTIVE;
    planner_status.status_msg = "Sending StepPlanRequest action goal...";
    planner_status_pub_.publish(planner_status);
}

// action callbacks for step plan request
void FootstepManager::activeStepPlanRequest()
{
    ROS_INFO("StepPlanRequest: Status changed to active.");
}

void FootstepManager::feedbackStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequestFeedbackConstPtr& feedback)
{
    ROS_INFO("StepPlanRequest: Feedback received.");

    vigir_footstep_planning::vis::publishRecentlyVistedSteps(planner_plan_request_feedback_cloud_pub_, feedback->feedback.visited_steps, feedback->feedback.header);

    // send updated status to ocs
    flor_ocs_msgs::OCSFootstepStatus planner_status;
    planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ACTIVE;
    planner_status.status_msg = "Footstep planner working (percent complete).";
    planner_status_pub_.publish(planner_status);
}

void FootstepManager::doneStepPlanRequest(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::StepPlanRequestResultConstPtr& result)
{
    ROS_INFO("StepPlanRequest: Got action response.");

    if(vigir_footstep_planning::hasError(result->status))
    {
        ROS_ERROR("StepPlanRequest: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = result->status.error_msg;
        planner_status_pub_.publish(planner_status);
    }
    else
    {
        // send update to glance_hub "done"
        boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

        if(result->step_plan.header.stamp.nsec != last_ocs_step_plan_stamp_.nsec || result->step_plan.header.stamp.sec != last_ocs_step_plan_stamp_.sec)
        {
            vigir_footstep_planning_msgs::StepPlan plan = result->step_plan;
            processNewStepPlan(plan);

            last_ocs_step_plan_stamp_ = result->step_plan.header.stamp;
        }
        else
        {
            ROS_INFO("StepPlanRequest: Ignoring repeated plan (%d, %d).", result->step_plan.header.stamp.sec, result->step_plan.header.stamp.nsec);
        }

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_SUCCESS;
        planner_status.status_msg = "Footstep planner done.";
        planner_status_pub_.publish(planner_status);
    }
}

// action goal for EditStep
void FootstepManager::sendEditStepGoal(vigir_footstep_planning_msgs::StepPlan step_plan, vigir_footstep_planning_msgs::Step step)
{
//    //convert to ankle for planner - Onboard only works in ankle frame
//    foot_pose_transformer_->transformToRobotFrame(step_plan);
//    foot_pose_transformer_->transformToRobotFrame(step.foot);

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
        ROS_INFO("EditStep: Server not connected!");
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ERROR, vigir_footstep_planning::toString(result->status));
    }
}

// action callbacks
void FootstepManager::activeEditStep()
{
    ROS_INFO("EditStep: Status changed to active.");
}

void FootstepManager::feedbackEditStep(const vigir_footstep_planning_msgs::EditStepFeedbackConstPtr& feedback)
{
    ROS_INFO("EditStep: Feedback received.");
}

void FootstepManager::doneEditStep(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::EditStepResultConstPtr& result)
{
    ROS_INFO("EditStep: Got action response.");

    if (vigir_footstep_planning::hasError(result->status) && result->status.error != vigir_footstep_planning_msgs::ErrorStatus::ERR_INVALID_TERRAIN_MODEL)
    {
        ROS_ERROR("EditStep: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());
        publishPlannerStatus(flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED, vigir_footstep_planning::toString(result->status));
    }
    else
    {
        if(result->step_plans.size() == 0)
        {
            ROS_ERROR("EditStep: Received no step plan.");
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

// action goal for pplan
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
    ROS_INFO("UpdateStepPlan: Got action response.");

    if(vigir_footstep_planning::hasError(result->status))
    {
        ROS_ERROR("UpdateStepPlan: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = result->status.error_msg;
        planner_status_pub_.publish(planner_status);
    }
    else
    {
        boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

        // this is reserved for validation
    }
}

// action goal for executestep
void FootstepManager::sendExecuteStepPlanGoal()
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    // need to make sure we only have one step plan, and that plan has steps
    if(getStepPlanList().size() != 1 || !getStepPlan().steps.size())
    {
        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = "Can't execute empty plan or multiple plans (stitch first).";
        planner_status_pub_.publish(planner_status);

        return;
    }

    // also need to make sure this plan hasn't been sent before
    if(last_executed_step_plan_stamp_ == getStepPlan().header.stamp)
    {
        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = "Can't execute same plan twice.";
        planner_status_pub_.publish(planner_status);

        return;
    }

    // save the last plan executed timestamp
    last_executed_step_plan_stamp_ = getStepPlan().header.stamp;

    // Fill in goal here
    vigir_footstep_planning_msgs::ExecuteStepPlanGoal action_goal;
    action_goal.step_plan = getStepPlan();

    //convert transform to ankle for planner, might be redundant here?
    foot_pose_transformer_->transformToRobotFrame(action_goal.step_plan);

    // and send it to the server
    if(execute_step_plan_client_->isServerConnected())
    {
        execute_step_plan_client_->sendGoal(action_goal,
                                            boost::bind(&FootstepManager::doneExecuteStepPlan, this, _1, _2),
                                            boost::bind(&FootstepManager::activeExecuteStepPlan, this),
                                            boost::bind(&FootstepManager::feedbackExecuteStepPlan, this, _1));
    }
    else
    {
        ROS_INFO("ExecuteStepPlan: Server not connected!");
    }
}

// action callbacks
void FootstepManager::activeExecuteStepPlan()
{
    ROS_INFO("ExecuteStepPlan: Status changed to active.");

    // send updated status to ocs
    flor_ocs_msgs::OCSFootstepStatus planner_status;
    planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ACTIVE;
    planner_status.status_msg = "Execution of footstep plan active";
    planner_status_pub_.publish(planner_status);
}

void FootstepManager::feedbackExecuteStepPlan(const vigir_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback)
{
    ROS_INFO("ExecuteStepPlan: Feedback received.");

    // send updated status to ocs
    flor_ocs_msgs::OCSFootstepStatus planner_status;
    planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_ACTIVE;
    planner_status.status_msg = "Execution feedback received: Step "+boost::lexical_cast<std::string>(feedback->last_performed_step_index)+" -> Step "+boost::lexical_cast<std::string>(feedback->currently_executing_step_index);
    planner_status_pub_.publish(planner_status);
}

void FootstepManager::doneExecuteStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result)
{
    ROS_INFO("ExecuteStepPlan: Got action response.");

    if(!(result->status.status & vigir_footstep_planning_msgs::FootstepExecutionStatus::REACHED_GOAL))
    {
        ROS_ERROR("ExecuteStepPlan: Error occured!\nExecution error code: %d", result->status.status);

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = "Execution error code: "+boost::lexical_cast<std::string>(result->status.status);
        planner_status_pub_.publish(planner_status);
    }
    else
    {
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_SUCCESS;
        planner_status.status_msg = "Succesfully reached goal! Status: "+boost::lexical_cast<std::string>(result->status.status);
        planner_status_pub_.publish(planner_status);
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
        ROS_INFO("GetAllParameterSets: Server not connected!");
    }
}

void FootstepManager::activeGetAllParameterSets()
{
    ROS_INFO("GetAllParameterSets: Status changed to active.");
}

void FootstepManager::feedbackGetAllParameterSets(const vigir_footstep_planning_msgs::GetAllParameterSetsFeedbackConstPtr& feedback)
{
    ROS_INFO("GetAllParameterSets: Feedback received.");
}

void FootstepManager::doneGetAllParameterSets(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::GetAllParameterSetsResultConstPtr& result)
{
    ROS_INFO("GetAllParameterSets: Got action response. %s", result->status.error_msg.c_str());

    if(vigir_footstep_planning::hasError(result->status))
    {
        ROS_ERROR("GetAllParameterSets: Error occured!\n%s", vigir_footstep_planning::toString(result->status).c_str());

        // send updated status to ocs
        flor_ocs_msgs::OCSFootstepStatus planner_status;
        planner_status.status = flor_ocs_msgs::OCSFootstepStatus::FOOTSTEP_PLANNER_FAILED;
        planner_status.status_msg = result->status.error_msg;
        planner_status_pub_.publish(planner_status);
    }
    else
    {
        boost::recursive_mutex::scoped_lock lock(param_mutex_);

        footstep_parameter_set_list_.clear();
        for(int i = 0; i < result->param_sets.size(); i++)
            footstep_parameter_set_list_.push_back(result->param_sets[i]);

        this->publishFootstepParameterSetList();
    }
}

//void FootstepManager::processNewStepPlanGoal(vigir_footstep_planning_msgs::Feet& goal)
//{
//    // update the goal feet
//    boost::recursive_mutex::scoped_lock lock(goal_mutex_);

//    goal_ = goal;

//    // need to send visualization message
//    updateGoalVisMsgs();

//    plan_goal_array_pub_.publish(footstep_goal_array_);

//    // update goal pose - should I be doing this?
//    goal_pose_.pose.position.x = (goal_.left.pose.position.x+goal_.right.pose.position.x)/2.0;
//    goal_pose_.pose.position.y = (goal_.left.pose.position.y+goal_.right.pose.position.y)/2.0;
//    goal_pose_.pose.position.z = (goal_.left.pose.position.z+goal_.right.pose.position.z)/2.0;
//    Ogre::Quaternion q1(goal_.left.pose.orientation.w,goal_.left.pose.orientation.x,goal_.left.pose.orientation.y,goal_.left.pose.orientation.z);
//    Ogre::Quaternion q2(goal_.right.pose.orientation.w,goal_.right.pose.orientation.x,goal_.right.pose.orientation.y,goal_.right.pose.orientation.z);
//    Ogre::Quaternion qr = Ogre::Quaternion::Slerp(0.5,q1,q2);
//    goal_pose_.pose.orientation.w = qr.w;
//    goal_pose_.pose.orientation.x = qr.x;
//    goal_pose_.pose.orientation.y = qr.y;
//    goal_pose_.pose.orientation.z = qr.z;

//    // and update the interactive markers
//    publishGoalMarkerFeedback();
//}

void FootstepManager::processNewStepPlan(vigir_footstep_planning_msgs::StepPlan& step_plan)
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    ROS_INFO("processNewStepPlan: Processing new step plan (%d, %d).", step_plan.header.stamp.sec, step_plan.header.stamp.nsec);

    if(step_plan.steps.size() == 0)
    {
        ROS_ERROR("processNewStepPlan: Received empty step plan.");
        return;
    }

    //convert to sole for visualization
    foot_pose_transformer_->transformToPlannerFrame(step_plan);

    // we only change the current step lists if we receive a response
    if(step_plan.steps[0].step_index == 0)
        // This function will create a completely new plan, so we need to add a new empty list of plans to the stack
        addNewPlanList();
    else
        // This function will add a copy of the current step plan list to the stack, so we can change it
        addCopyPlanList();

    // add resulting plan to the top of the stack of plans, removing any extra steps
    extendPlanList(step_plan);

    publishFootsteps();

    //publishGoalMarkerClear();

    // update goal pose
    vigir_footstep_planning_msgs::Feet goal;
    goal.header = getStepPlan().steps[getStepPlan().steps.size()-1].header;
    goal.left = getStepPlan().steps[getStepPlan().steps.size()-1].foot.foot_index == vigir_footstep_planning_msgs::Foot::LEFT ?
                getStepPlan().steps[getStepPlan().steps.size()-1].foot :
                getStepPlan().steps[getStepPlan().steps.size()-2].foot;
    goal.right = getStepPlan().steps[getStepPlan().steps.size()-1].foot.foot_index == vigir_footstep_planning_msgs::Foot::RIGHT ?
                 getStepPlan().steps[getStepPlan().steps.size()-1].foot :
                 getStepPlan().steps[getStepPlan().steps.size()-2].foot;
    processNewStepPlanGoal(goal);
}

// onboard action callbacks
void FootstepManager::processOnboardStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequest::ConstPtr& step_plan_request)
{
    boost::recursive_mutex::scoped_lock lock(param_mutex_);

    // update parameter set
    selected_footstep_parameter_set_ = step_plan_request->parameter_set_name.data;
    std_msgs::String cmd;
    cmd.data = selected_footstep_parameter_set_;
    footstep_param_set_selected_pub_.publish(cmd);
}

void FootstepManager::processOnboardStepPlan(const vigir_footstep_planning_msgs::StepPlan::ConstPtr& step_plan)
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    if(step_plan->header.stamp.nsec != last_onboard_step_plan_stamp_.nsec || step_plan->header.stamp.sec != last_onboard_step_plan_stamp_.sec)
    {
        vigir_footstep_planning_msgs::StepPlan plan = *step_plan;
        processNewStepPlan(plan);

        last_onboard_step_plan_stamp_ = step_plan->header.stamp;
    }
    else
    {
        ROS_INFO("processOnboardStepPlan: Ignoring repeated plan (%d, %d).", step_plan->header.stamp.sec, step_plan->header.stamp.nsec);
    }
}

// onboard action callbacks
void FootstepManager::processOCSStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequest::ConstPtr& step_plan_request)
{
    boost::recursive_mutex::scoped_lock lock(param_mutex_);

    // update parameter set
    selected_footstep_parameter_set_ = step_plan_request->parameter_set_name.data;

    std_msgs::String cmd;
    cmd.data = selected_footstep_parameter_set_;
    footstep_param_set_selected_pub_.publish(cmd);
}

void FootstepManager::processOCSStepPlan(const vigir_footstep_planning_msgs::StepPlan::ConstPtr& step_plan)
{
    boost::recursive_mutex::scoped_lock lock(step_plan_mutex_);

    if(step_plan->header.stamp.nsec != last_ocs_step_plan_stamp_.nsec || step_plan->header.stamp.sec != last_ocs_step_plan_stamp_.sec)
    {
        vigir_footstep_planning_msgs::StepPlan plan = *step_plan;
        processNewStepPlan(plan);

        last_ocs_step_plan_stamp_ = step_plan->header.stamp;
    }
    else
    {
        ROS_INFO("processOnboardStepPlan: Ignoring repeated plan (%d, %d).", step_plan->header.stamp.sec, step_plan->header.stamp.nsec);
    }
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


}

PLUGINLIB_DECLARE_CLASS (vigir_onboard_footstep_manager, FootstepManager, onboard_footstep::FootstepManager, nodelet::Nodelet)
