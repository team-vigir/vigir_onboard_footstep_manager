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
#include <ros/ros.h>
#include <tf/tf.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <boost/thread.hpp>

#include <vector>
#include <stack>
#include <string>

#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <vigir_footstep_planning_msgs/visualization.h>

#include <vigir_ocs_msgs/OCSFootstepUpdate.h>
#include <vigir_ocs_msgs/OCSFootstepPlanGoal.h>
#include <vigir_ocs_msgs/OCSFootstepPlanGoalUpdate.h>
#include <vigir_ocs_msgs/OCSFootstepPlanRequest.h>

#include <vigir_ocs_msgs/OCSFootstepPlanParameters.h>

#include <vigir_ocs_msgs/OCSFootstepPlanUpdate.h>
#include <vigir_ocs_msgs/OCSFootstepStatus.h>

#include <vigir_footstep_planning_msgs/footstep_planning_msgs.h>
#include <vigir_footstep_planning_msgs/parameter_set.h>

#include <vigir_foot_pose_transformer/foot_pose_transformer.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

namespace onboard_footstep
{
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::UpdateFeetAction>           UpdateFeetClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::StepPlanRequestAction>      StepPlanRequestClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::EditStepAction>             EditStepClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::StitchStepPlanAction>       StitchStepPlanClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::UpdateStepPlanAction>       UpdateStepPlanClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::ExecuteStepPlanAction>      ExecuteStepPlanClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::GetAllParameterSetsAction>  GetAllParameterSetsClient;
    typedef actionlib::SimpleActionClient<vigir_footstep_planning_msgs::GenerateFeetPoseAction>     GenerateFeetPoseClient;

    typedef actionlib::SimpleActionServer<vigir_footstep_planning_msgs::UpdateFeetAction>           UpdateFeetServer;
    typedef actionlib::SimpleActionServer<vigir_footstep_planning_msgs::StepPlanRequestAction>      StepPlanRequestServer;
    typedef actionlib::SimpleActionServer<vigir_footstep_planning_msgs::ExecuteStepPlanAction>      ExecuteStepPlanServer;
    typedef actionlib::SimpleActionServer<vigir_footstep_planning_msgs::GenerateFeetPoseAction>     GenerateFeetPoseServer;


    class FootstepManager : public nodelet::Nodelet
    {
    public:
        virtual void onInit();

        // triggers footstep planning via topic calls from OCS

        // Update single step
        void processFootstepPoseUpdate(const vigir_ocs_msgs::OCSFootstepUpdate::ConstPtr& msg); //// done

        // Update the goal feet poses from OCS (implicit request to replan with specified parameters)
        void processFootstepPlanGoalUpdate(const vigir_ocs_msgs::OCSFootstepPlanGoalUpdate::ConstPtr& plan_goal); // shell only

        // Set a new 3D goal (x,y,yaw) and calculate feet poses from terrain (implicit request to replan)
        void processFootstepPlanGoal(const vigir_ocs_msgs::OCSFootstepPlanGoal::ConstPtr& plan_goal); // need replan request

        // Set planning parameters (e.g. timeout)
        void processFootstepPlanParameters(const vigir_ocs_msgs::OCSFootstepPlanParameters::ConstPtr& msg); //// done

        // Select a default parameter set for planning
        void processFootstepParamSetSelected(const std_msgs::String::ConstPtr& msg); //// stores name, no validation

        // Update with new plan from OCS (feet poses only, need to update parameters
        void processFootstepPlanUpdate(const vigir_footstep_planning_msgs::StepPlan::ConstPtr& msg); // shell only

        // Request to re-plan with different
        void processFootstepPlanRequest(const vigir_ocs_msgs::OCSFootstepPlanRequest::ConstPtr& plan_request); // in progress

        // Execute via action call

        // Action Server interfaces for use by behaviors or OCS
        // flor_footstep_planning_msgs::ExecuteStepPlanAction
        // flor_footstep_planning_msgs::UpdateFeetAction
        // vigir_footstep_planning_msgs::StepPlanRequestAction

    private:

        // callbacks for action clients
        //generate feet pose
        void activeGenerateFeetPose();
        void feedbackGenerateFeetPose(const vigir_footstep_planning_msgs::GenerateFeetPoseFeedbackConstPtr& feedback);
        void doneGenerateFeetPose(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::GenerateFeetPoseResultConstPtr& result);

        //updatefeet
        void activeUpdateFeet();
        void feedbackUpdateFeet(const vigir_footstep_planning_msgs::UpdateFeetFeedbackConstPtr& feedback);
        void doneUpdateFeet(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateFeetResultConstPtr& result);

        //stepplanrequest
        void activeStepPlanRequest();
        void feedbackStepPlanRequest(const vigir_footstep_planning_msgs::StepPlanRequestFeedbackConstPtr& feedback);
        void doneStepPlanRequest(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::StepPlanRequestResultConstPtr& result);

        //editstep
        void activeEditStep();
        void feedbackEditStep(const vigir_footstep_planning_msgs::EditStepFeedbackConstPtr& feedback);
        void doneEditStep(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::EditStepResultConstPtr& result);

        //updatestepplan
        void activeUpdateStepPlan();
        void feedbackUpdateStepPlan(const vigir_footstep_planning_msgs::UpdateStepPlanFeedbackConstPtr& feedback);
        void doneUpdateStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::UpdateStepPlanResultConstPtr& result);

        //executestep
        void activeExecuteStepPlan();
        void feedbackExecuteStepPlan(const vigir_footstep_planning_msgs::ExecuteStepPlanFeedbackConstPtr& feedback);
        void doneExecuteStepPlan(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::ExecuteStepPlanResultConstPtr& result);

        //getallparametersets
        void activeGetAllParameterSets();
        void feedbackGetAllParameterSets(const vigir_footstep_planning_msgs::GetAllParameterSetsFeedbackConstPtr& feedback);
        void doneGetAllParameterSets(const actionlib::SimpleClientGoalState& state, const vigir_footstep_planning_msgs::GetAllParameterSetsResultConstPtr& result);


        // callbacks for action servers
        void stepPlanRequestGoalCB();
        void stepPlanRequestPreemptCB();
        void executeStepPlanGoalCB();
        void executeStepPlanPreemptCB();
        void updateFeetGoalCB();
        void updateFeetPreemptCB();
        void generateFeetPoseGoalCB();
        void generateFeetPosePreemptCB();

        // send action goals
        void sendStepPlanRequestGoal(vigir_footstep_planning_msgs::Feet start, vigir_footstep_planning_msgs::Feet goal, const unsigned int start_step_index = 0, const unsigned char start_foot = vigir_footstep_planning_msgs::StepPlanRequest::AUTO);
        void sendEditStepGoal(vigir_footstep_planning_msgs::StepPlan step_plan, vigir_footstep_planning_msgs::Step step);
        void sendUpdateStepPlanGoal(vigir_footstep_planning_msgs::StepPlan step_plan);
        void sendExecuteStepPlanGoal();
        void sendGetAllParameterSetsGoal();

        // plan requests
        bool calculateGoal(const geometry_msgs::PoseStamped& goal_pose); // calculate initial goal pose
        void requestStepPlanFromRobot();
        void requestStepPlanFromStep(const vigir_footstep_planning_msgs::Step &step);

        // helper function for finding step based on step_index
        bool findStep(const unsigned int& step_index, vigir_footstep_planning_msgs::Step& step, unsigned int& step_plan_index );
        void publishPlannerStatus(const vigir_ocs_msgs::OCSFootstepStatus::_status_type& status, const std::string& status_msg = "" );
        bool processNewStepPlan(const vigir_footstep_planning_msgs::StepPlan& step_plan);
        bool getStartFeet(vigir_footstep_planning_msgs::Feet& feet);


        ros::Timer timer;

        ros::Publisher  onboard_footstep_plan_pub_;
        ros::Publisher  onboard_planner_status_pub_;
        ros::Publisher  onboard_planner_parameter_set_pub_;
        ros::Publisher  onboard_planner_configuration_pub_;
        ros::Publisher  onboard_footstep_plan_request_pub_;

        ros::Subscriber ocs_footstep_update_sub_;
        ros::Subscriber ocs_footstep_plan_goal_update_sub_;
        ros::Subscriber ocs_footstep_plan_goal_sub_;
        ros::Subscriber ocs_footstep_plan_parameters_sub_;
        ros::Subscriber ocs_footstep_set_parameter_set_sub_;
        ros::Subscriber ocs_footstep_plan_sub_;
        ros::Subscriber ocs_footstep_plan_request_sub_;

        // feet pose generator client
        ros::ServiceClient          generate_feet_pose_service_client_;

        // Action clients
        GenerateFeetPoseClient*     generate_feet_pose_action_client_;
        UpdateFeetClient*           update_feet_client_;
        StepPlanRequestClient*      step_plan_request_client_;
        EditStepClient*             edit_step_client_;
        UpdateStepPlanClient*       update_step_plan_client_;
        ExecuteStepPlanClient*      execute_step_plan_client_;
        GetAllParameterSetsClient*  get_all_parameter_sets_client_;

        // Action servers
        StepPlanRequestServer*      step_plan_request_server_;
        ExecuteStepPlanServer*      execute_step_plan_server_;
        UpdateFeetServer*           update_feet_server_;
        GenerateFeetPoseServer*     generate_feet_pose_server_;

        // messages
        vigir_footstep_planning_msgs::StepPlanRequestActionGoal      step_plan_request_goal_;
        vigir_footstep_planning_msgs::StepPlanRequestActionResult    step_plan_request_result_;

        vigir_footstep_planning_msgs::ExecuteStepPlanActionGoal      execute_step_plan_goal_;
        vigir_footstep_planning_msgs::ExecuteStepPlanActionResult    execute_step_plan_result_;

        vigir_footstep_planning_msgs::StepPlan                       current_step_plan_;

        // Footstep parameter sets
        std::vector<vigir_footstep_planning_msgs::ParameterSet> footstep_parameter_set_list_;
        std::string                                             selected_footstep_parameter_set_;

        // Parameters
        geometry_msgs::Vector3 foot_size;

        geometry_msgs::Vector3 upper_body_size;
        geometry_msgs::Vector3 upper_body_origin_shift;

        // used to calculate feet poses for start/end of footstep plan
        geometry_msgs::PoseStamped          active_goal_pose_;
        geometry_msgs::PoseStamped          goal_pose_;
        vigir_footstep_planning_msgs::Feet  goal_;

        // last step plan request received, saved and used mostly for message parameters
        vigir_ocs_msgs::OCSFootstepPlanParameters planner_config_;

        // specifies which footstep will be used as starting point for the planner, -1 to start a new one
        int start_step_index_;



        // local instance of foot pose transformer
        vigir_footstep_planning::FootPoseTransformer::Ptr foot_pose_transformer_;

        boost::recursive_mutex step_plan_mutex_;
        boost::recursive_mutex param_mutex_;
        boost::recursive_mutex goal_mutex_;

        ros::Time last_ocs_step_plan_stamp_;
        ros::Time last_onboard_step_plan_stamp_;

        ros::Time last_executed_step_plan_stamp_;

        //ros::ServiceClient edit_step_service_client_;
    };
}
