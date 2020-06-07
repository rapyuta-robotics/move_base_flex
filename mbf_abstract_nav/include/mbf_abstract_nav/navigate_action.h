/*
 *  Copyright 2018, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  move_base_action.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */
#ifndef MBF_ABSTRACT_NAV__MOVE_BASE_ACTION_H_
#define MBF_ABSTRACT_NAV__MOVE_BASE_ACTION_H_

#include <mutex>

#include <actionlib/server/action_server.h>
#include <actionlib/client/simple_action_client.h>

#include <mbf_msgs/GetPathAction.h>
#include <mbf_msgs/ExePathAction.h>
#include <mbf_msgs/RecoveryAction.h>
#include <forklift_interfaces/NavigateAction.h>
#include <forklift_interfaces/Checkpoint.h>
#include <aifl_msg/SpinTurnAction.h>

#include "mbf_abstract_nav/MoveBaseFlexConfig.h"
#include "mbf_abstract_nav/robot_information.h"


namespace mbf_abstract_nav
{

class NavigateAction
{
 public:

  //! Action clients for the Navigate action
  typedef actionlib::SimpleActionClient<mbf_msgs::ExePathAction> ActionClientExePath;
  typedef actionlib::SimpleActionClient<aifl_msg::SpinTurnAction> ActionClientSpinTurn;


  typedef actionlib::ActionServer<forklift_interfaces::NavigateAction>::GoalHandle GoalHandle;

  NavigateAction(const std::string &name, const RobotInformation &robot_info);

  ~NavigateAction();

  void start(GoalHandle &goal_handle);

  void cancel();

  void reconfigure(
      mbf_abstract_nav::MoveBaseFlexConfig &config, uint32_t level);

 protected:

  void actionExePathFeedback(const mbf_msgs::ExePathFeedbackConstPtr &feedback);

  int8_t isSmoothTurnPossible(const forklift_interfaces::Checkpoint& previous, 
    const forklift_interfaces::Checkpoint& current, const forklift_interfaces::Checkpoint& next);

  bool getSplitPath(
      const forklift_interfaces::NavigatePath &plan,
      std::vector<forklift_interfaces::NavigatePath> &result);

  void actionExePathActive();

  void actionSpinTurnActive();

  void startNavigate();

  void runNavigate();

  void actionExePathDone(
      const actionlib::SimpleClientGoalState &state,
      const mbf_msgs::ExePathResultConstPtr &result);

  void actionGetPathReplanningDone(
      const actionlib::SimpleClientGoalState &state,
      const mbf_msgs::GetPathResultConstPtr &result);

  void actionSpinTurnFeedback(const aifl_msg::SpinTurnFeedbackConstPtr &feedback);

  void actionSpinTurnDone(
      const actionlib::SimpleClientGoalState &state,
      const aifl_msg::SpinTurnResultConstPtr &result);
  
  double getYaw(geometry_msgs::Quaternion orientation);
  


  mbf_msgs::ExePathGoal exe_path_goal_;
  forklift_interfaces::NavigatePath get_path_goal_;
  aifl_msg::SpinTurnGoal spin_turn_goal_;


  geometry_msgs::PoseStamped last_oscillation_pose_;
  ros::Time last_oscillation_reset_;
  ros::Time spin_turn_goal_timer_;

  ros::Duration oscillation_timeout_;
  ros::Duration spin_turn_timeout_;

  std::vector<forklift_interfaces::NavigatePath> path_segments_;

  double oscillation_distance_;

  GoalHandle goal_handle_;

  std::string name_;

  RobotInformation robot_info_;

  geometry_msgs::PoseStamped robot_pose_;

  ros::NodeHandle private_nh_;

  //! Action client used by the navigate action
  ActionClientExePath action_client_exe_path_;

  //! Action client used by the navigate action
  ActionClientSpinTurn action_client_spin_turn_;

  bool replanning_;
  ros::Rate replanning_rate_;
  boost::mutex replanning_mtx_;


  forklift_interfaces::NavigateFeedback navigate_feedback_;


  enum NavigateActionState
  {
    SPIN_ACTIVE,
    EXE_PATH_ACTIVE,
    IDLE,
    SPLIT_PATH,
    NAVIGATE,
    EXE_PATH,
    SPIN_TURN,
    OSCILLATING,
    SUCCEEDED,
    CANCELED,
    FAILED
  };

  NavigateActionState action_state_;
};

} /* mbf_abstract_nav */

#endif //MBF_ABSTRACT_NAV__MOVE_BASE_ACTION_H_
