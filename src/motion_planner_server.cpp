#include <ros/ros.h>

#include <task_motion_planner/motion_service.h>
#include <task_motion_planner/motion_plan_parameters.h>
#include <task_motion_planner/motion_plan.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

bool get_motion_plan(task_motion_planner::motion_service::Request &req, task_motion_planner::motion_service::Response &res) {
    ROS_INFO("Setting current state");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  
    planning_scene_interface.addCollisionObjects(req.parameters.world.collision_objects);

    moveit_msgs::RobotState curr_state = req.parameters.start;

    for (int i = 0; i < req.parameters.group_names.size(); i++) {
        ROS_INFO("Move group %s", req.parameters.group_names[i].c_str());
        moveit::planning_interface::MoveGroup group(req.parameters.group_names[i]);
    
        group.setPlanningTime(5.0);
        group.setStartState(curr_state);
        group.setJointValueTarget(req.parameters.goals[i]);
    
        moveit::planning_interface::MoveGroup::Plan plan;
        bool success = group.plan(plan);
    
        if (success) {
            ROS_INFO("Motion plan succeeded");
            res.plan.trajectory.trajectory_start = plan.start_state_;
            res.plan.trajectory.trajectory.push_back(plan.trajectory_);
        
            // Determine new state after execution of plan
            sensor_msgs::JointState joint_state;
            sensor_msgs::MultiDOFJointState multi_dof_joint_state;
            
            // Extract last joint trajectory in the robot trajectory
            trajectory_msgs::JointTrajectory last_joint_step = res.plan.trajectory.trajectory.back().joint_trajectory;
            trajectory_msgs::MultiDOFJointTrajectory last_multi_dof_joint_step = res.plan.trajectory.trajectory.back().multi_dof_joint_trajectory;
            
            // Extract last position of each joint in the joint trajectories
            joint_state.name = last_joint_step.joint_names;
            for (int j = 0; j < joint_state.name.size(); j++) {
                joint_state.position.push_back(last_joint_step.points[j].positions.back());
                joint_state.velocity.push_back(last_joint_step.points[j].velocities.back());
                joint_state.effort.push_back(last_joint_step.points[j].effort.back());
            }
            
            multi_dof_joint_state.joint_names = last_multi_dof_joint_step.joint_names;
            for (int j = 0; j < multi_dof_joint_state.joint_names.size(); j++) {
                multi_dof_joint_state.transforms.push_back(last_multi_dof_joint_step.points[j].transforms.back());
            }
        
            ROS_INFO("Setting new current state");
        
            // Update current state for next planned action
            curr_state.joint_state = joint_state;
            curr_state.multi_dof_joint_state = multi_dof_joint_state;
        } 
        else {
            ROS_INFO("Motion plan failed");
            moveit_msgs::DisplayTrajectory empty_trajectory_msg;
            res.plan.trajectory = empty_trajectory_msg;
            res.plan.success = false;
            return 0;
        }
    }

    ROS_INFO("Done");

    return 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "motion_planner_server");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    ros::ServiceServer service = nh.advertiseService("motion_server_service", get_motion_plan);
    
    ROS_INFO("Ready to serve motion plans");
    
    /*moveit::planning_interface::MoveGroup group("left_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
    ROS_INFO("Set goal");
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = 10.0;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.0;
    target_pose1.orientation.w = 1.0;
    group.setJointValueTarget(target_pose1);
    
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);
    
    ROS_INFO("%i", success);*/
    
    ros::waitForShutdown();
    
    return 0;
}

