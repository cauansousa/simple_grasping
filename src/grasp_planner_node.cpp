#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <simple_grasping/shape_grasp_planner.h>
#include <grasping_msgs/GraspPlanningAction.h>

class GraspPlannerNode
{
  typedef actionlib::SimpleActionServer<grasping_msgs::GraspPlanningAction> server_t;

public:
  GraspPlannerNode(ros::NodeHandle n)
  {
    // Planner
    planner_.reset(new simple_grasping::ShapeGraspPlanner(n));

    // Action for grasp planning
    server_.reset(new server_t(n, "plan",
                               boost::bind(&GraspPlannerNode::executeCallback, this, _1),
                               false));
    server_->start();
  }

private:
  void executeCallback(const grasping_msgs::GraspPlanningGoalConstPtr& goal)
  {
    grasping_msgs::GraspPlanningResult result;
    planner_->plan(goal->object, result.grasps);
    server_->setSucceeded(result);
  }

  boost::shared_ptr<simple_grasping::ShapeGraspPlanner> planner_;
  boost::shared_ptr<server_t> server_;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "grasp_planner");
  ros::NodeHandle nh("~");
  GraspPlannerNode planning(nh);
  ros::spin();
  return 0;
}
