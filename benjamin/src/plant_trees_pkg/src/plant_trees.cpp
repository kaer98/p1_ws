#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdlib.h>
#include <time.h>


// Types
struct Point{
  double x;
  double y;
  double rotationDE;
};

// Function declaration //
bool moveToGoal(int x, int y);
double distanceToNextPoint(Point firstPoint, Point secondPoint);
void greateGrid();
void getRobotPos(const nav_msgs::Odometry::ConstPtr& msg);
Point getNextPoint();

// Points of interests, grid array //
std::array<Point, 10> gridPoints;

bool goalReached = false;
int currentPointNumber = 0;
Point currentRobotPos;

int main(int argc, char** argv){
  ros::init(argc, argv, "roundround");

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("odom", 10, getRobotPos);

  // Random number gen //
  srand(time(NULL));

  // Setting the points //
  for(int i = 0; i < 10; i++){
    gridPoints[i] = getNextPoint();
  }

  do{
    // Check if the robot has reached the point by comparing the distance between the robot and goal point //
      
      //double distanceToPoint = distanceToNextPoint(currentRobotPos, gridPoints[currentPointNumber]);

    Point currentPoint = gridPoints[currentPointNumber];
    goalReached = moveToGoal(currentPoint.x, currentPoint.y);

    if(goalReached){
        currentPointNumber++;
        ROS_INFO("Setting next point...");
        ROS_INFO("New point x: %f  ::: Point y: %f", gridPoints[currentPointNumber].x, gridPoints[currentPointNumber].y);
    }
    ros::spin();
  }
  while (currentPointNumber != 10);
  //While we havent reached the last point //

  return 0;
}

void getRobotPos(const nav_msgs::Odometry::ConstPtr& msg){

  double x = msg->pose.pose.position.x;
  double y = msg->pose.pose.position.y;
  double rotation = 0;
  Point robotPos;

  robotPos.x = x;
  robotPos.y = y;
  robotPos.rotationDE = rotation;

  currentRobotPos = robotPos;

  ROS_INFO("x: %f, y: %f", x, y);
}


// KAN AF EN ELLER ANDEN MÆRKELIG GRUND IKKE BRUGE TYPEN POINT HER
bool moveToGoal(int x, int y){
  
  // This defines the client to send goal request to the move_base server through a SimpleActionClient //
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  // Wait for the action server to come up //
  while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // Set up the frame parameters, remember to change when not using the sim! //
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Move towards the goal //

   goal.target_pose.pose.position.x =  x;
   goal.target_pose.pose.position.y =  y;
   goal.target_pose.pose.position.z =  0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal location...");
  ac.sendGoal(goal);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    
    ROS_INFO("I have reached the goal!");
    ROS_INFO("Point x: %f  ::: Point y: %f", x, y);
    return true;

  }
  else{

    ROS_INFO("I have failed you, I shall now go die.");
    return false;

  }
    
}

Point getNextPoint(){
  
  Point ranPoint;
  ranPoint.x = rand() % 2;
  ranPoint.y = rand() % 2;

  return ranPoint;

}

double distanceToNextPoint (Point firstPoint, Point secondPoint){
  
  return sqrt(pow(secondPoint.x - firstPoint.x, 2) + pow(secondPoint.y - firstPoint.y, 2) * 1.0f);

}