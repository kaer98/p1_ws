#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <iostream>

using namespace std;

// Types
struct Point{
  double x;
  double y;
  double rotationDE;
};

// Classes
class corner{
    double x;
    double y;
};

class line{
    public:
        double slope;
        double startX;
        double startY;
        double endX;
        double endY;
};

// Function declaration //
//bool moveToGoal(int x, int y);
bool moveToLocation(double x, double y);
void greateGrid();
void updateMarker(ros::Publisher marker_pub, visualization_msgs::Marker marker, Point currentPoint, int id);
void updateLine(ros::Publisher marker_pub, Point FirstCurrentPoint, Point SecondCurrentPoint, int id);
void getPointerPose(const geometry_msgs::PointStamped point);
double slope(double x0, double x1, double y0, double y1);
double findSmallest(double x0, double x1, double x2, double x3);
double findBiggest(double x0, double x1, double x2, double x3);
double getArrayLenght(Point *array);
bool isInSquare(line topLine, line rightLine, line botLine, line leftLine, double X, double Y);
vector<double> createGrid(double x0, double x1, double x2, double x3, double y0, double y1, double y2, double y3,
 double d);
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// Placed points from RVIZ //
std::array<Point, 4> placedPoints;
bool hasAllPoints = false;
int placedPointsNumber = 0;

// Array of markers //
std::array<visualization_msgs::Marker, 3> markerPlacedPoints;



int main(int argc, char** argv){
  ros::init(argc, argv, "roundround");

  ros::NodeHandle n;
  //ros::Subscriber sub = n.subscribe("odom", 10, getRobotPos);
  ros::Subscriber subPointer = n.subscribe("clicked_point", 10, getPointerPose);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Rate r(1);

  // Random number gen //
  srand(time(NULL));

  // VIZ TEST //
  while (ros::ok())
  {
    // Type Marker in RVIZ
    visualization_msgs::Marker marker;
    
    // Now that the point is found and set to memory (currentPoint variable), we can use the function "moveToGoal", 
    // which takes in an X og Y position (can't use the Point type for some reason), which we get from the found point before. This
    // function returns a bool (is the goal reached or not)
    if(hasAllPoints){
      
      double d = 0.75; // Distance between plants ;)
    
      double x0 = placedPoints[0].x;  //0 Upper left corner
      double x1 = placedPoints[1].x;  //1 upper right corner
      double x2 = placedPoints[2].x;  //2 lower right corner
      double x3 = placedPoints[3].x;  //The last corner
      double y0 = placedPoints[0].y;
      double y1 = placedPoints[1].y;
      double y2 = placedPoints[2].y;
      double y3 = placedPoints[3].y;

      for (int i = 0; i < 4; i++)
      {
        cout << "Point " << " :: Value: " << placedPoints[i].x << endl;
      }
      
      vector<double>grid = createGrid(x0, x1, x2, x3, y0, y1, y2, y3, d);

      // Create array of points from grid with type Point //
      int gridPointsAmount = grid.size() / 2;

      //cout << "Points amount: " << gridPointsAmount << endl;

      cout << "gridsize:" << grid.size()<< "\n";
      Point seedlingsPoints[gridPointsAmount];
      //updateMarker(marker_pub, marker, seedlingsPoints[0],  10);
      int j = 0;
      int h = 0;

      cout << "Length of seed: " << sizeof(seedlingsPoints)/sizeof(seedlingsPoints[0]) << endl;
      // Create points from the generated grid //

      for (int i = 0; i < grid.size() / 2; i++){
        seedlingsPoints[i].x = grid[h];
        
        
        if( h < grid.size())
          h++;
          
        seedlingsPoints[i].y = grid[h];
        h++;
      }
      // Update the markers pose //
      // Update the placed markers
      for (int i = 0; i < 4; i++)
      {
        updateMarker(marker_pub, marker, placedPoints[i], i); 
      }
      
      for (int i = 0; i < grid.size()/2; i++)
      {
        updateMarker(marker_pub, marker, seedlingsPoints[i], i + 10);
      }
      
      // Now that the points are created, we can draw lines between them. //
      updateLine(marker_pub, placedPoints[0], placedPoints[1], 5);
      updateLine(marker_pub, placedPoints[1], placedPoints[2], 6);
      updateLine(marker_pub, placedPoints[2], placedPoints[3], 7);
      updateLine(marker_pub, placedPoints[3], placedPoints[0], 8);

  for (int i = 0; i < gridPointsAmount; i++){
    cout << "\n moving to location: " << seedlingsPoints[i].x << ", "<< seedlingsPoints[i].y << "\n";
    moveToLocation(seedlingsPoints[i].x, seedlingsPoints[i].y);
  }

    }


    ROS_INFO("UPDATE UPDATE");

    

    ros::spinOnce();
    r.sleep();
  }
}

void getPointerPose(const geometry_msgs::PointStamped point){

  double x = point.point.x;
  double y = point.point.y;

  // Get the four points from RVIZ //
  ROS_WARN("Please publish points in RVIZ.");

  if (!hasAllPoints && placedPointsNumber < 4){
    placedPoints[placedPointsNumber].x = x;
    placedPoints[placedPointsNumber].y = y;
    placedPointsNumber++;
    std::cout << "It is run and the number of points saved: " << placedPointsNumber << std::endl;
  }
  else{
    ROS_INFO("Got all the points successfully.");
    hasAllPoints = true;
  }

  std::cout << "Value of hasAllPoints: " << hasAllPoints << std::endl;
}

void updateMarker(ros::Publisher marker_pub, visualization_msgs::Marker marker, Point currentPoint, int id){

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::SPHERE;

  // Setup frame ID and timestamp.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = id;

  // Set the marker type.
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = currentPoint.x;
  marker.pose.position.y = currentPoint.y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker, (1,1,1) means 1m in each direction
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;

  // Setting the color
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      continue; // return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(marker);
}

void updateLine(ros::Publisher marker_pub, Point FirstCurrentPoint, Point SecondCurrentPoint, int id){

    visualization_msgs::Marker line_strip, line_list;
    line_strip.header.frame_id = line_list.header.frame_id = "map";
    line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    line_strip.ns = line_list.ns = "points_and_lines";
    line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    line_strip.id = id;
    line_list.id = id + 1;

    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.05;
    line_list.scale.x = 0.05;

    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Line list is red
    line_list.color.b = 1.0;
    line_list.color.a = 1.0;

    // Create the vertices for the points and lines

    geometry_msgs::Point p1;
    geometry_msgs::Point p2;

    // Start point for the line //
    p1.x = FirstCurrentPoint.x;
    p1.y = FirstCurrentPoint.y;
    p1.z = 0.1;

    // End point for the line //
    p2.x = SecondCurrentPoint.x;
    p2.y = SecondCurrentPoint.y;
    p2.z = 0.1;

    line_strip.points.push_back(p1);
    line_strip.points.push_back(p2);

    // The line list needs two points for each line
    line_list.points.push_back(p1);
    line_list.points.push_back(p2);

    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);
}




bool moveToLocation(double x, double y){
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  
  move_base_msgs::MoveBaseGoal goal;
  
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = 1.0;
  
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  ac.waitForResult();
 
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    std::cout <<"Hooray, the base moved 1 meter forward\n";
    return (true);
  }
  else{
    ROS_INFO("The base failed to move forward 1 meter for some reason\n");
    return (false);
  }}




// Calculating the slope of a line
double slope(double x0, double x1, double y0, double y1){
    double x01 = x1 - x0;
    double y01 = y1 - y0;
    double a;

    //a=(Y2-Y1)/(X2-x1)
    //y=ax+b

    if (y01 == 0 || x01 == 0){
        a=0;
        }
    else {
        a = (double)y01/(double)x01;
        }
    return(a);
}

double findSmallest(double x0, double x1, double x2, double x3){
    double xTemp = x0;
    if (x1 < xTemp){
    xTemp = x1;}
    if (x2 < xTemp){
    xTemp = x2;}
    if (x3 < xTemp){
    xTemp = x3;}
    return xTemp;
}

double findBiggest(double x0, double x1, double x2, double x3){
    double xTemp = x0;
    if (x1 > xTemp){
    xTemp = x1;}
    if (x2 > xTemp){
    xTemp = x2;}
    if (x3 > xTemp){
    xTemp = x3;}
    return xTemp;
}

//Gives a true if (X,Y) is inside square
bool isInSquare(line topLine, line rightLine, line botLine, line leftLine, double X, double Y){

    //Calculates max and min Y with regards to the chosen X value
    //Calculates max and min X with regards to the chosen Y value
    double botY = (X - botLine.endX)* botLine.slope + botLine.endY;
    double topY = (X - topLine.endX) * topLine.slope + topLine.endY;
    double rightX = (Y - rightLine.endY) * rightLine.slope + rightLine.endX;
    double leftX = (Y - leftLine.startY) * leftLine.slope + leftLine.startX;

    //Checks if the start of a line's Y value equals the Y value in the end of the line
    //If true, then we know the line is perpendicular to the X axis
    if (topLine.startY == topLine.endY){
    topY = topLine.startY;}
    if (botLine.startY == botLine.endY){
    botY = botLine.startY;}
    //Checks if the start of a line's X value equals the X value in the end of the line
    //If true, then we know the line is perpendicular to the Y axis
    if (rightLine.startX == rightLine.endX){
    rightX = rightLine.endX;}
    if (leftLine.startX == leftLine.endX){
    leftX = leftLine.endX;}

    // Checks if X and Y lies within its boundaries
    if (X > leftX && X < rightX && Y > botY && topY > Y){
        //cout<<leftX<<"\n" << rightX <<"\n" << botY <<"\n" << topY <<"\nSucces!!!\n";
        //cout<<"\nSucces!!!\n";
        return true;
        }
    else{
        //cout<<leftX<<"\n" << rightX <<"\n" << botY <<"\n" << topY;
        return false;
        }
}

vector<double> createGrid(double x0, double x1, double x2, double x3, double y0, double y1, double y2, double y3,
 double d){

    double smallestX = findSmallest(x0, x1, x2, x3);
    double biggestX = findBiggest(x0, x1, x2, x3);
    double smallestY = findSmallest(y0, y1, y2,y3);
    double biggestY = findBiggest(y0,y1,y2,y3);



    vector<double>grid;

    // Initializes lines and defines the values.
    line topLine; //Topline
    topLine.slope = slope(x0, x1, y0, y1);
    topLine.startX = x0;
    topLine.startY = y0;
    topLine.endX = x1;
    topLine.endY = y1;


    line rightLine; //RightLine
    rightLine.slope = slope(y1, y2, x1, x2);
    rightLine.startX = x1;
    rightLine.startY = y1;
    rightLine.endX = x2;
    rightLine.endY = y2;

    line botLine; //BotLine
    botLine.slope = slope(x2, x3, y2, y3);
    botLine.startX = x2;
    botLine.startY = y2;
    botLine.endX = x3;
    botLine.endY = y3;

    line leftLine; //LeftLine
    leftLine.slope = slope(y0, y3, x0, x3);
    leftLine.startX = x3;
    leftLine.startY = y3;
    leftLine.endX = x0;
    leftLine.endY = y0;


    double X;
    double Y;
    double dX = d; // For zigzag in for loop
    double zigzagTemp0 = smallestX; // For zigzag in X for loop
    double zigzagTemp1 = biggestX; // For zigzag in X for loop
    double zigzagTemp2 = X; // For zigzag in X for loop

    for (double Y = biggestY; Y > smallestY; Y = Y - d){

        //This will run on half of all occasions, because of the X>
        for (double X = zigzagTemp1; X > zigzagTemp0; X = X - dX){
            //Checks if XY is inside square
            if (isInSquare(topLine, rightLine, botLine, leftLine, X, Y) == 1){
                //cout <<"Plant"<< X << " :: " << Y <<"\n";
                grid.push_back(X);
                grid.push_back(Y);
            }

            }

        //This will run on half of all occasions, because of the X<
        for (double X = zigzagTemp1; X < zigzagTemp0; X = X - dX){
            if (isInSquare(topLine, rightLine, botLine, leftLine, X, Y) == 1){
                //cout <<"Plant"<< X << " :: " << Y <<"\n";
                grid.push_back(X);
                grid.push_back(Y);
            }
            }

        // Does a little switch of the X values, so the Plants will form a zigzag pattern...
        dX = dX * -1;
        if (smallestX == zigzagTemp1){
            zigzagTemp0 = smallestX;
            zigzagTemp1 = biggestX;
            }
        else{
            zigzagTemp1 = smallestX;
            zigzagTemp0 = biggestX;
            }

    }
    return grid;
}