#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>
#include <sstream>
#include <vector>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
// Init of some stuff, like cmd_vel_message for communication with turtlesim
// Also some assignment of these variables and messages 
geometry_msgs::Twist cmd_vel_message;
bool start = false;
float velocity_gain = 0.1;
float angular_gain = 1.0;
turtlesim::Pose turtle1_pose;

//Function running when recievin data from turtlesim node
void cbPose1(const turtlesim::Pose::ConstPtr& message){
  turtle1_pose =*message;
}
//Defining a struct
struct point{
    double X; 
    double Y;
    double angle;
};
//Function for scanning text
point scanText(const std_msgs::String::ConstPtr& text){
  //Init of a temporary point
  point temp;
  temp.X = 0;
  int length = text->data.length();
  //Const char assigned to be ","
  const char komma = 44;
  std::stringstream tempString;

  
  for (int i = 0; i < length; i++){
    //Checks if there isnt a "," in the [i] location of string
    if ((char)text->data[i] != komma ){
      //If there is not, then append string[i] to tempstring 
      tempString  << text->data[i];
      
    } else{
      //Else assign temp.X to the tempstring
    tempString >> temp.X;
    //Clear tempstring for temp.Y
    tempString.clear();
    
    }
  }
  //As only one comma can be found the next number will surely be the 'Y' coordinate... Assign Temp.y to the value of the "new" tempstring
  tempString >> temp.Y;

  //return point Temp
    return temp;
}

//Init of point nextPlant
point nextPlant;
//Function for when recieving info from other node
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  //init of a stringstream
 std::stringstream ss;
 //Assignment of same stringstream
    ss << msg->data.c_str();
    //Assign start to true, so it will know later on a connection has been established with the node...
  start = true;
  //Define point nextPlant to the numbers recieved from the other node.
   nextPlant=scanText(msg);

}
//Calculating a difference in angles from the turtlebots angle and the angle, between the nextplant point and the X axis, defined by the turtlebot as origon.
double angleDif(double X1, double Y1, double X2, double Y2, double angle){
  //Trigonomitry stuff. Calculating lengts of triangle
  double b = X2 - X1;
  double a = Y2 - Y1;
  double c = (a*a) + (b*b);
  c = sqrt(c);
  //Calculating the angle between the next planting spot and X axis, with turtlebot as Origon
  double bb = b*b;
  double cc = c*c;
  double aa = a*a;
  double A = (bb+cc-aa)/(2*b*c);
  A = acos(A);
  if (Y1 > Y2){

  //When Turtlebot is above the planting point, it will calculate a positive angle however a negative is needed. Therefor it will just take the negative value
    A = -A;
  }
 //Calculating the difference in angles..
  A = A - angle;
  return A;
}
//Main function...
int main(int argc, char **argv)
{
  //Init of ros...
  ros::init(argc, argv, "talker");

  //Init of nodehandle...
  ros::NodeHandle n;

  

  //Subscribes and publishes to diffent topics. Subscribes to other node, and position of turtlebot... Publishes to cmd_vel_pub which will make the turtlesim move.
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatters", 1000);
  ros::Subscriber pose1_subscriber = n.subscribe("/turtle1/pose", 100, &cbPose1);
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1);
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  //Sets looprate to 5. 5 Hz is now the basic frequency of the program...
  ros::Rate loop_rate(5);
  //Sets counter to 0... This was used for debugging
  int count = 0;
  //Init of point TB, turtebot...
  point TB;
  //Init of point C
  point C;
 // Init and assignment of some needed values.
  geometry_msgs::Twist cmd_vel_message;
  cmd_vel_message.linear.x = 0;
  cmd_vel_message.angular.z = 1;
  bool goalReached = false;
      nextPlant.X = 10;
    nextPlant.Y = 10;
  //Init and assignment of sleeptime to 0
    int sleepTime = 0;

    //Main loop, will run "forever" or until stopped
  while (ros::ok())
  {
    //Init of strings and stringstream.
    std_msgs::String msg;

    std::stringstream ss;
    //Asigns point TB to turtlebots position and angle
    TB.X = turtle1_pose.x;
    TB.Y = turtle1_pose.y;
    TB.angle = turtle1_pose.theta;
    //Publishes movement data
    cmd_vel_pub.publish(cmd_vel_message);
    
    //Checks if connection has been made to node
    if (start == true){
      //Calculates how much the turtlebot will need to turn to point in the direction of the next treespot  
    double turn = angleDif(TB.X,TB.Y,nextPlant.X,nextPlant.Y,TB.angle);
    //Assigns this value to cmd_vel_message
    cmd_vel_message.angular.z = turn;
    //Checks if it is not on the spot. With a tolerance of 10%
    if (TB.X <= nextPlant.X*0.9 || TB.X >= nextPlant.X*1.1 || TB.Y <= nextPlant.Y*0.9 || TB.Y >= nextPlant.Y*1.1){
      //If it is not on the spot it will move .4 forward and assign ss to "Fail"
      cmd_vel_message.linear.x = 0.4;
      ss << "Fail";
      sleepTime = 0;
    }
    else{
      //If it is on the spot, it will not move, but it will however assign ss to "Succes"
      cmd_vel_message.linear.x = 0;
      ss << "Succes";
      //It will also tell the user a tree has been planted and where it has been planted
      std::cout <<"Location X,Y:"<< TB.X<<","<<TB.Y<<"  Planting a tree...........\n";
      //Sets sleeptime to 1
      sleepTime = 1;
    }}
    //Assigns msg to be equal to ss.str()
    msg.data = ss.str();
    chatter_pub.publish(msg);
    ros::spinOnce();
   
    
    
    loop_rate.sleep();
    sleep(sleepTime);
    ++count;
  }


  return 0;
}