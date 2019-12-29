#include "ros/ros.h"
#include "std_msgs/String.h"
#include <vector>
#include <iostream>
using namespace std;
//Initialises and assignment public variables
bool plantedTree = false;
// Structs
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
// self explanatory
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
// self explanatory
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
        return true;
        }
    else{
        return false;
        }
}
//Function for returning a grid
vector<double> createGrid(double x0, double x1, double x2, double x3, double y0, double y1, double y2, double y3,
 double d){

    double smallestX = findSmallest(x0, x1, x2, x3);
    double biggestX = findBiggest(x0, x1, x2, x3);
    double smallestY = findSmallest(y0, y1, y2,y3);
    double biggestY = findBiggest(y0,y1,y2,y3);



    vector<double>grid;

    // Initializes lines and assigns the values.
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
             
                grid.push_back(X);
                grid.push_back(Y);
            }

            }

        //This will run on half of all occasions, because of the X<
        for (double X = zigzagTemp1; X < zigzagTemp0; X = X - dX){
            if (isInSquare(topLine, rightLine, botLine, leftLine, X, Y) == 1){
                
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
//returns a bool depending on the recieved text from the other node
bool scanText(const std_msgs::String::ConstPtr& text){

    //Assignment of const char to value of capital S
  const char CapS = 83;
    //Checks if the message starts with capital S
    if ((char)text->data[0] == CapS ){
        //Capital S means Succes, and it will let the user know a tree has been planted... Also returns true
        std::cout <<  "Planted a tree!!!\n";
        return true;
    } 
    else{
        //Returns false if no tree has been planted
        return false;

    
      }
    }
    //Function that will run when receiving info from node. 
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  //Scans message and checks whether or not a tree has been planted
  plantedTree = scanText(msg);
}
//Main
int main(int argc, char **argv)
{
//initialises ROS
  ros::init(argc, argv, "listener");

    //creates a nodehandle for later user.
  ros::NodeHandle n;

    //Subscribes to topic "chatters"
  ros::Subscriber sub = n.subscribe("chatters", 1000, chatterCallback);
  //Publishes to topic "chatter"... Not the best names for topics, but it works... Does not follow naming scheme...
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

 
  
    //Assignment of the frequency of the loop. This gives us 2 Hz
  ros::Rate loop_rate(2);

  //Init of positions of corners and some other stuff
double x0;
double x1;
double x2;
double x3;
double y0;
double y1;
double y2;
double y3;
double d;

  int count = 0;
    int sleepTime=0;

    //A lot of prints and userinputs that will define the area that the robot will plant in...
    std::cout <<"Time to plant some trees!!! Now it is up to you the user\n Define the quadrilateral the robot will plant inside...";
    std::cout <<"Pick the X coordinates for the first corner. This is the upper left corner of the quadrilateral... This number must be between 2 and 10!!!\n";
    std::cin >> x0;
    std::cout <<"Pick the Y coordinates for the a corner. This is the upper left corner of the quadrilateral... This number must be between 2 and 10!!!\n";
    std::cin >> y0;
    std::cout <<"Pick the X coordinates for the a corner. This is the upper right corner of the quadrilateral... This number must be between 2 and 10!!! but higher than " << x0<<".\n";
    std::cin >> x1;
    std::cout <<"Pick the Y coordinates for the a corner. This is the upper right corner of the quadrilateral... This number must be between 2 and 10!!!but higher or equal to " << y0<<".\n";
    std::cin >> y1;
    std::cout <<"Pick the X coordinates for the a corner. This is the lower right corner of the quadrilateral... This number must be between 2 and 10!!!\n";
    std::cin >> x2;
    std::cout <<"Pick the Y coordinates for the a corner. This is the lower right corner of the quadrilateral... This number must be between 2 and 10!!!but lower than " << y1<<".\n";
    std::cin >> y2;
    std::cout <<"Pick the X coordinates for the a corner. This is the lower left corner of the quadrilateral... This number must be between 2 and 10!!!but lower or equal to " << x2<<".\n";
    std::cin >> x3;
    std::cout <<"Pick the Y coordinates for the a corner. This is the lower left corner of the quadrilateral... This number must be between 2 and 10!!!but lower than " << y0<<".\n";
    std::cin >> y3;
    std::cout <<"\nPick the distance between each tree. Recommended is 1.\n";
    std::cin >> d;
    std::cout<<"The turtleSim will now start following a route and plant a \"couple\" of trees along the route\n";
    
    //The loop which will continue running
  while (ros::ok())
  {
      //init of a string
    std_msgs::String msg;
    //Init of string stream to be published
   std::stringstream ss;
   //Sets standard sleeptime to 0 seconds
   sleepTime=0;
    if (plantedTree == true){
      //Because of the grid vectors 1D structure (XYXYXYXY) we need to add 2 to the counter for each run.
      count = count + 2;
      //Sleeps for 5 seconds... It takes time to plant a tree, so for some kind of realism
      sleepTime=5;
      
    }
    //Creates a grid the robot will follow when planting trees
    vector <double>grid = createGrid(x0,x1,x2,x3,y0,y1,y2,y3,d);
    //Assignment of the string to be published to be equal to the next planting goal in the shaoe of "X,Y"
    ss << grid[count] << ","<< grid[count+1];
    msg.data = ss.str();


    //Publishing
    chatter_pub.publish(msg);
    
    ros::spinOnce();

    loop_rate.sleep();
    //Sleeps the desired extra time... For when a tree has been planted...
    sleep(sleepTime);
    
  }

  return 0;
}