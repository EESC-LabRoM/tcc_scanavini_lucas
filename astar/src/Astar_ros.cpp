#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>

#include "Astar_ros.h"

#include <pluginlib/class_list_macros.h>
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(Astar_planner::AstarPlannerROS, nav_core::BaseGlobalPlanner)


int value;
int mapSize;
bool* OGM;
static const float INFINIT_COST = INT_MAX; //!< cost of non connected nodes
float infinity = std::numeric_limits< float >::infinity();
float tBreak;  // coefficient for breaking ties
ofstream MyExcelFile ("A_result.xlsx", ios::trunc);

int clock_gettime(clockid_t clk_id, struct timespect *tp);

timespec diff(timespec start, timespec end)
{
  timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

inline vector <int> findFreeNeighborCell (int CellID);

namespace Astar_planner
{
/*******************************************************************************/
/*********************************************************************************/
//Default Constructor
AstarPlannerROS::AstarPlannerROS()
{

}
/*******************************************************************************/
/*********************************************************************************/
AstarPlannerROS::AstarPlannerROS(ros::NodeHandle &nh)
{
  ROSNodeHandle = nh;

}
/*******************************************************************************/
/*********************************************************************************/
AstarPlannerROS::AstarPlannerROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  initialize(name, costmap_ros);
}
/*******************************************************************************/
/*********************************************************************************/
void AstarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{

if (!initialized_)
  {
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  ros::NodeHandle private_nh("~/" + name);

  originX = costmap_->getOriginX();
  originY = costmap_->getOriginY();

	width = costmap_->getSizeInCellsX();
	height = costmap_->getSizeInCellsY();
	resolution = costmap_->getResolution();
	mapSize = width*height;
	tBreak = 1+1/(mapSize); 
	value =0;
 
	OGM = new bool [mapSize]; 
    for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
    {
      for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
      {
        unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));
        //cout<<cost;
        if (cost == 0){
          OGM[iy*width+ix]=true;
          }
        else{
          OGM[iy*width+ix]=false;
        }
      }
    }
  
   
	MyExcelFile << "StartID\tStartX\tStartY\tGoalID\tGoalX\tGoalY\tPlannertime(ms)\tpathLength\tnumberOfCells\t" << endl;

    ROS_INFO("Astar planner initialized successfully");
    initialized_ = true;
  }
  else
    ROS_WARN("This planner has already been initialized... doing nothing");
}
/*******************************************************************************/
/*********************************************************************************/
bool AstarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan)
{

  if (!initialized_)
  {
    ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
    return false;
  }

  ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
            goal.pose.position.x, goal.pose.position.y);

  plan.clear();

  if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
  {
    ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
              costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
    return false;
  }

  tf::Stamped < tf::Pose > goal_tf;
  tf::Stamped < tf::Pose > start_tf;

  poseStampedMsgToTF(goal, goal_tf);
  poseStampedMsgToTF(start, start_tf);

  // convert the start and goal positions

  float startX = start.pose.position.x;
  float startY = start.pose.position.y;

  float goalX = goal.pose.position.x;
  float goalY = goal.pose.position.y;

  getCorrdinate(startX, startY);
  getCorrdinate(goalX, goalY);

  int startCell;
  int goalCell;
  
  if (isCellInsideMap(startX, startY) && isCellInsideMap(goalX, goalY))
  {
    startCell = convertToCellIndex(startX, startY);

    goalCell = convertToCellIndex(goalX, goalY);
    
    cout <<"StartCell: "<< startCell<<endl;
    std::cout << "Goalcell:" << goalCell << std::endl;
MyExcelFile << startCell <<"\t"<< start.pose.position.x <<"\t"<< start.pose.position.y <<"\t"<< goalCell <<"\t"<< goal.pose.position.x <<"\t"<< goal.pose.position.y;

  }
  else
  {
    ROS_WARN("the start or goal is out of the map");
    return false;
  }
  // call global planner

  if (isStartAndGoalCellsValid(startCell, goalCell)){

  vector<int> bestPath;
	bestPath.clear();
  bestPath = AstarPlanner(startCell, goalCell);
//if the global planner find a path
    if ( bestPath.size()>0)
    {

// convert the path

      for (int i = 0; i < bestPath.size(); i++)
      {

        float x = 0.0;
        float y = 0.0;

        int index = bestPath[i];

        convertToCoordinate(index, x, y);

        geometry_msgs::PoseStamped pose = goal;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        plan.push_back(pose);
      }


	float path_length = 0.0;
	
	std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();
	
	geometry_msgs::PoseStamped last_pose;
	last_pose = *it;
	it++;
	for (; it!=plan.end(); ++it) {
	   path_length += hypot(  (*it).pose.position.x - last_pose.pose.position.x, 
		                 (*it).pose.position.y - last_pose.pose.position.y );
	   last_pose = *it;
	}
	cout <<"The global path length: "<< path_length<< " meters"<<endl;
	MyExcelFile << "\t" <<path_length <<"\t"<< plan.size() <<endl;
      //publish the plan

      return true;

    }

    else
    {
      ROS_WARN("The planner failed to find a path, choose other goal position");
      return false;
    }

  }

  else
  {
    ROS_WARN("Not valid start or goal");
    return false;
  }

}
/*******************************************************************************/
/*********************************************************************************/
void AstarPlannerROS::getCorrdinate(float& x, float& y)
{

  x = x - originX;
  y = y - originY;

}
/*******************************************************************************/
/*********************************************************************************/
int AstarPlannerROS::convertToCellIndex(float x, float y)
{

  int cellIndex;

  float newX = x / resolution;
  float newY = y / resolution;

  cellIndex = getCellIndex(newY, newX);

  return cellIndex;
}
/*******************************************************************************/
/*********************************************************************************/
void AstarPlannerROS::convertToCoordinate(int index, float& x, float& y)
{

  x = getCellColID(index) * resolution;

  y = getCellRowID(index) * resolution;

  x = x + originX;
  y = y + originY;

}
/*******************************************************************************/
/*********************************************************************************/
bool AstarPlannerROS::isCellInsideMap(float x, float y)
{
  bool valid = true;

  if (x > (width * resolution) || y > (height * resolution))
    valid = false;

  return valid;
}
/*******************************************************************************/
/*********************************************************************************/
void AstarPlannerROS::mapToWorld(double mx, double my, double& wx, double& wy){
   costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    wx = costmap->getOriginX() + mx * resolution;
    wy = costmap->getOriginY() + my * resolution;
}
/*******************************************************************************/
/*********************************************************************************/
vector<int> AstarPlannerROS::AstarPlanner(int startCell, int goalCell){

vector<int> bestPath;
float g_score[mapSize];

for (uint i=0; i<mapSize; i++)
	g_score[i]=infinity;

   timespec time1, time2;
  /* take current time here */
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

   bestPath=findPath(startCell, goalCell,  g_score);
   std::cout << "out of while while\n" << std::endl;
   clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);


   cout<<"time to generate best global path by Relaxed A* = " << (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 << " microseconds" << endl;
   
   MyExcelFile <<"\t"<< (diff(time1,time2).tv_sec)*1e3 + (diff(time1,time2).tv_nsec)*1e-6 ;

  return bestPath;

}
/*******************************************************************************/
/*********************************************************************************/
vector<int> AstarPlannerROS::findPath(int startCell, int goalCell, float g_score[])
{
	value++;
  float aux=infinity;
  int PositionOnOpenList=0;
	vector<int> bestPath;
	vector<int> emptyPath;
  vector<int> openList;
 
  float f_cost[mapSize];
  for (uint x=0; x<mapSize; x++)
  	f_cost[x]=infinity;
 
  int Parent[mapSize];
  for (int y=0; y<mapSize; y++)
  	Parent[y]=-1;   
    
  bool closedList[mapSize];
  for (int i=0; i<mapSize; i++)
	closedList[i]=0;

	int currentCell;
	//calculate g_score and f_score of the start position
	g_score[startCell]=0;
  f_cost[startCell]=g_score[startCell]+calculateHCost(startCell,goalCell);
	//add the start cell to the open list
  openList.insert(openList.begin(),startCell);
	currentCell=startCell;
  //set parent for start cell
  Parent[startCell]=startCell;
	//while the open list is not empty continuie the search or g_score(goalCell) is equal to infinity
	while (!openList.empty() && g_score[goalCell]==infinity) 
	{
		//choose the cell that has the lowest cost fCost 
	for(int z=0;z<openList.size();z++){
      int IDCellonOpenlist = openList[z];
      if(f_cost[IDCellonOpenlist]<aux && !closedList[IDCellonOpenlist]){
      aux=f_cost[IDCellonOpenlist];
      currentCell=IDCellonOpenlist;
      PositionOnOpenList=z;
	  }
    }
    aux=infinity;
    //adciona na closedList
   	closedList[openList[PositionOnOpenList]]=1;
    //remove the currentCell from the openList
		openList.erase(openList.begin()+PositionOnOpenList);	
		//search the neighbors of the current Cell
		vector <int> neighborCells; 
		neighborCells=findFreeNeighborCell(currentCell);
		for(uint i=0; i<neighborCells.size(); i++) //for each neighbor of current cell
		{
			// if the g_score of the neighbor is equal to INF: unvisited cell
			if(g_score[neighborCells[i]]>=g_score[currentCell] && !closedList[neighborCells[i]])
			{
				g_score[neighborCells[i]]=g_score[currentCell]+getMoveCost(currentCell,neighborCells[i]);
				addNeighborCellToOpenList(openList, neighborCells[i], goalCell, g_score, f_cost);
        //set neighborCell Parent
        Parent[neighborCells[i]]=currentCell;
			}//end if
		}//end for
  }//end while
  
	if(g_score[goalCell]!=infinity)  // if g_score(goalcell)==INF : construct path 
	{
		bestPath=constructPath(startCell, goalCell, Parent);
		return   bestPath; 
	}
	else
	{
		cout << "Failure to find a path !" << endl;
		return emptyPath;
	}
}
/*******************************************************************************/
/*********************************************************************************/
vector<int> AstarPlannerROS::constructPath(int startCell, int goalCell, int Parent[])
{
	vector<int> bestPath;
	vector<int> path;
  float aux=infinity;
  int aux2;
  int posMinfScore;
  int ParentCell;
  
  bool alreadyVisited [mapSize];
  for (int i=0; i<mapSize; i++)
	alreadyVisited[i]=0;
  
	path.insert(path.begin()+bestPath.size(), goalCell);
	int currentCell=goalCell;

	while(Parent[currentCell]!=startCell)
	{ 
    //set ParentCell as the parent of currentCell
    ParentCell=Parent[currentCell];
		path.insert(path.begin()+path.size(), ParentCell);
    //next loop we will find the parent of the parent cell of the currentcell 
    currentCell=ParentCell;
	}
	for(uint i=0; i<path.size(); i++)
		bestPath.insert(bestPath.begin()+bestPath.size(), path[path.size()-(i+1)]);

	return bestPath;
}

/*******************************************************************************/
/*********************************************************************************/
void AstarPlannerROS::addNeighborCellToOpenList(std::vector<int>& openList, int neighborCell, int goalCell, float g_score[],float f_cost[])
{
	f_cost[neighborCell]=g_score[neighborCell]+calculateHCost(neighborCell,goalCell);
  openList.push_back(neighborCell);
}
/*******************************************************************************
*********************************************************************************/
vector <int> AstarPlannerROS::findFreeNeighborCell (int CellID){
 
  int rowID=getCellRowID(CellID);
  int colID=getCellColID(CellID);
  int neighborIndex;
  vector <int>  freeNeighborCells;

  for (int i=-1;i<=1;i++){
    for (int j=-1; j<=1;j++){
      //check whether the index is valid
     if ((rowID+i>=0)&&(rowID+i<height)&&(colID+j>=0)&&(colID+j<width)&& (!(i==0 && j==0))){
	neighborIndex = getCellIndex(rowID+i,colID+j);
        if(isFree(neighborIndex))
	    freeNeighborCells.push_back(neighborIndex);
	}
}}
    return  freeNeighborCells;
 
}
/*******************************************************************************/
/*********************************************************************************/
bool AstarPlannerROS::isStartAndGoalCellsValid(int startCell,int goalCell)
{ 
 bool isvalid=true;
 bool isFreeStartCell=isFree(startCell);
 bool isFreeGoalCell=isFree(goalCell);
    if (startCell==goalCell)
    {
    //cout << "The Start and the Goal cells are the same..." << endl; 
    isvalid = false;
    }
   else
   {
      if (!isFreeStartCell && !isFreeGoalCell)
      {
	cout << "The start and the goal cells are obstacle positions..." << endl;
        isvalid = false;
      }
      else
      {
	if (!isFreeStartCell)
	{
	 cout << "The start is an obstacle..." << endl;
	  isvalid = false;
	}
	else
	{
	    if(!isFreeGoalCell)
	    {
	      cout << "The goal cell is an obstacle..." << endl;
	      isvalid = false;
	    }
	    else
	    {
	      if (findFreeNeighborCell(goalCell).size()==0)
	      {
		cout << "The goal cell is encountred by obstacles... "<< endl;
		isvalid = false;
	      }
	      else
	      {
		if(findFreeNeighborCell(startCell).size()==0)
		{
		  cout << "The start cell is encountred by obstacles... "<< endl;
		  isvalid = false;
		}
	      }
	    }
	}
      }
  }
 return isvalid;
}
/*******************************************************************************/
/*********************************************************************************/
 float  AstarPlannerROS::getMoveCost(int i1, int j1, int i2, int j2){
   float moveCost=INFINIT_COST;//start cost with maximum value. Change it to real cost of cells are connected
   //if cell2(i2,j2) exists in the diagonal of cell1(i1,j1)
   if((j2==j1+1 && i2==i1+1)||(i2==i1-1 && j2==j1+1) ||(i2==i1-1 && j2==j1-1)||(j2==j1-1 && i2==i1+1)){
     //moveCost = DIAGONAL_MOVE_COST;
     moveCost = 1.4;
   }
    //if cell 2(i2,j2) exists in the horizontal or vertical line with cell1(i1,j1)
   else{
     if ((j2==j1 && i2==i1-1)||(i2==i1 && j2==j1-1)||(i2==i1+1 && j2==j1) ||(i1==i2 && j2==j1+1)){
       //moveCost = MOVE_COST;
       moveCost = 1;
     }
   }
   return moveCost;
 } 
/*******************************************************************************/
/*********************************************************************************/
  float  AstarPlannerROS::getMoveCost(int CellID1, int CellID2){
   int i1=0,i2=0,j1=0,j2=0;
    
   i1=getCellRowID(CellID1);
   j1=getCellColID(CellID1);
   i2=getCellRowID(CellID2);
   j2=getCellColID(CellID2);
    
    return getMoveCost(i1, j1, i2, j2);
 } 
/*******************************************************************************/
/*********************************************************************************/
//verify if the cell(i,j) is free
 bool  AstarPlannerROS::isFree(int i, int j){
   int CellID = getCellIndex(i, j);
 return OGM[CellID];
 } 
/*******************************************************************************/
/*********************************************************************************/
//verify if the cell(i,j) is free
 bool  AstarPlannerROS::isFree(int CellID){
 return OGM[CellID];
 } 
}
;
