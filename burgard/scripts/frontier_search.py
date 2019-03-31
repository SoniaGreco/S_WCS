from Queue import Queue;
from costmap_tools import *;
import math;
from nav_msgs.msg import Path , OccupancyGrid, Odometry;
from geometry_msgs.msg import Point , PoseStamped
from std_msgs.msg import Bool;

class Frontier:
    def __init__(self):
        self.size=0;
        self.min_distance=0.0;
        self.travel_point=Point();
        self.points=[];

class FrontierSearch:
     # * @brief Constructor for search task
     # * @param costmap Reference to costmap data to search. @type OccupancyGrid
     # * @param min_frontier_size The minimum size to accept a frontier @type int
     # * @param travel_point The requested travel point (closest|middle|centroid) @type string
     # */
    def __init__(self,costmap,min_frontier_size,travel_point):
        self.costmap_=costmap;
        self.map_=[];
        self.size_x_ =0;
        self.size_y_=0 ;
        self.min_frontier_size_=min_frontier_size;
        self.travel_point_=travel_point;
        self.map_=[];
        self.robot_pose=None;

    # /**
    #  * @brief Runs search implementation, outward from the start position
    #  * @param position Initial position to search from @type geometry_msgs::Point
    #  * @return List of frontiers, if any
    #  */
    def searchFrom(self,position):
        self.robot_pose=position;
        mx = (int)((position.x - self.costmap_.info.origin.position.x) / self.costmap_.info.resolution);
        my = (int)((position.y - self.costmap_.info.origin.position.y) / self.costmap_.info.resolution);
        test_listofpoints=[];
        frontier_list=[];
        self.map_=self.costmap_.data;
        self.size_x_ = self.costmap_.info.width;
        self.size_y_ = self.costmap_.info.height;
        #initialize flag arrays to keep track of visited and frontier cells
        frontier_flag=[False]*(self.size_x_ * self.size_y_);
        visited_flag=[False]*(self.size_x_ * self.size_y_);
        #initialize breadth first search
        bfs=Queue();
        #find closest clear cell to start search
        clear=0;
        pos=my*self.size_x_ + mx; #index in costmap list
        nc=nearestCell( pos, 0, self.costmap_);
        if(nc[0]==True):
            clear=nc[1];
            bfs.put(clear);
        else:
            bfs.put(pos);
            ROS_WARN("Could not find nearby clear cell to start search");

        visited_flag[bfs.queue[0]] = True;

        while(not bfs.empty()):
                idx =bfs.get();
                #iterate over 4-connected neighbourhood
                for nbr in nhood4(idx, self.costmap_):
                    #add to queue all free, unvisited cells, use descending search in case initialized on non-free cell
                    if(self.map_[nbr] <= self.map_[idx] and (not visited_flag[nbr])):
                               visited_flag[nbr] = True;
                               bfs.put(nbr);
                               #check if cell is new frontier cell (unvisited, NO_INFORMATION, free neighbour)
                    elif(self.isNewFrontierCell(nbr, frontier_flag)):
                               frontier_flag[nbr] = True;
                               test_new_frontier = self.buildNewFrontier(nbr, pos, frontier_flag);
                               new_frontier=test_new_frontier[0];
                               if(new_frontier.size > self.min_frontier_size_):
                                   frontier_list.append(new_frontier);
                                   test_listofpoints.append(test_new_frontier[1]);

        return [frontier_list,test_listofpoints];

     #  /**
     # * @brief Starting from an initial cell, build a frontier from valid adjacent cells
     # * @param initial_cell Index of cell to start frontier building @type int
     # * @param reference Reference index to calculate position from @type int
     # * @param frontier_flag Flag vector indicating which cells are already marked as frontiers @type list of boolean
     # * @return a frontier object @type Frontier
     # */
    def buildNewFrontier(self, initial_cell,  reference,  frontier_flag):
        #initialize frontier structure
        test_listofpoints=[]
        output=Frontier();
        centroid=Point();
        middle=Point();
        output.size = 1;
        output.min_distance = 999999;
        #record initial contact point for frontier
        ix=0;
        iy=0;
        #self.costmap_.indexToCells(initial_cell,ix,iy);
        iy = int(initial_cell / self.size_x_);
        ix = initial_cell - (iy * self.size_x_);

        #self.costmap_.mapToWorld(ix,iy,output.travel_point.x,output.travel_point.y);
        output.travel_point.x = self.costmap_.info.origin.position.x + (ix + 0.5) * self.costmap_.info.resolution;
        output.travel_point.y = self.costmap_.info.origin.position.y + (iy + 0.5) * self.costmap_.info.resolution;


        #push initial gridcell onto queue
        bfs=Queue();
        bfs.put(initial_cell);

        #cache reference position in world coords

        #self.costmap_.indexToCells(reference,rx,ry);
        ry = int(reference / self.size_x_);
        rx = reference - (ry * self.size_x_);

        #self.costmap_.mapToWorld(rx,ry,reference_x,reference_y);
        reference_x = self.costmap_.info.origin.position.x + (rx + 0.5) * self.costmap_.info.resolution;
        reference_y = self.costmap_.info.origin.position.y + (ry + 0.5) * self.costmap_.info.resolution;


        while(not bfs.empty()):
            idx = bfs.get();

            #try adding cells in 8-connected neighborhood to frontier
            for nbr in nhood8(idx, self.costmap_):
                #check if neighbour is a potential frontier cell
                if(self.isNewFrontierCell(nbr,frontier_flag)):

                    #mark cell as frontier
                    frontier_flag[nbr] = True;

                    #self.costmap_.indexToCells(nbr,mx,my);
                    my = int(nbr / self.size_x_);
                    mx = nbr - (my * self.size_x_);

                    #self.costmap_.mapToWorld(mx,my,wx,wy);
                    wx = self.costmap_.info.origin.position.x + (mx + 0.5) * self.costmap_.info.resolution;
                    wy = self.costmap_.info.origin.position.y + (my + 0.5) * self.costmap_.info.resolution;
                    test_listofpoints.append(Point(wx,wy,0.0));
                    output.points.append(Point(wx,wy,0.0));
                    #update frontier size
                    output.size+=1;

                    #update centroid of frontier
                    centroid.x += wx;
                    centroid.y += wy;

                    #determine frontier's distance from robot, going by closest gridcell to robot
                    distance = math.sqrt(((float(self.robot_pose.x)-float(wx))**2.0) + ((float(self.robot_pose.y)-float(wy))**2.0));
                    if(distance < output.min_distance):
                        output.min_distance = distance;
                        middle.x = wx;
                        middle.y = wy;
                    #add to queue for breadth first search
                    bfs.put(nbr);

        #average out frontier centroid
        centroid.x /= output.size;
        centroid.y /= output.size;

        if(self.travel_point_ == "closest"):
            # point already set
            pass;
        elif(self.travel_point_ == "middle"):
            output.travel_point = middle;
        elif(self.travel_point_ == "balanced"):
            output.travel_point = Point((middle.x+centroid.x)/2,(middle.y+centroid.y)/2,0.0);
        elif(self.travel_point_ == "centroid"):
            output.travel_point = centroid;
        else:
            ROS_ERROR("Invalid 'frontier_travel_point' parameter, falling back to 'closest'");
            #point already set


        return [output,test_listofpoints];

     #  /**
     # * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate for a new frontier.
     # * @param idx Index of candidate cell @type int
     # * @param frontier_flag Flag vector indicating which cells are already marked as frontiers @type list of boolean
     # * @return
     # */
    def isNewFrontierCell(self, idx, frontier_flag):
            #check that cell is unknown and not already marked as frontier
            if(self.map_[idx] != -1 or frontier_flag[idx]==True):
                return False;
            #frontier cells should have at least one cell in 4-connected neighbourhood that is free
            for  nbr in nhood4(idx, self.costmap_):
                if(self.map_[nbr] == 0):
                    return True;
            return False;
