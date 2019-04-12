from Queue import Queue;
from nav_msgs.msg import Path , OccupancyGrid, Odometry;
from geometry_msgs.msg import Point , PoseStamped
from std_msgs.msg import Bool;
import math;

# /**
#  * @brief Determine 4-connected neighbourhood of an input cell, checking for map edges
#  * @param idx input cell index @type idx
#  * @param costmap Reference to map data @type OccupancyGrid
#  * @return neighbour cell indexes @type list of int
#  */
def nhood4( idx,costmap):
    #get 4-connected neighbourhood indexes, check for edge of map
    out=[];

    size_x_ = costmap.info.width;
    size_y_ = costmap.info.height;

    if (idx > size_x_ * size_y_ -1):
        ROS_WARN("Evaluating nhood for offmap point");
        return out;

    if(idx % size_x_ > 0):
        out.append(idx - 1);

    if(idx % size_x_ < size_x_ - 1):
        out.append(idx + 1);

    if(idx >= size_x_):
        out.append(idx - size_x_);

    if(idx < size_x_*(size_y_-1)):
        out.append(idx + size_x_);

    return out;


#
# /**
#  * @brief Determine 8-connected neighbourhood of an input cell, checking for map edges
#  * @param idx input cell index @type int
#  * @param costmap Reference to map data @type OccupancyGrid
#  * @return neighbour cell indexes @type list of int
#  */

def nhood8( idx,costmap):
    # get 8-connected neighbourhood indexes, check for edge of map
    out = nhood4(idx, costmap);

    size_x_ = costmap.info.width;
    size_y_ = costmap.info.height;

    if (idx > size_x_ * size_y_ -1):
        return out;


    if(idx % size_x_ > 0 and idx >= size_x_):
        out.append(idx - 1 - size_x_);

    if(idx % size_x_ > 0 and idx < size_x_*(size_y_-1)):
        out.append(idx - 1 + size_x_);

    if(idx % size_x_ < size_x_ - 1 and idx >= size_x_):
        out.append(idx + 1 - size_x_);

    if(idx % size_x_ < size_x_ - 1 and idx < size_x_*(size_y_-1)):
        out.append(idx + 1 + size_x_);

    return out;



# /**
#  * @brief Find nearest cell of a specified value
#  * @param start Index initial cell to search from @type int
#  * @param val Specified value to search for @type int
#  * @param costmap Reference to map data @type OccupancyGrid
#  * @return [True,Index of located cell] if a cell with the requested value was found
#  */
def nearestCell(start,val,  costmap):

    map_ = costmap.data;
    size_x_ = costmap.info.width;
    size_y_ = costmap.info.height;

    if (start >= size_x_ * size_y_):
        return [False];


    # initialize breadth first search
    bfs=Queue();
    visited_flag=[False]*(size_x_ * size_y_);

    #push initial cell
    bfs.put(start);
    visited_flag[start] = True;

    #search for neighbouring cell matching value
    while(not bfs.empty()):
        idx = bfs.get();
        #return if cell of correct value is found
        if(map_[idx] == val):
            return [True,idx];

        #iterate over all adjacent unvisited cells
        for nbr in  nhood8(idx, costmap):
            if(not visited_flag[nbr]):
                bfs.put(nbr);
                visited_flag[nbr] = True;

    return [False];
