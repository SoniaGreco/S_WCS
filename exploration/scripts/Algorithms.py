import math;


class State:
    def __init__(self,parent=None,cost=0,heu=0,x=0,y=0,depth=0):
        self.parent=parent;
        self.cost=cost;
        self.heu=heu;
        self.x=x;
        self.y=y;
        self.depth=depth;



class Problem:
    def __init__(self,sx=0,sy=0,gx=0,gy=0,matrix=[],width=0,height=0):
        self.matrix=[];
        for i in range(0,height):
            self.matrix.append(list(matrix[i*width:(i+1)*width]));
        self.width=width;
        self.height=height;
        self.start_x=sx;
        self.start_y=sy;
        self.goal_x=gx;
        self.goal_y=gy;
        self.foot_print=1;
    def set_params(self,sx,sy,gx,gy,matrix):
        self.matrix=[];
        for i in range(0,self.height):
            self.matrix.append(list(matrix[i*self.width:(i+1)*self.width]));
        self.start_x=sx;
        self.start_y=sy;
        self.goal_x=gx;
        self.goal_y=gy;

    def check_state(self,node1 , node2):
        if(node1.x==node2.x and node2.y==node1.y):
            return True;
        else:
            return True;

    def check_neighber(self,x,y):
        temp_list=[];
        for i in range(-self.foot_print,self.foot_print+1):
            temp_list.append(i);
        for i in temp_list:
            for j in temp_list:
                if(j==0 and i==0 ):continue;
                if(y+i>=self.height or x+j>=self.width):continue;
                if(self.matrix[y+i][x+j]>90):return True;
        return False;


    def expand(self,node):
        children=[];
        y=node.y;
        x=node.x;
        temp_list=[-1,0,1];
        for i in temp_list:
            for j in temp_list:
                if(j==0 and i==0 ):continue;
                if (y+i>=self.height or x+j>=self.width):continue;
                if(self.matrix[y+i][x+j]>90):continue;
                #if(self.check_neighber(x+j,y+i)==True):continue;
                my_node=State(parent=node,x=x+j,y=y+i,depth=node.depth+1);
                my_node.heu=self.heuristic(my_node);
                if(self.matrix[y+i][x+j]>=0 and self.matrix[y+i][x+j]<10):
                    my_node.cost=node.cost+1;
                else:
                    my_node.cost=node.cost+1000;
                children.append(my_node);
        return children;

    def Goal_test(self,node):
        if(node.x==self.goal_x and node.y==self.goal_y):
            return True;
        else:
            return False

    def Solution(self,node):
        mylist = [];
        n = node;
        while (n != None):
            mylist.insert(0, n);
            n = n.parent;
        #print ("found something");
        return node.depth;

    def initialize(self):
        mylist=[5,8,3,1,6,4,7,2,0];
        my_node=State(parent=None,cost=0,x=self.start_x,y=self.start_y,depth=0);
        my_node.heu=self.heuristic(my_node);
        return [my_node];

    def heuristic(self,node):
        heu=0;
        heu=math.sqrt((self.goal_x-node.x)**2+(self.goal_y - node.y)**2);
        return heu;




class Algorithmes:
    def __init__(self,problem=None):
        self.problem=problem;
        self.f_list=[];
        self.e_list=[];
        self.num_nodes=0;
        self.max_nodes=0;
        self.num_expansion=0;
    def Astar_graph(self):
        self.e_list = [];
        self.f_list = self.problem.initialize();
        while (len(self.f_list) > 0):
            node = self.f_list.pop(0);
            self.e_list.append(node);
            if (self.problem.Goal_test(node)):
                return self.problem.Solution(node);
            children = self.problem.expand(node);
            for i in children:
                self.insert_with_heuristic(i);
                if (self.problem.Goal_test(i)):
                    return self.problem.Solution(i);
        #print("no solution found");
        return None;

    def check_e(self,child):
        for i in self.e_list:
            if (self.problem.check_state(i,child)):
                return True;
        return False;

    def check_f(self,child):
        for i in self.f_list:
            if (self.problem.check_state(i, child)):
                return True;
        return False;

    def insert_with_cost(self,child):
        if (self.check_e(child)):
                return;
        k=0;
        for i in self.f_list:
            if (self.problem.check_state(i, child)):
                if(child.cost>=i.cost):
                  return ;
                self.f_list.pop(k);
                self.insert_with_cost(child);
                return;
            k+=1;
        k = 0;
        for i in self.f_list:
                if (child.cost <= i.cost):
                    self.f_list.insert(k,child);
                    return ;
                k += 1;
        self.f_list.insert(k, child);
        return ;

    def insert_with_heuristic(self, child):
        if (self.check_e(child)):
            return;
        k = 0;
        for i in self.f_list:
            if (self.problem.check_state(i, child)):
                if (child.heu >= i.heu):
                    return;
                self.f_list.pop(k);
                self.num_nodes -= 1;
                self.insert_with_heuristic(child);
                return;
            k += 1;
        k = 0;
        self.num_nodes += 1;
        for i in self.f_list:
            if (child.heu <= i.heu):
                self.f_list.insert(k, child);
                return;
            k += 1;
        self.f_list.insert(k, child);
        return;
