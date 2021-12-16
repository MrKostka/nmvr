import numpy as np
import math

from numpy.lib.function_base import append
import rclpy
from rclpy.node import Node

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from std_msgs.msg import String

class U():
    def __init__(self, stuff = None):
        self.vertices = {} if stuff is None else stuff

    def in_the(self, key):
        return key in self.vertices.keys()

    def topKey(self):
        return 0 if len(self.vertices) == 0 else self.vertices[next(iter(self.vertices))]

    def pop(self):
        foo = iter(self.vertices)
        first = next(foo)
        self.vertices.pop(first)
        return first

    def insert(self, s_vertex, k_key):
        self.vertices[s_vertex] = k_key
        self._reorder()

    def update(self, s_vertex, k_key):
        self.vertices[s_vertex] = k_key
        self._reorder()

    def remove(self, s_vertex):
        del self.vertices[s_vertex]

    def _reorder(self):
        self.vertices = {k: v for k, v in sorted(self.vertices.items(), key=lambda item: (item[1][0],item[1][1]))}

class DStarLite(Node):
    def __init__(self,row_count=60, col_count=40, actual_borders = None, tile_size = 20):
        super().__init__("d_star_lite")
        self.s_start = (0, 0)
        self.s_goal = None
        self.s_last = None

        self.tile_size = tile_size

        self.col_count = col_count
        self.row_count = row_count

        self.U = None
        self.S = [(x, y) for x in range(0, row_count) for y in range(0,col_count)]
        self.B = [] if actual_borders is None else actual_borders
        self.old_B = []

        self.k_m = None

        self.rhss = {}
        self.gs = {}

        self.planned_path = []

        self.new_partial_goal_pose_pub = self.create_publisher(Pose,'new_partial_goal_pose',10)

        self.path_pub = self.create_publisher(String,'path_pub',10)

        self.pose_subs = self.create_subscription(Pose,'pose',self.pose_callback,10)
        self.goal_subs = self.create_subscription(Pose,'goal',self.goal_callback,10)

        self.border_subs = self.create_subscription(Pose,'border',self.border_callback,10)


        self.json_borders_subs = self.create_subscription(String,'json_borders',self.json_borders_callback,10)
        

        timer_period = 1.0
        self.timer = self.create_timer(timer_period,self.main_stuff)

        self.robot_is_on_position = False
        self.is_init_done = False

        self.need_to_be_loaded = True

    def publish_path_pose(self):
        listik = str(self.planned_path)
        msg = String()
        msg.data = listik
        self.path_pub.publish(msg)

    def publish_new_goal_pose(self, x, y):
        goal_pose = Pose()
        goal_pose.x = float(x)
        goal_pose.y = float(y)
        goal_pose.theta =  0.0
        self.new_partial_goal_pose_pub.publish(goal_pose) 

    def border_callback(self,data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""

        border = (data.x, data.y)

        if border in self.B:
            ind = self.B.index(border)
            del self.B[ind]   
        else:
            self.B.append(border)

    def json_borders_callback(self,data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""

        if self.need_to_be_loaded:
            datas = data.data

            datas = list(eval(datas))

            for foo in datas:
                self.B.append(foo)

            self.old_B = self.B.copy()
            self.need_to_be_loaded = False
            

    def pose_callback(self,data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""

        x = math.floor(data.x / self.tile_size)
        y = math.floor(data.y / self.tile_size)

        

        if self.s_start == (x, y):
            self.robot_is_on_position = True        

    def goal_callback(self,data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""

        self.s_goal = (data.x, data.y)

        self.s_last = self.s_start
        self.initialize()
        self.compute_shortest_path()  

        
        if(not self.s_start == self.s_goal):

            if self.gs[self.s_start] == np.inf:
                print("Not possible to find a path.")  
                return              

            self.s_start = self.find_arg_min(self.s_start)

            self.publish_new_goal_pose(self.s_start[0], self.s_start[1])
            self.is_init_done = True

    def cost(self, s_vertex, s_next_vertex):        
        return np.sqrt( np.power( np.abs(s_vertex[0] - s_next_vertex[0]), 2 ) + np.power( np.abs(s_vertex[1] - s_next_vertex[1]), 2) ) if s_next_vertex not in self.B else np.inf
        # return np.abs(s_vertex[0] - s_next_vertex[0]) + np.abs(s_vertex[1] - s_next_vertex[1])

    def h(self, s_vertex, s_star):
        """Returns heuristics - the manhattan distance between two args"""
        return max(abs(s_vertex[0] - s_star[0]), abs(s_vertex[1] - s_star[1]))

    def neighbors(self, s_vertex):
        
        modifiers = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        return [(s_vertex[0] - modifier[0], s_vertex[1] - modifier[1]) 
        for modifier in modifiers 
        if ((s_vertex[0] - modifier[0]) < self.row_count) and ((s_vertex[0] - modifier[0]) > -1) and ((s_vertex[1] - modifier[1]) < self.col_count) and ((s_vertex[1] - modifier[1]) > -1)]


    def calculate_key(self, s_vertex):
        return ( min(self.gs[s_vertex], self.rhss[s_vertex]) + self.h(self.s_start, s_vertex) + self.k_m, min(self.gs[s_vertex], self.rhss[s_vertex]))

    def initialize(self):
        self.U = U(None)
        self.k_m = 0
        
        for s_vertex in self.S:         
            self.gs[s_vertex] = np.inf
            self.rhss[s_vertex] = np.inf

        self.rhss[self.s_goal] = 0        
        
        self.U.insert(self.s_goal, self.calculate_key(self.s_goal))         

    def update_vertex(self, u_vertex):
        if not u_vertex == self.s_goal:
            self.rhss[u_vertex] = min([self.gs[s_next_vertex]+self.cost(u_vertex, s_next_vertex) for s_next_vertex in self.neighbors(u_vertex)])
        if self.U.in_the(u_vertex):
            self.U.remove(u_vertex)
        if not self.gs[u_vertex] == self.rhss[u_vertex]:
            self.U.insert(u_vertex, self.calculate_key(u_vertex))        

    def compute_shortest_path(self):

        while(self.U.topKey() < self.calculate_key(self.s_start) or not self.rhss[self.s_start] == self.gs[self.s_start]):
            k_old = self.U.topKey()
            u_vertex = self.U.pop() 
            if k_old < self.calculate_key(u_vertex):
                self.U.insert(u_vertex, self.calculate_key(u_vertex))
            elif self.gs[u_vertex] > self.rhss[u_vertex]:
                self.gs[u_vertex] = self.rhss[u_vertex]
                [self.update_vertex(s_vertex) for s_vertex in self.neighbors(u_vertex)]
            else:
                self.gs[u_vertex] = np.inf
                bar = self.neighbors(u_vertex)
                bar.append(u_vertex)
                [self.update_vertex(s_vertex) for s_vertex in bar]

        pseudo_start = self.s_start
        self.planned_path = []
        self.planned_path.append(pseudo_start)
        while(not pseudo_start == self.s_goal):
            if self.gs[pseudo_start] == np.inf:
                print("Not possible to find a path.")
                break
            pseudo_start = self.find_arg_min(pseudo_start)
            self.planned_path.append(pseudo_start)   

        self.publish_path_pose()  

    def find_arg_min(self, s_start):
        vertex_to_return = None
        minimum = np.inf
        for neighbor in self.neighbors(s_start):
            bar = self.cost(s_start, neighbor) + self.gs[neighbor]
            if bar < minimum:
                minimum = bar
                vertex_to_return = neighbor

        return vertex_to_return

    def scan_graph_for_changed_edge_costs(self):
        # self.old_B = self.B.copy()
        set_difference = set(self.B) - set(self.old_B)    
        return True if len(list(set_difference)) > 0 else False

    def changed_edges(self):        
        set_difference = set(self.B) - set(self.old_B)    
        return list(set_difference)

    def main_stuff(self):         

        if self.is_init_done:
            if(not self.s_start == self.s_goal):
                if self.gs[self.s_start] == np.inf:
                    print("Something\'s wrong man.")
                    return

                if self.robot_is_on_position:
                    self.s_start = self.find_arg_min(self.s_start)   

                    self.publish_new_goal_pose(self.s_start[0], self.s_start[1])

                    self.robot_is_on_position = False


        edge_costs_changed = self.scan_graph_for_changed_edge_costs()

        if edge_costs_changed:
            self.k_m = self.k_m + self.h(self.s_last, self.s_start)
            self.last_k = self.s_start
            for edge in self.changed_edges():
                for neighbor in self.neighbors(edge):
                    self.update_vertex(neighbor)
            self.old_B = self.B.copy()
            self.compute_shortest_path()


def main(args=None):
    rclpy.init(args=args)
    d_star_lite = DStarLite(row_count=40, col_count=25, actual_borders = [], tile_size=30)
    rclpy.spin(d_star_lite) 
    rclpy.shutdown()
    d_star_lite.destroy_node()

    # d_star_lite.main_stuff()

if __name__ == '__main__':
    main()