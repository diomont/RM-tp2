
import sys
from typing import List, Literal, Tuple, Union
from croblink import *
from math import *

Coords = Tuple[int, int]

TURNING_THRESH = 0.2
DRIVE_STRAIGHT_THRESH = 0.05
SENSOR_DIST_TO_NODE_THRESH = 0.2
UPDATE_ANGLE_TOLERANCE = pi/2*0.03


class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, fname):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

        self.fname = fname
        self.nodemap = NodeMap()
        self.plan = []

        self.diameter = 1
        self.line_sensor_dist = 0.438
        self.pose = [0, 0, 0]
        self.goal_thresh = 0.05

        self.turning = False
        self.out1 = 0
        self.out2 = 0



    def plan_final_path(self) -> List[Coords]:
        """Plans shortest path that passes through all spots, starting and ending on position 0"""

        # calculate shortest path between every pair of beacons and find smallest sum?

        pass

    def correct_estimation(self):
        """Uses line sensor and compass measures to correct its position and orientation estimate."""
        pass

    def move_by_plan(self):
        """
        Sends `driveMotors` command such that the robot drives to the next position in the current plan.
        The robot will only ever move up, down, left or right, decided by comparing the current position estimate with the current goal.
        When the distance between the position estimate and the goal is below a certain threshold, the goal is removed from the plan.

        Calling this method once will not complete all of the plan, rather it makes progress towards its next goal,
        such as rotating a small amount each call to face the next destination.
        It should be used in a `while` loop after a `readSensors` call, and as such allows for other code to run at each step of the plan execution.
        """

        dest = self.plan[0]
        x, y, th = self.pose
        # check if close enough to destination to remove from plan
        if abs(dest[0] - x) < self.goal_thresh and abs(dest[1] - y) < self.goal_thresh:
            self.plan.pop(0)
            if not self.plan: return
            dest = self.plan[0]

        # determine whether to rotate in place or go forward
        ang = atan2( dest[1]-y, dest[0]-x )
        #if ang < 0: ang = 2*pi + ang  # shift to [0, 2pi] range

        diff = th-ang

        if abs(diff) < 0.05 or abs(diff) > 2*pi - 0.05:
            # destination is straight ahead (...almost)
            self.out1, self.out2 = 0.1, 0.1
        elif abs(diff) > 0.2 and (diff < 0 or diff >= pi):
            # destination is to the left of current orientation
            self.out1, self.out2 = -0.05, 0.05
        elif abs(diff) > 0.2:
            # destination is to the right of current orientation
            self.out1, self.out2 = 0.05, -0.05


        # apply movement model to pose
        lin = (self.out1 + self.out2)/2
        new_x = x + lin*cos(th)
        new_y = y + lin*sin(th)
        new_th = th + (self.out2 - self.out1)/self.diameter

        # TODO: check if wrapping orientation is necessary

        self.pose = [new_x, new_y, new_th]
        self.driveMotors(self.out1, self.out2)



    def update_map(self):
        """
        Uses the current sensor measures to update the map (locating spots, adding paths to map).
        """

        # some tolerance in angle ranges
        tol = UPDATE_ANGLE_TOLERANCE

        # don't update map if not facing up, down, left or right
        # if not (
        #     pi/2-tol < self.pose[2] < pi/2+tol
        #     or pi*1.5-tol < self.pose[2] < pi*1.5+tol
        #     or (2*pi-tol < self.pose[2] or self.pose[2] < tol)
        #     or pi-tol < self.pose[2] < pi+tol
        # ):
        if not (
            pi/2-tol < self.pose[2] < pi/2+tol
            or -pi/2-tol < self.pose[2] < -pi/2+tol
            or (pi-tol < self.pose[2] or self.pose[2] < -pi+tol)
            or -tol < self.pose[2] < tol
        ):
            return

        sensor_pos_x = self.pose[0] + cos(self.pose[2])*self.line_sensor_dist
        sensor_pos_y = self.pose[1] + sin(self.pose[2])*self.line_sensor_dist

        # TODO: don't update if sensor_pos is too close to node, to deal with position error?
        far_from_node = True
        closest_node = (round(sensor_pos_x), round(sensor_pos_y))
        if closest_node[0] % 2 == 0 and closest_node[1] % 2 == 0:
            dist_to_node = abs(sensor_pos_x - closest_node[0]) + abs(sensor_pos_y - closest_node[1])
            if dist_to_node < SENSOR_DIST_TO_NODE_THRESH: 
                far_from_node = False
        

        # TODO: orientation is [-pi, pi] or [0, 2pi]?  it's -pi to pi
        if pi/2-tol < self.pose[2] < pi/2+tol \
        or -pi/2-tol < self.pose[2] < -pi/2+tol:
            # facing up or down

            # nearest node coordinates
            x = round(sensor_pos_x)
            y_down = floor(sensor_pos_y)
            y_up = ceil(sensor_pos_y)
            if y_down == y_up:
                return # i don't feel like programming this edge case
            elif y_down % 2 == 0:
                y_up += 1     # down is closest to node, move y_up further up to match closest node coordinate up
            else:
                y_down -= 1   # up is closest to node, move y_down further down to match closest node coordinate down

            try:
                if far_from_node and any(v == "1" for v in self.measures.lineSensor[2:5]):                
                    # add new path between nodes
                    self.nodemap.add_new_path((x,y_up), (x,y_down))
                elif far_from_node:
                    # no edge between nodes
                    self.nodemap.add_new_path((x,y_up), (x,y_down), null_path=True)
            except Exception as e:
                print(self.pose)
                print(sensor_pos_x, sensor_pos_y)
                print(f"x: {x}, up: {y_up}, down: {y_down}")
                print(e)

        elif (pi-tol < self.pose[2] or self.pose[2] < -pi+tol) \
        or   -tol < self.pose[2] < tol:
            # facing left or right

            # nearest node coordinates
            y = round(sensor_pos_y)
            x_left = floor(sensor_pos_x)
            x_right = ceil(sensor_pos_x)
            if x_left == x_right:
                return # i don't feel like programming this edge case
            elif x_left % 2 == 0:
                x_right += 1  # left is closest to node, move x_right further right to match closest node coordinate to the right
            else:
                x_left -= 1   # right is closest to node, move x_left further left to match closest node coordinate to the left

            try:
                if far_from_node and any(v == "1" for v in self.measures.lineSensor[2:5]):
                    # add new path between nodes
                    self.nodemap.add_new_path((x_left,y), (x_right,y))
                elif far_from_node:
                    # no edge between nodes
                    self.nodemap.add_new_path((x_left,y), (x_right,y), null_path=True)
            except Exception as e:
                print(self.pose)
                print(sensor_pos_x, sensor_pos_y)
                print(f"y: {y}, left: {x_left}, right: {x_right}")
                print(e)


        # TODO: update map with line sensor readings from edges of sensor

        self.write_map_to_file()


    def write_map_to_file(self):
        s = str(self.nodemap)
        with open(self.fname + ".map", "w") as f:
            f.write(s)


    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'
        phase = "exploring"


        while True:

            self.readSensors()
            self.correct_estimation()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == "run":

                if phase == "exploring":
                    self.update_map()

                # always move according to plan if there is one
                if self.plan:
                    self.move_by_plan()
                # if there is no plan and there is still more to explore, get new destination
                elif phase == "exploring" and not self.nodemap.is_fully_explored():
                    curr_pos = (round(self.pose[0], round(self.pose[1])))
                    next_dest = self.nodemap.get_closest_unexplored(curr_pos)
                    self.plan = self.nodemap.plan_path(curr_pos, next_dest)

                    # TODO: spin in place to find paths


                # if there is no plan and there is no more to explore, then calculate path through all beacons and return to (0,0)
                elif phase == "exploring":
                    phase = "finalizing"
                    self.plan_final_path()
                    # TODO: write path to file
                    curr_pos = (round(self.pose[0], round(self.pose[1])))
                    self.plan = self.nodemap.plan_path(curr_pos, (0,0))
                    self.setReturningLed(True)
                # not exploring and no plan, meaning robot has returned to (0,0) and accomplished all tasks
                else:
                    self.finish()
                    quit()


    
    def test(self):

        #print(self.measures.lineSensor)

        state = 'stop'
        stopped_state = 'run'

        self.plan = [(2,0), (2,-2), (4,-2), (4,0), (4,2), (6,2), (6,0), (8,0), (8,-2), (6,-2), (6,-4), (4,-4), (2,-4), (0,-4), (0,-2)]

        while True:

            self.readSensors()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == "run":

                self.update_map()

                if self.plan:
                    self.move_by_plan()
                    #print(self.pose)
                else:
                    self.finish()

                    for node in [ self.nodemap.nodes[x][y] for x,y in self.nodemap.node_coords ]:
                        print(str(node))

                    quit()


class NodeMap():

    class Node():
        def __init__(self, coords, starting_edge = None) -> None:
            self.x = coords[0]
            self.y = coords[1]
            self.edges = [starting_edge] if starting_edge else []
            self.null_edges = set()  # can contain "up", "down", "left" and "right". their presence means that there is no path in that direction
        
        def add_edge(self, node):
            if node not in self.edges:
                if self.explored:
                    raise Exception(f"Cannot add edge towards {(node.x, node.y)}, node {self} already fully explored!")
                self.edges.append(node)
                print(f"Added edge between {(self.x, self.y)} and {(node.x, node.y)}")

        
        def add_null_edge(self, direction: Literal["up", "down", "left", "right"]):
            if direction not in self.null_edges and self.explored:
                raise Exception("Node already fully explored!")
            self.null_edges.add(direction)

        @property
        def explored(self):
            """A node is considered to be explored when 4 edges or null edges are known"""
            return len(self.edges) + len(self.null_edges) == 4
        
        def __eq__(self, __value: object) -> bool:
            return self.x == __value.x and self.y == __value.y
        
        def __str__(self) -> str:
            return f"({self.x}, {self.y}): Edges{[(n.x, n.y) for n in self.edges]}, Null Edges:{self.null_edges}"


    def __init__(self) -> None:
        self.nodes = [] # index self.nodes with [x][y]
        self.node_coords = []  # stores internal coordinates of known nodes, not the world coordinates
        for _ in range(25):
            self.nodes.append([None]*11)

        start = self.Node((0,0))
        self.beacon_coords = {0: (0,0)}
        self.coords_to_beacon = {(0,0): 0}
        self.add_node(start)

    def add_node(self, node: Node, beacon_id = None):
        if node.x % 2 != 0 or node.y % 2 != 0:
            raise Exception("Node cannot have odd coordinates")

        print(f"Added node {(node.x, node.y)}")
        x,y = node.x, node.y
        x = int(x/2 + 12)
        y = int(y/2 + 5)
        #print(x, y)
        self.nodes[x][y] = node
        self.node_coords.append((x,y))
        if beacon_id:
            self.beacon_coords[beacon_id] = (node.x, node.y)
            self.coords_to_beacon[(node.x, node.y)] = beacon_id

    def get_node(self, coords: Coords) -> Union[Node, None]:
        x,y = coords
        x = int(x/2 + 12)
        y = int(y/2 + 5)
        return self.nodes[x][y]
    
    def add_new_path(self, origin: Coords, end: Coords, null_path = False):
        origin_node =  self.get_node(origin)
        end_node = self.get_node(end)
        if origin_node is None:
            origin_node = self.Node(origin)
            self.add_node(origin_node)
        if end_node is None:
            end_node = self.Node(end)
            self.add_node(end_node)
        
        if not null_path:
            origin_node.add_edge(end_node)
            end_node.add_edge(origin_node)
        else:
            if origin[0] < end[0]:  # origin left of end
                origin_node.add_null_edge("right")
                end_node.add_null_edge("left")
            elif origin[0] > end[0]:  # origin right of end
                origin_node.add_null_edge("left")
                end_node.add_null_edge("right")
            elif origin[1] > end[1]:  # origin above end
                origin_node.add_null_edge("down")
                end_node.add_null_edge("up")
            else:  # origin bellow end
                origin_node.add_null_edge("up")
                end_node.add_null_edge("down")
            print(f"Added null edge between {origin} and {end}")

    def is_fully_explored(self) -> bool:
        return all( self.nodes[x][y].explored for x,y in self.node_coords )

    def get_closest_unexplored(self, curr: Coords) -> Coords:
        """Uses manhattan distance as heuristic to pick closest unexplored node"""
        best = None
        best_dist = 1000

        for x,y in self.node_coords:
            node = self.nodes[x][y]
            if node.explored or (node.x, node.y) == curr: continue

            distance = abs(curr[0] - node.x) + abs(curr[1] - node.y)

            if distance < best_dist:
                best = node
                best_dist = distance
        
        return (best.x, best.y)

    def plan_path(self, start: Coords, goal: Coords) -> List[Coords]:
        pass

    def __str__(self) -> str:
        s_mat = []
        # 21 lines of 49 characters each
        for _ in range(21):
            s_mat.append([" "]*49)

        for x,y in self.node_coords:
            node = self.nodes[x][y]
            if (node.x, node.y) in self.coords_to_beacon:
                s_mat[y*2][x*2] = str(self.coords_to_beacon[(node.x, node.y)])
            for con_node in node.edges:
                diff_x = -1 if con_node.x < node.x else 1 if con_node.x > node.x else 0
                diff_y = -1 if con_node.y < node.y else 1 if con_node.y > node.y else 0
                s_mat[y*2+diff_y][x*2+diff_x] = "|" if diff_x == 0 else "-"
        
        s = ""
        for line in s_mat:
            s += "".join(line) + "\n"
        return s



rob_name = "pClient1"
fname = "mymap"
host = "localhost"
pos = 1

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--fname" or sys.argv[i] == "-f") and i != len(sys.argv) - 1:
        fname = sys.argv[i + 1]
    else:
        print("Unknown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host,fname)
    
    #rob.run()
    rob.test()