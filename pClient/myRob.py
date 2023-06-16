
import sys
from typing import List, Literal, Tuple, Union
from croblink import *
from math import *

Coords = Tuple[int, int]

GOAL_THRESH = 0.05
TURNING_THRESH = 0.1
DRIVE_STRAIGHT_THRESH = 0.05
ROBOT_DIST_TO_NODE_THRESH = 0.1
UPDATE_ANGLE_TOLERANCE = pi/2*0.06
WALL_THICKNESS = 0.1

NODE_RANGE_THRESH = 0.3
MIN_READINGS_TO_UPDATE = 4
MIN_MOVEMENT_TO_UPDATE = 0.5



class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, fname):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)

        self.fname = fname
        self.nodemap = NodeMap()
        self.node_neighborhood = None
        self.plan = []

        self.diameter = 1
        self.line_sensor_dist = 0.438
        self.pose = [0, 0, 0]

        self.entered_line = False
        self.spinning = False
        self.out1 = 0
        self.out2 = 0



    def plan_final_path(self) -> List[Coords]:
        """Plans shortest path that passes through all spots, starting and ending on position 0"""

        # calculate shortest path between every pair of beacons and find smallest sum?

        pass

    def correct_estimation(self):
        """Uses line sensor and compass measures to correct its position and orientation estimate."""

        # correct orientation with compass measure
        # TODO: edge case where prediction is close to -pi and compass is close to pi? (and vice-versa)
        #self.pose[2] = 0.2*self.pose[2] + 0.8*radians(self.measures.compass)
        self.pose[2] = radians(self.measures.compass)


        # # correct position with linesensor measures, if either left half or right half is detecting line
        # if not self.entered_line \
        # and all(s == "1" for s in self.measures.lineSensor[3:]) \
        # or all(s == "1" for s in self.measures.lineSensor[:4]):
            
        #     self.entered_line = True

        #     # offset from robot center position to sensor position
        #     offset_x = cos(self.pose[2])*self.line_sensor_dist
        #     offset_y = sin(self.pose[2])*self.line_sensor_dist

        #     # determine where predicted sensor position is
        #     sensor_pos_x = self.pose[0] + offset_x
        #     sensor_pos_y = self.pose[1] + offset_y

        #     # determine closest node position to sensor
        #     node_x = round(sensor_pos_x)
        #     node_y = round(sensor_pos_y)
        #     # if one of the coordinates is odd, abort
        #     #if node_x % 2 != 0 or node_y % 2 != 0:
        #     #    self.entered_line = False
        #     #    return

        #     # determined corrected sensor position
        #     correction_x = sensor_pos_x
        #     correction_y = sensor_pos_y
        #     facing: Literal[None, "up", "down", "left", "right"] = None

        #     if abs(offset_x) < abs(offset_y) and offset_y > 0:
        #         correction_y = node_y
        #         facing = "up"
        #     elif abs(offset_x) < abs(offset_y) and offset_y < 0:
        #         correction_y = node_y
        #         facing = "down"
        #     elif abs(offset_y) < abs(offset_x) and offset_x > 0:
        #         correction_x = node_x
        #         facing = "right"
        #     elif abs(offset_y) < abs(offset_x) and offset_x < 0:
        #         correction_x = node_x
        #         facing = "left"
            
        #     corrected_sensor = (correction_x, correction_y)


        #     # does position need correction? (update only if it should not be possible to see the line from predicted position)
        #     corrected_pos_x = corrected_sensor[0] - offset_x
        #     corrected_pos_y = corrected_sensor[1] - offset_y
        #     if (facing == "up" and self.pose[1] < corrected_pos_y - WALL_THICKNESS) \
        #     or (facing == "down" and self.pose[1] > corrected_pos_y + WALL_THICKNESS) \
        #     or (facing == "left" and self.pose[0] > corrected_pos_x + WALL_THICKNESS) \
        #     or (facing == "right" and self.pose[0] < corrected_pos_x - WALL_THICKNESS):
        #         print(f"correcting position from {(self.pose[0], self.pose[1])} to {(corrected_pos_x, corrected_pos_y)}")
        #         self.pose[0] = corrected_pos_x
        #         self.pose[1] = corrected_pos_y

        # else:
        #     self.entered_line = False



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
        if abs(dest[0] - x) < GOAL_THRESH and abs(dest[1] - y) < GOAL_THRESH:
            #print(f"reached destination {dest} ({(x,y,th)})")
            self.commit_neighbors()
            self.plan.pop(0)
            if not self.plan:
                self.out1, self.out2 = 0, 0
                # apply movement model to pose
                lin = (self.out1 + self.out2)/2
                new_x = x + lin*cos(th)
                new_y = y + lin*sin(th)
                new_th = th + (self.out2 - self.out1)/self.diameter
                # wrap orientation to [-pi, pi]
                new_th = new_th % (2*pi)
                if new_th > pi: new_th = -(2*pi - new_th)
                self.pose = [new_x, new_y, new_th]
                self.driveMotors(0,0)
                return
            dest = self.plan[0]
            

        # determine whether to rotate in place or go forward
        ang = atan2( dest[1]-y, dest[0]-x )
        #if ang < 0: ang = 2*pi + ang  # shift to [0, 2pi] range

        diff = th-ang
        # normalize difference
        diff = diff % (2*pi)
        if diff > pi: diff = -(2*pi - diff)


        if abs(diff) < DRIVE_STRAIGHT_THRESH:
            # destination is straight ahead (...almost)
            self.out1, self.out2 = 0.1, 0.1
        elif abs(diff) > TURNING_THRESH*2 and diff < 0:
            # destination is to the left of current orientation
            self.out1, self.out2 = -0.05, 0.05
        elif abs(diff) > TURNING_THRESH and diff < 0:
            # destination is slightly left of current orientation
            self.out1, self.out2 = 0.05, 0.08
        elif abs(diff) > TURNING_THRESH*2 and diff > 0:
            # destination is to the right of current orientation
            self.out1, self.out2 = 0.05, -0.05
        elif abs(diff) > TURNING_THRESH and diff > 0:
            # destination is slightly right of current orientation
            self.out1, self.out2 = 0.08, 0.05


        # apply movement model to pose
        lin = (self.out1 + self.out2)/2
        new_x = x + lin*cos(th)
        new_y = y + lin*sin(th)
        new_th = th + (self.out2 - self.out1)/self.diameter
        # wrap orientation to [-pi, pi]
        new_th = new_th % (2*pi)
        if new_th > pi: new_th = -(2*pi - new_th)

        self.pose = [new_x, new_y, new_th]
        self.driveMotors(self.out1, self.out2)


    def spin(self):
        x, y, th = self.pose
        self.out1 = 0.05
        self.out2 = -0.05
        # apply movement model to pose
        lin = (self.out1 + self.out2)/2
        new_x = x + lin*cos(th)
        new_y = y + lin*sin(th)
        new_th = th + (self.out2 - self.out1)/self.diameter
        # wrap orientation to [-pi, pi]
        new_th = new_th % (2*pi)
        if new_th > pi: new_th = -(2*pi - new_th)

        self.pose = [new_x, new_y, new_th]
        self.driveMotors(self.out1, self.out2)



    def update_neighbors(self):

        sensor_pos_x = self.pose[0] + cos(self.pose[2])*self.line_sensor_dist
        sensor_pos_y = self.pose[1] + sin(self.pose[2])*self.line_sensor_dist

        # closest node to sensor
        node_x = round(sensor_pos_x)
        node_y = round(sensor_pos_y)
        if node_x % 2 != 0 or node_y % 2 != 0: return
        if self.nodemap.get_node((node_x, node_y)).explored: return


        # some tolerance in angle ranges
        tol = UPDATE_ANGLE_TOLERANCE

        facing = None

        if pi/2-tol < self.pose[2] < pi/2+tol:
            facing = "up"
        elif -pi/2-tol < self.pose[2] < -pi/2+tol:
            facing = "down"
        elif (pi-tol < self.pose[2] or self.pose[2] < -pi+tol):
            facing = "left"
        elif -tol < self.pose[2] < tol:
            facing = "right"
        else:
            # don't update if not facing up, down, left or right
            #print("outside tolerance, discarding reading...")
            return
        

        # when the sensor enters the area around a node
        node_dist = sqrt( (node_x - sensor_pos_x)**2 + (node_y - sensor_pos_y)**2 )
        if node_dist > NODE_RANGE_THRESH: return

        if self.node_neighborhood == None:
            self.node_neighborhood = {
                "readings": 0,
                "starting_pos": (self.pose[0], self.pose[1]),
                "node": (node_x,node_y),
                "facing": facing,
                "left": False, "right": False   # if left/right side of line sensor have detected something
            }

        ### update neighborhood
        self.node_neighborhood["readings"] += 1
        if self.measures.lineSensor[0] == "1":  self.node_neighborhood["left"] = True
        if self.measures.lineSensor[-1] == "1": self.node_neighborhood["right"] = True


    def commit_neighbors(self):

        if self.node_neighborhood == None: return

        x, y = self.node_neighborhood["node"]
        ix, iy = self.node_neighborhood["starting_pos"]

        # only commit edges if enough readings were made
        if self.node_neighborhood["readings"] < MIN_READINGS_TO_UPDATE:
            print(f"Not enough readings ({self.node_neighborhood['readings']}), not commiting...")
            self.node_neighborhood = None
            return
        # only commit edges if robot moved more than set amount
        if sqrt( (self.pose[0] - ix)**2 + (self.pose[1] - iy)**2 ) < MIN_MOVEMENT_TO_UPDATE:
            print(f"Not enough distance ({sqrt( (self.pose[0] - ix)**2 + (self.pose[1] - iy)**2 )}), not commiting...")
            self.node_neighborhood = None
            return

        # left and right refer to robot's relative left and right, not global
        left, right = self.node_neighborhood["left"], self.node_neighborhood["right"]

        # facing refers to global orientation
        if self.node_neighborhood["facing"] == "up":
            l_node = (x-2,y)
            r_node = (x+2,y)
        elif self.node_neighborhood["facing"] == "down":
            l_node = (x+2,y)
            r_node = (x-2,y)
        elif self.node_neighborhood["facing"] == "left":
            l_node = (x,y-2)
            r_node = (x,y+2)
        else:
            l_node = (x,y+2)
            r_node = (x,y-2)

        try:
            self.nodemap.add_new_path((x,y), l_node, null_path=(not left))
        except Exception as e:
            print(f"Failed to add edge between {(x, y)} and {l_node}")
            print(f"  Null edge: {not left}")
            print(f"  {self.pose}")
            print(f"  {e}")
        try:
            self.nodemap.add_new_path((x,y), r_node, null_path=(not right))
        except Exception as e:
            print(f"Failed to add edge between {(x, y)} and {r_node}")
            print(f"  Null edge: {not right}")
            print(f"  {self.pose}")
            print(f"  {e}")

        self.node_neighborhood = None


    def update_map(self):
        """
        Uses the current sensor measures to update the map (locating spots, adding paths to map).
        Only updates paths between closest current node and the node in front of it.
        """

        # some tolerance in angle ranges
        tol = UPDATE_ANGLE_TOLERANCE

        facing = None

        if pi/2-tol < self.pose[2] < pi/2+tol:
            facing = "up"
        elif -pi/2-tol < self.pose[2] < -pi/2+tol:
            facing = "down"
        elif (pi-tol < self.pose[2] or self.pose[2] < -pi+tol):
            facing = "left"
        elif -tol < self.pose[2] < tol:
            facing = "right"
        else:
            # don't update if not facing up, down, left or right
            return
    

        # don't update if robot not near node coordinates
        node_x = round(self.pose[0])
        node_y = round(self.pose[1])
        if (
            #abs(node_x - self.pose[0]) > ROBOT_DIST_TO_NODE_THRESH
            #or abs(node_y - self.pose[1]) > ROBOT_DIST_TO_NODE_THRESH
            sqrt( (node_x - self.pose[0])**2 + (node_y - self.pose[1])**2 ) > ROBOT_DIST_TO_NODE_THRESH
            or node_x % 2 != 0
            or node_y % 2 != 0
        ):
            return

        if self.measures.ground != -1:
            self.nodemap.set_beacon((node_x, node_y), self.measures.ground)

        if facing == "up":
            next_node = (node_x, node_y+2)
        elif facing == "down":
            next_node = (node_x, node_y-2)
        elif facing == "left":
            next_node = (node_x-2, node_y)
        else:
            next_node = (node_x+2, node_y)

        line_detected = any(m == "1" for m in self.measures.lineSensor[2:5])
        try:
            self.nodemap.add_new_path((node_x,node_y), next_node, null_path=(not line_detected))
        except Exception as e:
            print(f"Failed to add edge between {(node_x, node_y)} and {next_node}")
            print(f"  Null edge: {not line_detected}")
            print(f"  {self.pose}")
            print(f"  {e}")

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
                    self.update_neighbors()
                    self.move_by_plan()
                # if there is no plan and there is still more to explore, get new destination
                elif phase == "exploring" and not self.nodemap.is_fully_explored():
                    curr_pos = (round(self.pose[0]), round(self.pose[1]))

                    if not self.nodemap.get_node(curr_pos).explored:
                        #print("Spinning at", (self.pose[0], self.pose[1]))
                        self.spin()
                    else:
                        next_dest = self.nodemap.get_closest_unexplored(curr_pos)
                        self.plan = self.nodemap.plan_path(curr_pos, next_dest)
                        print(f"Going from {(self.pose[0], self.pose[1])} to {next_dest} now...")
                        print("Plan:", self.plan)

                # if there is no plan and there is no more to explore, then calculate path through all beacons and return to (0,0)
                elif phase == "exploring":
                    print("Done exploring!")
                    phase = "finalizing"
                    self.plan_final_path()
                    # TODO: write path to file
                    curr_pos = (round(self.pose[0], round(self.pose[1])))
                    self.plan = self.nodemap.plan_path(curr_pos, (0,0))
                    self.setReturningLed(True)
                # not exploring and no plan, meaning robot has returned to (0,0) and accomplished all tasks
                else:
                    self.finish()
            
            if self.measures.endLed:
                quit()


    
    def test(self):

        #print(self.measures.lineSensor)

        state = 'stop'
        stopped_state = 'run'

        self.plan = [(2,0), (2,-2), (4,-2), (4,0), (4,2), (6,2), (6,0), (8,0), (8,-2), (6,-2), (6,-4), (4,-4), (2,-4), (0,-4), (0,-2)]

        while True:

            self.readSensors()
            self.correct_estimation()

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

            if self.measures.endLed:
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
                #print(f"Added edge between {(self.x, self.y)} and {(node.x, node.y)}")

        
        def add_null_edge(self, direction: Literal["up", "down", "left", "right"]):
            if direction not in self.null_edges and self.explored:
                raise Exception("Node already fully explored!")
            self.null_edges.add(direction)

        @property
        def explored(self):
            """A node is considered to be explored when 4 edges or null edges are known"""
            return len(self.edges) + len(self.null_edges) == 4
        
        @property
        def reachable(self) -> bool:
            """A node cannot be reached if it has no edges"""
            return len(self.edges) > 0
        
        def __eq__(self, __value: object) -> bool:
            return self.x == __value.x and self.y == __value.y
        
        def __str__(self) -> str:
            return f"({self.x}, {self.y}): Edges{[(n.x, n.y) for n in self.edges]}, Null Edges:{self.null_edges}"


    def __init__(self) -> None:
        self.nodes: List[List[NodeMap.Node]] = [] # index self.nodes with [x][y]
        self.node_coords: List[Coords] = []  # stores internal coordinates of known nodes, not the world coordinates
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
    
    def set_beacon(self, coords: Coords, beacon_id: int):
        x,y = coords
        x = int(x/2 + 12)
        y = int(y/2 + 5)

        if self.nodes[x][y] is None:
            self.add_node(self.Node(coords), beacon_id)
        else:
            self.beacon_coords[beacon_id] = (coords[0], coords[1])
            self.coords_to_beacon[(coords[0], coords[1])] = beacon_id

    def add_new_path(self, origin: Coords, end: Coords, null_path = False):
        """Adds edge between nodes at origin and end coordinates, creating those node if they don't exist.
        If `null_path` is True, adds a null edge, meaning there is no path between those nodes."""

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
            #print(f"Added null edge between {origin} and {end}")

    def is_fully_explored(self) -> bool:
        return all( self.nodes[x][y].explored for x,y in self.node_coords )

    def get_closest_unexplored(self, curr: Coords) -> Coords:
        """Use a heuristic to pick closest unexplored node"""
        best = None
        best_dist = 1000

        for x,y in self.node_coords:
            node = self.nodes[x][y]
            if not node.reachable or node.explored or (node.x, node.y) == curr: continue

            # manhattan distance to node, plus 1, minus 1 if there exists a direct edge to node
            distance = abs(curr[0] - node.x) + abs(curr[1] - node.y) + 1 - int(curr in [(n.x, n.y) for n in node.edges])

            if distance < best_dist:
                best = node
                best_dist = distance
        
        return (best.x, best.y)

    def plan_path(self, start: Coords, goal: Coords) -> List[Coords]:
        """Returns path from start to goal, using A* search. Coordinates passed are world coordinates, not internal.
        Algorithm adapted from https://en.wikipedia.org/wiki/A*_search_algorithm.
        """
        # heuristic function: manhattan distance to goal
        h = lambda n: abs(n[0] - goal[0]) + abs(n[1] - goal[1])

        open_set = [start]  # keep sorted by heuristic, with smallest value at the end
        came_from = {}
        g_score = { start: 0 }
        f_score = { start: h(start)}

        while len(open_set) > 0:
            current = open_set.pop()

            # goal found, return path
            if current == goal:
                final_path = [current]
                while current in came_from.keys():
                    current = came_from[current]
                    final_path.append(current)
                final_path.reverse()
                return final_path
        
            for neighbor in self.get_node(current).edges:
                neighbor_coords = (neighbor.x, neighbor.y)
                tentative_g_score = g_score[current] + 1  # all edges have weight 1
                if tentative_g_score < g_score.get(neighbor_coords, inf):
                    came_from[neighbor_coords] = current
                    g_score[neighbor_coords] = tentative_g_score
                    f_score[neighbor_coords] = tentative_g_score + h(neighbor_coords)
                    if neighbor_coords not in open_set:
                        open_set.append(neighbor_coords)
                        open_set.sort(key=h, reverse=True)
        
        return []


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
        
        s_mat.reverse()
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
    
    try:
        rob.run()
        #rob.test()
    except KeyboardInterrupt:
        for node in [ rob.nodemap.nodes[x][y] for x,y in rob.nodemap.node_coords ]:
            print(str(node))
        quit()
