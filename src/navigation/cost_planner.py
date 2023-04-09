#!/usr/bin/env python

import rospy
import math
import heapq
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class CostPlanner:
    # design as a service
    # def __init__(self):
    #     rospy.init_node("cost_planner")
        
    #     self.goal = None
    #     self.costmap = None

    # def goal_callback(self, data):
    #     self.goal = data

    # def costmap_callback(self, data):
    #     self.costmap = data

    # def run(self):
    #     while not rospy.is_shutdown():
    #         if self.goal and self.costmap:
    #             plan, path = self.generate_plan_and_path(self.goal, self.costmap)
    #             self.plan_pub.publish(plan)
    #             self.path_pub.publish(path)
    #             self.goal = None
    #             self.costmap = None
    #         self.rate.sleep()

    def generate_plan_and_path(self, goal, costmap):
        # will be in the map frame 
        resolution = costmap.info.resolution
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        width = costmap.info.width
        height = costmap.info.height
        map_data = costmap.data
        # start_x, start_y = self.world_to_map(origin_x, origin_y, resolution, goal.pose.position.x, goal.pose.position.y)

        # we want start is relative to the rover
        start_x, start_y = self.world_to_map(origin_x, origin_y, resolution, 0, 0)
        start = (start_x, start_y)
        goal = (goal_x, goal_y)

        # goal from nav and path sent to nav
        goal_x, goal_y = self.world_to_map(origin_x, origin_y, resolution, goal.pose.orientation.x, goal.pose.orientation.y)

        # A*
        def heuristic(node):
            return math.sqrt((node[0] - goal[0]) ** 2 + (node[1] - goal[1]) ** 2)

        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {}
        cost_so_far = {}
        came_from[start] = None
        cost_so_far[start] = 0

        while len(frontier) > 0:
            current = heapq.heappop(frontier)[1]

            if current == goal:
                break

            for next in self.get_neighbors(current[0], current[1], width, height):
                new_cost = cost_so_far[current] + self.get_cost(current[0], current[1], next[0], next[1], map_data, width)

                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + heuristic(next)
                    heapq.heappush(frontier, (priority, next))
                    came_from[next] = current;

        # Reconstruct path
        path = [goal]
        current = goal
        while current != start:
            current = came_from[current]
            path.append(current)
        path.reverse()

        # Convert path from map coordinates to world coordinates
        plan = []
        for node in path:
            x, y = self.map_to_world(origin_x, origin_y, resolution, node[0], node[1])
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            plan.append(pose)

        return plan, path
        