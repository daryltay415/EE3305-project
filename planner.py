from heapq import heappush, heappop
from math import hypot, floor, inf

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
    qos_profile_services_default,
)
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path


class DijkstraNode:
    def __init__(self, c, r):
        self.parent = None
        self.f = inf
        self.g = inf
        self.h = inf
        self.c = c
        self.r = r
        self.expanded = False

    def __lt__(self, other):  # comparator for heapq (min-heap) sorting
        #return self.g < other.g
        return self.f < other.f


class Planner(Node):

    def __init__(self, node_name="planner"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("max_access_cost", int(100))

        # Parameters: Get Values
        self.max_access_cost_ = self.get_parameter("max_access_cost").value

        # Handles: Topic Subscribers
        # Global costmap subscriber
        qos_profile_latch = QoSProfile(
            history=qos_profile_services_default.history,
            depth=qos_profile_services_default.depth,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=qos_profile_services_default.reliability,
        )
        self.sub_global_costmap_ = self.create_subscription(
            OccupancyGrid,
            "global_costmap",
            self.callbackSubGlobalCostmap_,
            qos_profile_latch,
        )

        # !TODO: Path request subscriber
        self.sub_path_request_ = self.create_subscription(
            Path,
            "path_request",
            self.callbackSubPathRequest_,
            10,
        )

        # Handles: Publishers
        # !TODO: Path publisher
        self.pub_path_ = self.create_publisher(
            Path,
            "path",
            10
        )

        # Handles: Timers
        self.timer = self.create_timer(0.1, self.callbackTimer_)

        # Other Instance Variables
        self.has_new_request_ = False
        self.received_map_ = False

    # Callbacks =============================================================

    # Path request subscriber callback
    def callbackSubPathRequest_(self, msg: Path):   
        
        # !TODO: write to rbt_x_, rbt_y_, goal_x_, goal_y_
        # 0, 1?
        self.rbt_x_ = msg.poses[0].pose.position.x
        self.rbt_y_ = msg.poses[0].pose.position.y
        self.goal_x_ = msg.poses[1].pose.position.x
        self.goal_y_ = msg.poses[1].pose.position.y
        self.has_new_request_ = True

    # Global costmap subscriber callback
    # This is only run once because the costmap is only published once, at the start of the launch.
    def callbackSubGlobalCostmap_(self, msg: OccupancyGrid):
        
        # !TODO: write to costmap_, costmap_resolution_, costmap_origin_x_, costmap_origin_y_, costmap_rows_, costmap_cols_
        self.costmap_ = list(msg.data)
        self.costmap_resolution = msg.info.resolution
        self.costmap_cols_ = msg.info.width
        self.costmap_rows_ = msg.info.height
        self.costmap_origin_x_ = msg.info.origin.position.x
        self.costmap_origin_y_ = msg.info.origin.position.y
        self.received_map_ = True

    # runs the path planner at regular intervals as long as there is a new path request.
    def callbackTimer_(self):
        if not self.received_map_ or not self.has_new_request_:
            return  # silently return if no new request or map is not received.

        # run the path planner
        #self.dijkstra_(self.rbt_x_, self.rbt_y_, self.goal_x_, self.goal_y_)
        self.aStar_(self.rbt_x_, self.rbt_y_, self.goal_x_, self.goal_y_)
        self.has_new_request_ = False

    # Publish the interpolated path for testing
    def publishInterpolatedPath(self, start_x, start_y, goal_x, goal_y):
        msg_path = Path()
        msg_path.header.stamp = self.get_clock().now().to_msg()
        msg_path.header.frame_id = "map"

        dx = start_x - goal_x
        dy = start_y - goal_y
        distance = hypot(dx, dy)
        steps = distance / 0.05

        # Generate poses at every 0.05m
        for i in range(int(steps)):
            pose = PoseStamped()
            pose.pose.position.x = goal_x + dx * i / steps
            pose.pose.position.y = goal_y + dy * i / steps
            msg_path.poses.append(pose)

        # Add the goal pose
        pose = PoseStamped()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        msg_path.poses.append(pose)

        # Reverse the path (hint)
        msg_path.poses.reverse()

        # publish the path
        self.pub_path_.publish(msg_path)

        self.get_logger().info(
            f"Publishing interpolated path between Start and Goal. Implement dijkstra_() instead."
        )

    # Converts world coordinates to cell column and cell row.
    def XYToCR_(self, x, y):
        mapCoorx = floor((x - self.costmap_origin_x_) / self.costmap_resolution)
        mapCoory = floor((y - self.costmap_origin_y_) / self.costmap_resolution)
        
        return mapCoorx, mapCoory

    # Converts cell column and cell row to world coordinates.
    def CRToXY_(self, c, r):
        worldCoorx = (c + 0.5) * self.costmap_resolution + self.costmap_origin_x_
        worldCoory = (r + 0.5) * self.costmap_resolution + self.costmap_origin_y_

        return worldCoorx, worldCoory

    # Converts cell column and cell row to flattened array index.
    def CRToIndex_(self, c, r):
        totalCol = self.costmap_cols_
        flatIndex = r*totalCol + c
        return int(flatIndex)

    # Returns true if the cell column and cell row is outside the costmap.
    def outOfMap_(self, c, r):
        maxCol = self.costmap_cols_ -1
        minCol = 0
        maxRow = self.costmap_rows_ - 1
        minRow = 0
        if c > maxCol or c < minCol or r > maxRow or r < minRow:
            return True
        return False

    # Runs the path planning algorithm based on the world coordinates.
    def dijkstra_(self, start_x, start_y, goal_x, goal_y):

        # Delete both lines when ready to code planner.py -----------------
        #self.publishInterpolatedPath(start_x, start_y, goal_x, goal_y)
        #return

        # Initializations ---------------------------------

        # Initialize nodes
        #nodes = [DijkstraNode(0, 0)]  # replace this
        #print(f"costmap: {self.costmap_}")
        #testx, testy = self.XYToCR_(-5.275,-4.945)
        #testx, testy = self.CRToXY_(9,4)
        #print(f"testx = {testx}, testy = {testy}, originx = {self.costmap_origin_x_}, originy = {self.costmap_origin_y_}, reso = {self.costmap_resolution}")
        nodes = []
        for r in range(self.costmap_rows_):
            for c in range(self.costmap_cols_):
                nodes.append(DijkstraNode(c,r))
        

        # Initialize start and goal
        rbt_c, rbt_r = self.XYToCR_(start_x, start_y)
        goal_c, goal_r = self.XYToCR_(goal_x, goal_y)  # replace this
        rbt_idx = self.CRToIndex_(rbt_c, rbt_r)
        start_node = nodes[rbt_idx]
        start_node.g = 0

        # Initialize open list
        open_list = []
        heappush(open_list, start_node)

        # Expansion Loop ---------------------------------
        while len(open_list) > 0:

            # Poll cheapest node
            node = heappop(open_list)

            # Skip if visited
            if node.expanded == True:
                continue
            node.expanded = True

            # Return path if reached goal
            if node.c == goal_c and node.r == goal_r:
                msg_path = Path()
                msg_path.header.stamp = self.get_clock().now().to_msg()
                msg_path.header.frame_id = "map"

                # obtain the path from the nodes.
                
                while node.parent:
                    worldx, worldy = self.CRToXY_(node.c, node.r)
                    pose = PoseStamped()
                    pose.pose.position.x = worldx
                    pose.pose.position.y = worldy
                    msg_path.poses.append(pose)
                    node = node.parent
                pose = PoseStamped()
                worldx, worldy = self.CRToXY_(node.c, node.r)
                pose.pose.position.x = worldx
                pose.pose.position.y = worldy
                msg_path.poses.append(pose)
                msg_path.poses.reverse()
                # publish path
                self.pub_path_.publish(msg_path)

                self.get_logger().info(
                    f"Path Found from Rbt @ ({start_x:7.3f}, {start_y:7.3f}) to Goal @ ({goal_x:7.3f},{goal_y:7.3f})"
                )

                return

            # Neighbor Loop --------------------------------------------------
            for dc, dr in [
                (1, 0),
                (1, 1),
                (0, 1),
                (-1, 1),
                (-1, 0),
                (-1, -1),
                (0, -1),
                (1, -1),
            ]:
                # Get neighbor coordinates and neighbor
                
                nb_c = dc + node.c
                nb_r = dr + node.r
                
                #nb_idx = 0 * nb_c * nb_r
                nb_idx = self.CRToIndex_(nb_c, nb_r)
                #print(f"dc = {dc}, node.c = {node.c}, nb_c = {nb_c}")
                #print(f"dr = {dr}, node.c = {node.r}, nb_c = {nb_r}")
                #print(f"nb_idx = {nb_idx}, rows = {self.costmap_rows_}, cols = {self.costmap_cols_}")

                # Continue if out of map
                if self.outOfMap_(nb_c, nb_r):
                    continue

                # Get the neighbor node
                nb_node = nodes[nb_idx]

                # Continue if neighbor is expanded
                if nb_node.expanded == True:
                    continue

                # Ignore if the cell cost exceeds max_access_cost (to avoid passing through obstacles)
                if self.costmap_[nb_idx] > self.max_access_cost_:
                    continue

                # Get the relative g-cost and push to open-list
                newnb_node_g = node.g + hypot(dc,dr) * (self.costmap_[nb_idx] + 1)
                #print(f"self.costmap = {self.costmap_[nb_idx]}, newnb_nodeg={newnb_node_g}")
                if newnb_node_g < nb_node.g:
                    nb_node.g = newnb_node_g
                    nb_node.parent = node
                    heappush(open_list, nb_node) 

        self.get_logger().warn("No Path Found!")

    # Runs the path planning algorithm based on the world coordinates.
    def aStar_(self, start_x, start_y, goal_x, goal_y):

        # Delete both lines when ready to code planner.py -----------------
        #self.publishInterpolatedPath(start_x, start_y, goal_x, goal_y)
        #return

        # Initializations ---------------------------------

        # Initialize nodes
        #nodes = [DijkstraNode(0, 0)]  # replace this
        nodes = []
        for r in range(self.costmap_rows_):
            for c in range(self.costmap_cols_):
                nodes.append(DijkstraNode(c,r))
        

        # Initialize start and goal
        rbt_c, rbt_r = self.XYToCR_(start_x, start_y)
        goal_c, goal_r = self.XYToCR_(goal_x, goal_y)  # replace this
        rbt_idx = self.CRToIndex_(rbt_c, rbt_r)
        start_node = nodes[rbt_idx]
        start_node.g = 0

        # Initialize open list
        open_list = []
        heappush(open_list, start_node)

        # Expansion Loop ---------------------------------
        while len(open_list) > 0:

            # Poll cheapest node
            node = heappop(open_list)

            # Skip if visited
            if node.expanded == True:
                continue
            node.expanded = True

            # Return path if reached goal
            if node.c == goal_c and node.r == goal_r:
                msg_path = Path()
                msg_path.header.stamp = self.get_clock().now().to_msg()
                msg_path.header.frame_id = "map"

                # obtain the path from the nodes.
                while node.c != rbt_c or node.r != rbt_r:
                    worldx, worldy = self.CRToXY_(node.c, node.r)
                    pose = PoseStamped()
                    pose.pose.position.x = worldx
                    pose.pose.position.y = worldy
                    msg_path.poses.append(pose)
                    node = node.parent
                worldx, worldy = self.CRToXY_(node.c, node.r)
                pose = PoseStamped()
                pose.pose.position.x = worldx
                pose.pose.position.y = worldy
                msg_path.poses.append(pose)
                msg_path.poses.reverse()
                # publish path
                self.pub_path_.publish(msg_path)

                self.get_logger().info(
                    f"Length of path = {len(msg_path.poses)}"
                )

                return

            # Neighbor Loop --------------------------------------------------
            for dc, dr in [
                (1, 0),
                (1, 1),
                (0, 1),
                (-1, 1),
                (-1, 0),
                (-1, -1),
                (0, -1),
                (1, -1),
            ]:
                # Get neighbor coordinates and neighbor
                nb_c = dc + node.c
                nb_r = dr + node.r
                #nb_idx = 0 * nb_c * nb_r
                nb_idx = self.CRToIndex_(nb_c, nb_r)

                # Continue if out of map
                if self.outOfMap_(nb_c, nb_r):
                    continue

                # Get the neighbor node
                nb_node = nodes[nb_idx]

                # Continue if neighbor is expanded
                if nb_node.expanded == True:
                    continue

                # Ignore if the cell cost exceeds max_access_cost (to avoid passing through obstacles)
                if self.costmap_[nb_idx] > self.max_access_cost_:
                    continue

                # Get the relative g-cost and push to open-list
                newnb_node_g = node.g + hypot(dc,dr) * (self.costmap_[nb_idx] + 1)
                if newnb_node_g < nb_node.g:
                    nb_node.g = newnb_node_g
                    nb_node.f = newnb_node_g + ((nb_c - goal_c)**2 + (nb_r - goal_r)**2) ** (0.5) 
                    nb_node.parent = node
                    heappush(open_list, nb_node) 

        self.get_logger().warn("No Path Found!")


# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Planner())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
