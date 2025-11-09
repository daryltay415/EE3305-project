from math import hypot, atan2, inf, cos, sin, pi, asin, e, exp
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, qos_profile_services_default
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan


class Controller(Node):

    def __init__(self, node_name="controller"):
        # Node Constructor =============================================================
        super().__init__(node_name)

        # Parameters: Declare
        self.declare_parameter("frequency", float(20))
        self.declare_parameter("lookahead_distance", float(0.3))
        self.declare_parameter("lookahead_lin_vel", float(0.1))
        self.declare_parameter("stop_thres", float(0.1))
        self.declare_parameter("max_lin_vel", float(0.2))
        self.declare_parameter("safe_lin_vel", float(0.2))
        self.declare_parameter("max_ang_vel", float(2.0))
        self.declare_parameter("turn_dist", float(0.2))
        self.declare_parameter("K_p", float(0.1))
        self.declare_parameter("K_i", float(0.0))
        self.declare_parameter("K_d", float(0.0))
        self.declare_parameter("sigmoid_k", float(5.0))
        self.declare_parameter("sigmoid_c", float(3))
        self.declare_parameter("startup_threshold", float(0.1))
       
        # Parameters: Get Values
        self.frequency_ = self.get_parameter("frequency").value
        self.lookahead_distance_ = self.get_parameter("lookahead_distance").value
        self.lookahead_lin_vel_ = self.get_parameter("lookahead_lin_vel").value
        self.stop_thres_ = self.get_parameter("stop_thres").value
        self.max_lin_vel_ = self.get_parameter("max_lin_vel").value
        self.safe_lin_vel_ = self.get_parameter("safe_lin_vel").value
        self.max_ang_vel_ = self.get_parameter("max_ang_vel").value
        self.turn_dist_ = self.get_parameter("turn_dist").value
        self.K_p_ = self.get_parameter("K_p").value
        self.K_i_ = self.get_parameter("K_i").value
        self.K_d_ = self.get_parameter("K_d").value
        self.sigmoid_k_ = self.get_parameter("sigmoid_k").value
        self.sigmoid_c_ = self.get_parameter("sigmoid_c").value
        self.startup_threshold_ = self.get_parameter("startup_threshold").value
        self.current_lin_vel = 0.0

        # Handles: Topic Subscribers
        self.sub_path_ = self.create_subscription(
            Path,
            "path",
            self.callbackSubPath_,
            10,
        )

        self.sub_odom_ = self.create_subscription(
            Odometry,
            "odom",
            self.callbackSubOdom_,
            10,
        )

        # Handles: Topic Publishers
        self.pub_cmd_vel_ = self.create_publisher(
            TwistStamped, 
            "cmd_vel", 
            10,
        )

        self.pub_lookahead_ = self.create_publisher(
            PoseStamped, 
            "lookahead", 
            10,
        )

        # Handles: Timers
        self.timer = self.create_timer(1.0 / self.frequency_, self.callbackTimer_)

        # Other Instance Variables
        self.received_odom_ = False
        self.received_path_ = False
        self.reach_goal_ = False
        self.startup_ = True
        self.lookahead_idx_ = 0

        # Instance PID Variables
        self.integral = 0
        self.time = 0
        self.prev_err = 0

        # Instance Sigmoid Controller Variables
        self.full_lookahead_dist_ = 0

    # Callbacks =============================================================
    
    # Path subscriber callback
    def callbackSubPath_(self, msg: Path):
        if len(msg.poses) == 0:  # not msg.poses is fine but not clear
            self.get_logger().warn(f"Received path message is empty!")
            return  # do not update the path if no path is returned. This will ensure the copied path contains at least one point when the first non-empty path is received.

        self.path_poses_ = msg.poses
        self.turn_boundaries = process_paths(msg, self.turn_dist_)

        self.reset_()
        self.received_path_ = True

    # Odometry subscriber callback
    def callbackSubOdom_(self, msg: Odometry):
        self.rbt_x_ = msg.pose.pose.position.x
        self.rbt_y_ = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.rbt_yaw_ = atan2(2*(q.w*q.z+q.x*q.y),(1-2*(q.y*q.y + q.z*q.z)))
        
        self.received_odom_ = True

    # Gets the lookahead point's coordinates based on the current robot's position and planner's path
    # Make sure path and robot positions are already received, and the path contains at least one point.
    def getLookaheadPoint_(self):
        # Find the point along the path that is closest to the robot

        # From the closest point, iterate towards the goal and find the first point that is at least a lookahead distance away.
        # Return the goal point if no such lookahead point can be found
        found_point = False
        while self.lookahead_idx_ < len(self.path_poses_):
            # Get the lookahead coordinates
            lookahead_pose = self.path_poses_[self.lookahead_idx_]
            lookahead_x = lookahead_pose.pose.position.x
            lookahead_y = lookahead_pose.pose.position.y
            final_pose = self.path_poses_[len(self.path_poses_)-1]
            self.goal_x_ = final_pose.pose.position.x
            self.goal_y_ = final_pose.pose.position.y

            distance = hypot(lookahead_x - self.rbt_x_, lookahead_y - self.rbt_y_)
            if distance > self.lookahead_distance_:
                found_point = True
                break

            self.lookahead_idx_ += 1

        if found_point == False:
            lookahead_x = self.path_poses_[-1].pose.position.x
            lookahead_y = self.path_poses_[-1].pose.position.y

        # Publish the lookahead coordinates
        msg_lookahead = PoseStamped()
        msg_lookahead.header.stamp = self.get_clock().now().to_msg()
        msg_lookahead.header.frame_id = "map"
        msg_lookahead.pose.position.x = lookahead_x
        msg_lookahead.pose.position.y = lookahead_y
        self.pub_lookahead_.publish(msg_lookahead)

        # Return the coordinates
        return lookahead_x, lookahead_y
    
    def getBoundaryLookaheadPoint_(self):
        # Boundary lookahead point determined by whenever the robot crosses the boundary for the next point in line
        old_idx = self.lookahead_idx_ 

        if self.reach_goal_ == False:
            # Keeping going through the boundaries until we find one that we haven't crossed yet
            while (self.turn_boundaries[self.lookahead_idx_].has_crossed_line(self.rbt_x_, self.rbt_y_)):
                self.lookahead_idx_ += 1
                if self.lookahead_idx_ == (len(self.path_poses_) - 1):
                    self.lookahead_idx_ = -1
                    self.reach_goal_ = True
                    break
        
        # Currently display point at which it starts turning, early to reaching the point
        msg_lookahead = self.path_poses_[self.lookahead_idx_ + 1]
        msg_lookahead.header.stamp = self.get_clock().now().to_msg()
        msg_lookahead.header.frame_id = "map"
        self.pub_lookahead_.publish(msg_lookahead)

        # If we have switched to a new lookahead point
        if old_idx != self.lookahead_idx_:
            self.integral = 0
            self.time = 0
            self.prev_err = 0
            self.full_lookahead_dist_ = hypot(msg_lookahead.pose.position.x - self.rbt_x_,
                                             msg_lookahead.pose.position.y - self.rbt_y_)

        #target_lookahead = self.path_poses_[self.lookahead_idx_]
        return msg_lookahead.pose.position.x, msg_lookahead.pose.position.y

    def getAdaptiveLookaheadPoint_(self):
        # Adaptive lookahead distance based on speed and curvature
        # Base lookahead distance adjusted by current speed
        minLookahead_distance = 0.2
        constantTime = 1.0
        base_lookahead = self.lookahead_distance_
        #speed_factor = max(minLookahead_distance, min(self.lookahead_distance_, self.current_lin_vel / self.max_lin_vel_))
        adaptive_lookahead = minLookahead_distance + constantTime * abs(self.current_lin_vel)
        speed_factor = max(minLookahead_distance, min(self.lookahead_distance_, minLookahead_distance + constantTime*self.current_lin_vel))
        #adaptive_lookahead = base_lookahead * speed_factor
        adaptive_lookahead = max(minLookahead_distance, min(1.0, adaptive_lookahead))

        # Find the closest point to the robot first
        closest_idx = 0
        closest_distance = float('inf')

        for i in range(len(self.path_poses_)):
            distance = hypot(self.path_poses_[i].pose.position.x - self.rbt_x_, 
                            self.path_poses_[i].pose.position.y - self.rbt_y_)
            if distance < closest_distance:
                closest_distance = distance
                closest_idx = i

        # Start searching from the closest point forward
        lookahead_idx = closest_idx
        found_point = False

        while lookahead_idx < len(self.path_poses_):
            lookahead_pose = self.path_poses_[lookahead_idx]
            lookahead_x = lookahead_pose.pose.position.x
            lookahead_y = lookahead_pose.pose.position.y

            distance = hypot(lookahead_x - self.rbt_x_, lookahead_y - self.rbt_y_)

            # Adaptive threshold - allow some tolerance
            if distance >= adaptive_lookahead * 0.8:  # 80% of adaptive lookahead
                found_point = True
                break

            lookahead_idx += 1

        # If no suitable point found, use the goal
        if not found_point or lookahead_idx >= len(self.path_poses_):
            lookahead_idx = len(self.path_poses_) - 1
            lookahead_x = self.path_poses_[lookahead_idx].pose.position.x
            lookahead_y = self.path_poses_[lookahead_idx].pose.position.y
            
        else:
            # Store for next iteration
            self.lookahead_idx_ = lookahead_idx

        # Additional adaptation: if we're close to goal, reduce lookahead
        #goal_distance = hypot(self.path_poses_[lookahead_idx].pose.position.x - self.rbt_x_, self.path_poses_[lookahead_idx].pose.position.y - self.rbt_y_)
        #if goal_distance < self.stop_thres_:  # When within 3x stopping threshold of goal
        #    lookahead_x = self.path_poses_[lookahead_idx].pose.position.x
        #    lookahead_y = self.path_poses_[lookahead_idx].pose.position.y

        # Publish the lookahead coordinates
        msg_lookahead = PoseStamped()
        msg_lookahead.header.stamp = self.get_clock().now().to_msg()
        msg_lookahead.header.frame_id = "map"
        msg_lookahead.pose.position.x = lookahead_x
        msg_lookahead.pose.position.y = lookahead_y
        self.pub_lookahead_.publish(msg_lookahead)

        # Store current linear velocity for next adaptation
       # self.current_lin_vel = getattr(self, 'current_lin_vel', 0.0)

        return lookahead_x, lookahead_y

    # Implement the pure pursuit controller here
    def callbackTimer_(self):
        if not self.received_odom_ or not self.received_path_:
            return  # return silently if path or odom is not received.


        # get lookahead point
        lookahead_x, lookahead_y = self.getBoundaryLookaheadPoint_()
        #lookahead_x, lookahead_y = self.getAdaptiveLookaheadPoint_()

        if self.startup_:
            lin_vel = 0.0
            target_angle = atan2(lookahead_y - self.rbt_y_, lookahead_x - self.rbt_x_)

            if abs(target_angle - self.rbt_yaw_) < self.startup_threshold_:
                ang_vel = 0.0
                self.startup_ = False
                self.full_lookahead_dist_ = hypot(self.path_poses_[1].pose.position.x - self.rbt_x_,
                                                  self.path_poses_[1].pose.position.y - self.rbt_y_)
            elif abs(target_angle - self.rbt_yaw_) > pi:
                if target_angle < -(pi/2):
                    ang_vel = self.angle_PID(target_angle + 2*pi, self.rbt_yaw_)
                else:
                    ang_vel = self.angle_PID(target_angle, self.rbt_yaw_ + 2*pi)
            else:
                ang_vel = self.angle_PID(target_angle, self.rbt_yaw_)
        else:
            # get distance to lookahead point (not to be confused with lookahead_distance)
            distance = hypot(lookahead_x - self.rbt_x_, lookahead_y - self.rbt_y_)
            # stop the robot if close to the point.
            if self.reach_goal_:
                lin_vel = 0.0
                ang_vel = 0.0
                msg_cmd_vel = TwistStamped()
                msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
                msg_cmd_vel.twist.linear.x = lin_vel
                msg_cmd_vel.twist.angular.z = ang_vel
                self.pub_cmd_vel_.publish(msg_cmd_vel)
                self.current_lin_vel = lin_vel
                return   
            else:
                # angle PID controller
                target_angle = atan2(lookahead_y - self.rbt_y_, lookahead_x - self.rbt_x_)
            
                if abs(target_angle - self.rbt_yaw_) > pi:
                    if target_angle < -(pi/2):
                        ang_vel = self.angle_PID(target_angle + 2*pi, self.rbt_yaw_)
                    else:
                        ang_vel = self.angle_PID(target_angle, self.rbt_yaw_ + 2*pi)
                else:
                    ang_vel = self.angle_PID(target_angle, self.rbt_yaw_)
            
            # linear speed controller
            # lin_vel = self.speed_sigmoid_controller(lookahead_x, lookahead_y)
            lin_vel = self.double_sigmoid_controller(lookahead_x, lookahead_y)
            self.current_lin_vel = lin_vel
        
        # saturate velocities. The following can result in the wrong curvature,
        # but only when the robot is travelling too fast (which should not occur if well tuned).
        # lin_vel = min(lin_vel, self.max_lin_vel_)
        if ang_vel < 0:
            ang_vel = max(ang_vel, -self.max_ang_vel_)
        else:
            ang_vel = min(ang_vel, self.max_ang_vel_)

        # publish velocities
        msg_cmd_vel = TwistStamped()
        msg_cmd_vel.header.stamp = self.get_clock().now().to_msg()
        msg_cmd_vel.twist.linear.x = lin_vel
        msg_cmd_vel.twist.angular.z = ang_vel
        self.pub_cmd_vel_.publish(msg_cmd_vel)


    def angle_PID(self, target, current):

        err = target - current

        if self.time == 0:
            dt = 0.0
            derivative = 0
        else:
            dt = self.time - time.time()
            derivative = (err - self.prev_err) / dt
        self.time = time.time()

        self.integral += err*dt
        self.prev_err = err

        return err*self.K_p_ + self.integral*self.K_i_ + derivative*self.K_d_
    
    def speed_sigmoid_controller(self, lookahead_x, lookahead_y):
            position_err = hypot(lookahead_x - self.rbt_x_, lookahead_y - self.rbt_y_)
            

            sigmoid_output = 1/(1 + e**(-1*(self.sigmoid_k_) * (position_err)))

            return sigmoid_output * self.safe_lin_vel_    
    
    def double_sigmoid_controller(self, lookahead_x, lookahead_y):

        x = hypot(lookahead_x - self.rbt_x_, lookahead_y - self.rbt_y_)
        return (self.safe_lin_vel_
             + (self.max_lin_vel_ - self.safe_lin_vel_)
             * (1-self.sigmoid(x - self.full_lookahead_dist_ + self.sigmoid_c_))
             * (self.sigmoid(x - self.turn_dist_ - self.sigmoid_c_)))

    def sigmoid(self, x):
        return 1/(1+exp(-1*self.sigmoid_k_*x))

    def reset_(self):
        # PID Instance Variables
        self.integral = 0
        self.time = 0
        self.prev_err = 0

        # Other Instance Variables
        self.received_odom_ = False
        self.received_path_ = False
        self.reach_goal_ = False
        self.startup_ = True
        self.lookahead_idx_ = 0

# Boundary line before each path point, perpendicular to the line between 
# the point and its predecessor.
# Provides a more accurate and lightweight description of when a robot has passed
# the rthreshold at which it should look to the next point 
class Line():
    def __init__(self, point_on_line: PoseStamped, point_perp: PoseStamped):
        vertical_line_gradient = 1.e5

        dx = point_on_line.pose.position.x - point_perp.pose.position.x
        dy = point_on_line.pose.position.y - point_perp.pose.position.y

        if dx == 0:
            self.perp_gradient = vertical_line_gradient
        else:
            self.perp_gradient = dy / dx
        
        if self.perp_gradient == 0:
            self.gradient = vertical_line_gradient
        else:
            self.gradient = -1 / self.perp_gradient

        self.y_intercept = point_on_line.pose.position.y - self.gradient*point_on_line.pose.position.x

        # Keep two points on the line saved
        self.point1 = point_on_line
        self.point2 = PoseStamped()
        self.point2.pose.position.x = point_on_line.pose.position.x + 1
        self.point2.pose.position.y = point_on_line.pose.position.y + self.gradient

        self.approach_side = self.get_side(point_perp.pose.position.x, point_perp.pose.position.y)
    
    # Returns true if the point (x, y) is on on side of the line and false if it is on the other side
    def get_side(self, x, y):
        return (x-self.point1.pose.position.x)*(self.point2.pose.position.y-self.point1.pose.position.y) > (y-self.point1.pose.position.y)*(self.point2.pose.position.x-self.point1.pose.position.x)
    
    # Determines if a specific robot position is beyond the boundary line
    def has_crossed_line(self, x, y):
        return self.get_side(x, y) != self.approach_side

# From a set of path poses, create a list of boundary lines a distance of 
# turn_dist away from their associated path point
def process_paths(way_points, turn_dist):
    finish_line_idx = len(way_points.poses) - 1
    turn_boundaries = []


    previous_point = way_points.poses[0]
    for i in range(1, len(way_points.poses)):

        current_point = way_points.poses[i]

        dir_to_curr_point = [current_point.pose.position.x - previous_point.pose.position.x, current_point.pose.position.y - previous_point.pose.position.y]
        d = hypot(dir_to_curr_point[0], dir_to_curr_point[1])

        dir_to_curr_point = [dir_to_curr_point[0]/d, dir_to_curr_point[1]/d]

        turn_boundary_point = PoseStamped()
        if i == finish_line_idx:
            turn_boundary_point = current_point
        else:
            turn_boundary_point.pose.position.x = current_point.pose.position.x - dir_to_curr_point[0]*turn_dist
            turn_boundary_point.pose.position.y = current_point.pose.position.y - dir_to_curr_point[1]*turn_dist
        
        point_perpendicular = PoseStamped()
        point_perpendicular.pose.position.x = previous_point.pose.position.x - dir_to_curr_point[0]*turn_dist
        point_perpendicular.pose.position.y = previous_point.pose.position.y - dir_to_curr_point[1]*turn_dist
        turn_boundaries.append(Line(turn_boundary_point, point_perpendicular))

        previous_point = turn_boundary_point
    
    return turn_boundaries        

# Main Boiler Plate =============================================================
def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Controller())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    