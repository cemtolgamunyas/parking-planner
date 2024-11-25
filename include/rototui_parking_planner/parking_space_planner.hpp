#ifndef ROTOTUI_PARKING_PLANNER__PARKING_SPACE_PLANNER_NODE_HPP_
#define ROTOTUI_PARKING_PLANNER__PARKING_SPACE_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/rclcpp.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <tier4_planning_msgs/msg/scenario.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_adapi_v1_msgs/msg/vehicle_dimensions.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>
#include <autoware_adapi_v1_msgs/srv/get_vehicle_dimensions.hpp>
#include <rototui_interfaces/srv/parking_trigger.hpp>

#include <autoware/route_handler/route_handler.hpp>

#include <tf2/LinearMath/Quaternion.h>

// NEWLY ADDED THINGS FROM FREESPACE PLANNER NODE

#include <autoware/freespace_planning_algorithms/astar_search.hpp>
#include <autoware/freespace_planner/freespace_planner_node.hpp>
using AbstractPlanningAlgorithm = autoware::freespace_planning_algorithms::AbstractPlanningAlgorithm;
using AstarParam = autoware::freespace_planning_algorithms::AstarParam;
using AstarSearch = autoware::freespace_planning_algorithms::AstarSearch;
using PlannerCommonParam = autoware::freespace_planning_algorithms::PlannerCommonParam;
using VehicleShape = autoware::freespace_planning_algorithms::VehicleShape;
// using NodeParam = autoware::freespace_planner::NodeParam;





#include <deque>
#include <utility>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <future>
#include <chrono>



using std::async;
using std::string;
using std::list;
using lanelet::utils::to2D;
using lanelet::utils::query::getLinkedParkingLot;
using lanelet::utils::query::getLinkedParkingSpaces;
using lanelet::utils::query::getAllParkingSpaces;


using Odometry = nav_msgs::msg::Odometry;
using OdometryPtr = nav_msgs::msg::Odometry::ConstSharedPtr;
using GoalPose = geometry_msgs::msg::PoseStamped;
using Twist = geometry_msgs::msg::TwistStamped::ConstSharedPtr;
using Position2D = lanelet::BasicPoint2d;
using Polygon3D = lanelet::ConstPolygon3d;
using Linestring3D = lanelet::ConstLineString3d;
using Linestrings3D = lanelet::ConstLineStrings3d;
using Map = autoware_map_msgs::msg::LaneletMapBin;
using LaneletMap = std::shared_ptr<lanelet::LaneletMap>;
using Route = std::shared_ptr<lanelet::routing::RoutingGraph>;
using Traffic = std::shared_ptr<lanelet::traffic_rules::TrafficRules>;

using RouteHandling = std::shared_ptr<autoware::route_handler::RouteHandler>;
using TwistBuffer = std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr>;
using PredictedObjects = autoware_perception_msgs::msg::PredictedObjects;

using MapList = std::map<int,list<double>>;
using ParkingStatusFuture = std::future<bool>;

using ChangeOpMode = autoware_adapi_v1_msgs::srv::ChangeOperationMode;
using VehicleDimensions = autoware_adapi_v1_msgs::srv::GetVehicleDimensions;
using ParkingTrigger = rototui_interfaces::srv::ParkingTrigger;

using namespace std::placeholders;




// struct NodeParam
// {
//     std::string planning_algorithm;
//     double waypoints_velocity;  // constant velocity on planned waypoints [km/h]
//     double update_rate;         // replanning and publishing rate [Hz]
//     double th_arrived_distance_m;
//     double th_stopped_time_sec;
//     double th_stopped_velocity_mps;
//     double th_course_out_distance_m;  // collision margin [m]
//     double th_obstacle_time_sec;
//     double vehicle_shape_margin_m;
//     bool replan_when_obstacle_found;
//     bool replan_when_course_out;
// };


class ParkingSpacePlannerNode : public rclcpp::Node
{
public:
    ParkingSpacePlannerNode();


private:

    // Parameters
    bool lot_found;
    bool space_found;
    bool goal_found;
    bool map_received_;
    bool stop_status;
    bool odometry_received_;
    bool response_check;
    bool isObjectInParkingSpace;

    int count;
    int functions_;
    int wait_the_function;

    float left_overhang;
    float right_overhang;
    float front_overhang;
    float rear_overhang;
    float wheel_base;
    float wheel_radius;
    float wheel_width;
    float wheel_tread;
    float height;

    // Specialized Params
    // NodeParam node_param_;
    VehicleShape vehicle_shape_;



    // Subscriptions
    rclcpp::Subscription<Map>::SharedPtr sub_lanelet_map_;
    rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<PredictedObjects>::SharedPtr sub_predicted_object_;

    // Publishers
    rclcpp::Publisher<GoalPose>::SharedPtr pub_goal_pose_;

    // Services
    rclcpp::Service<ParkingTrigger>::SharedPtr srv_parking_;

    // Service Clients
    rclcpp::Client<ChangeOpMode>::SharedPtr srv_client_autonomous_;
    rclcpp::Client<VehicleDimensions>::SharedPtr srv_client_vehicle_;
    rclcpp::Client<ParkingTrigger>::SharedPtr srv_client_parking_;



    Polygon3D getNearestParkingLot(const Position2D & current_position,
                                   const LaneletMap & lanelet_map_ptr);

    
    Linestring3D getNearestParkingSpace(const Position2D & current_position,
                                        const Polygon3D & nearest_parking_lot, 
                                        const LaneletMap & lanelet_map_ptr);


    Linestring3D findNewNearestSpace(MapList my_map, const Position2D & current_position);

    // Calculating Goal Pose In Front of the Parking Space
    GoalPose calculateGoalPose(const Linestring3D & parking_space);
    // Calculating Goal Pose Inside the Parking Space
    GoalPose calculateParkingGoalPose(const Linestring3D & parking_space);



    void onOdom(const OdometryPtr msg);
    void onMap(const Map::ConstSharedPtr msg);
    bool isStopped(const TwistBuffer & twist_buffer,
                   const double th_stopped_velocity_mps);
    void checkStatus(const OdometryPtr msg);
    void objectPrediction(const PredictedObjects::ConstSharedPtr msg); 
    void setStatusOfParkingSpace();
    void printParkingSpaces();
    void wait_for_service();
    


    // Callback Functions

    // Service - Client for Changing Operation Mode
    void cb_srv_change_operation_mode_();
    void cb_handle_operation_mode_response_(rclcpp::Client<ChangeOpMode>::SharedFuture future);

    // Service - Client for Getting Vehicle Dimensions
    void cb_srv_get_vehicle_dimensions_();
    void cb_handle_vehicle_dimensions_response_(rclcpp::Client<VehicleDimensions>::SharedFuture future);

    // // Service - Client for Parking Callback
    void cb_srv_execute_parking_operation_(const std::shared_ptr<ParkingTrigger::Request> request,
                                           const std::shared_ptr<ParkingTrigger::Response> response);

    void cb_handle_parking_operation_response_(rclcpp::Client<ParkingTrigger>::SharedFuture future);




    // Messages
    // std::map<int,list<double>> my_parking_spaces_ ; 
    // std::future<bool> future_ParkingStatus;
    // nav_msgs::msg::Odometry::ConstSharedPtr current_pose_;
    // nav_msgs::msg::Odometry::ConstSharedPtr new_current_pose_;
    // geometry_msgs::msg::PoseStamped goal_pose_;
    // geometry_msgs::msg::TwistStamped::ConstSharedPtr twist_;
    // lanelet::BasicPoint2d current_position_;
    // lanelet::BasicPoint2d goal_position_;
    // lanelet::BasicPoint2d new_check_;
    // lanelet::ConstPolygon3d nearest_lot_;
    // lanelet::ConstLineString3d nearest_space_;
    // lanelet::ConstLineString3d new_space_;
    // lanelet::ConstLineStrings3d linked_parking_spaces_for_checking; 
    // autoware_auto_perception_msgs::msg::PredictedObjects::ConstSharedPtr objectMsg;
    // std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;  
    // std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
    // std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
    // std::shared_ptr<route_handler::RouteHandler> route_handler_;
    // std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> twist_buffer_;



    // Messages with new theme
    OdometryPtr current_pose_;
    OdometryPtr new_current_pose_;
    GoalPose goal_pose_;
    Twist twist_;
    TwistBuffer twist_buffer_;
    Position2D current_position_;
    Position2D goal_position_;
    Position2D new_check_;
    Polygon3D nearest_lot_;
    Linestring3D nearest_space_;
    Linestring3D new_space_;
    Linestrings3D linked_parking_spaces_for_checking;
    LaneletMap lanelet_map_ptr_;
    Route routing_graph_ptr_;
    Traffic traffic_rules_ptr_;
    RouteHandling route_handler_;
    PredictedObjects objectMsg;
    MapList my_parking_spaces_;
    ParkingStatusFuture future_ParkingStatus;

    std::unique_ptr<AbstractPlanningAlgorithm> algo_;




};



#endif // ROTOTUI_PARKING_PLANNER__PARKING_SPACE_PLANNER_NODE_HPP_