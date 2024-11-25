#include "rototui_parking_planner/parking_space_planner.hpp"


ParkingSpacePlannerNode::ParkingSpacePlannerNode() : Node("parking_space_planner_node") 

{

    // Subscriptions
    sub_lanelet_map_ = this->create_subscription<Map>("/map/vector_map",
            rclcpp::QoS{1}.transient_local(), std::bind(&ParkingSpacePlannerNode::onMap, this, _1));

    sub_odom_ = this->create_subscription<Odometry>("/localization/kinematic_state",
            rclcpp::QoS{100}, std::bind(&ParkingSpacePlannerNode::onOdom, this, _1));

    // sub_predicted_object_ = this->create_subscription<PredictedObjects>("/perception/object_recognition/objects",
    //         rclcpp::QoS{1}, std::bind(&ParkingSpacePlannerNode::objectPrediction, this, _1));


    // Publishers
    pub_goal_pose_ = this->create_publisher<GoalPose>("/planning/mission_planning/goal", rclcpp::QoS{1});

    // // Services 
    srv_parking_ = create_service<ParkingTrigger>("call_parking_planner",               // (A service for triggering the executable node)
            std::bind(&ParkingSpacePlannerNode::cb_srv_execute_parking_operation_, this, _1, _2));

    // Clients
    srv_client_autonomous_ = this->create_client<ChangeOpMode>("/api/operation_mode/change_to_autonomous");
    srv_client_vehicle_ = this->create_client<VehicleDimensions>("/api/vehicle/dimensions");
    srv_client_parking_ = create_client<ParkingTrigger>("call_parking_planner");     // Create a parking operation client for calling the executable

    // Parameters
    lot_found = false;
    space_found = false;
    goal_found = false;
    map_received_ = false;  
    stop_status = false;
    odometry_received_ = false;
    response_check = false;
    isObjectInParkingSpace = false;

    count = 0;
    functions_ = 0;
    wait_the_function = 3;
    left_overhang = 0.0;
    right_overhang = 0.0;
    front_overhang = 0.0;
    rear_overhang = 0.0;
    wheel_base = 0.0;
    wheel_radius = 0.0;
    wheel_width = 0.0;
    wheel_tread = 0.0;
    height = 0.0;


};





Polygon3D ParkingSpacePlannerNode::getNearestParkingLot(const Position2D & current_position,
                                                        const LaneletMap & lanelet_map_ptr)
{
    auto all_parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map_ptr);
    Polygon3D nearest_lot;
    double min_distance = std::numeric_limits<double>::max();

    for (const auto & lot : all_parking_lots)
    {
        double distance = boost::geometry::distance(current_position, to2D(lot).basicPolygon());
        if (distance < min_distance)
        {
            min_distance = distance;
            nearest_lot = lot;
            lot_found = true;
        }
    }

    if (lot_found)
    {
        RCLCPP_INFO(this->get_logger(), "Nearest Lot is found on the Map!");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "There are no parking lots nearby!");
    }

    return nearest_lot;
}



Linestring3D ParkingSpacePlannerNode::getNearestParkingSpace(const Position2D & current_position,
                                                             const Polygon3D & nearest_parking_lot, 
                                                             const LaneletMap & lanelet_map_ptr)
{
    auto all_parking_spaces = getAllParkingSpaces(lanelet_map_ptr);
    auto linked_parking_spaces = getLinkedParkingSpaces(nearest_parking_lot, all_parking_spaces);
    linked_parking_spaces_for_checking = linked_parking_spaces;

    Linestring3D nearest_space;
    double min_distance = std::numeric_limits<double>::max();

    for (const auto & parking_space : linked_parking_spaces)
    {
        double distance = boost::geometry::distance(current_position, to2D(parking_space).basicLineString());
        if (distance < min_distance)
        {
            std::list<double> list = {parking_space.front().x(), parking_space.front().y(), parking_space.back().x(), parking_space.back().y(), 0.0};
            my_parking_spaces_[count] = list;
            count++;
            min_distance = distance;
            nearest_space = parking_space;
            space_found = true;
        }
    }

    if (space_found)
    {
        RCLCPP_INFO(this->get_logger(), "The Closest Parking Space is found on the Map!");
        printParkingSpaces();       // Print all found parking spaces
    }

    else
    {
        RCLCPP_INFO(this->get_logger(), "There are no parking spaces nearby!");
    }

    return nearest_space;
}




void ParkingSpacePlannerNode::onOdom(const OdometryPtr msg)
{
    if ( (map_received_ == false) || (msg == nullptr) )
    {
        RCLCPP_ERROR(this->get_logger(), "The Localization couldn't get the current position!");
        return;
    }

    current_pose_ = msg;
    current_position_.x() = current_pose_->pose.pose.position.x;
    current_position_.y() = current_pose_->pose.pose.position.y;


    switch (functions_)
    {
    case 0:
        cb_srv_get_vehicle_dimensions_();
        nearest_lot_ = getNearestParkingLot(current_position_, lanelet_map_ptr_);

        if (lot_found)
        {
            nearest_space_ = getNearestParkingSpace(current_position_, nearest_lot_, lanelet_map_ptr_);
        }
        if (space_found)
        {
            functions_ = 1;
        }
        break;

    case 1:
        if (space_found)
        {
            goal_pose_ = calculateGoalPose(nearest_space_);
            pub_goal_pose_->publish(goal_pose_);
            goal_found = true;
            RCLCPP_INFO_STREAM(this->get_logger(), "Parking Space is detected, ready to reach in front of it...");
            RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for trajectory...");

            sleep(2);
            functions_ = 2;
        }
        break;

    case 2:
        // When the nearest parking space is spotted and vehicle has stopped in front of it, check for obstacle inside
        cb_srv_change_operation_mode_();
        checkStatus(msg);

        if (stop_status)
        {
            RCLCPP_INFO(this->get_logger(), "Stop Status is active");
            RCLCPP_INFO(this->get_logger(), "Autonomous Park Completed!");

            functions_ = 3;
        }
        break;

    case 3:
    
        // Subscribe to predicted objects module to check the availability inside
        sub_predicted_object_ = this->create_subscription<PredictedObjects>("/perception/object_recognition/objects",
                rclcpp::QoS{1}, std::bind(&ParkingSpacePlannerNode::objectPrediction, this, _1));
        break;

    case 4:
        setStatusOfParkingSpace();
        break;
    
    case 5:
        nearest_space_ = findNewNearestSpace(my_parking_spaces_, current_position_);
        if (space_found == true)
        {
            RCLCPP_INFO(this->get_logger(), "New Nearest Parking Space Start x = %f", nearest_space_.front().x());
            RCLCPP_INFO(this->get_logger(), "New Nearest Parking Space Start y = %f", nearest_space_.front().y());
            RCLCPP_INFO(this->get_logger(), "New Nearest Parking Space End x = %f", nearest_space_.back().x());
            RCLCPP_INFO(this->get_logger(), "New Nearest Parking Space End y = %f", nearest_space_.back().y());
        
            functions_ = 1;
        }
        break;

    // case 6:
    //     goal_pose_ = calculateParkingGoalPose(nearest_space_);
        
    //     if (goal_found)
    //     {
    //         pub_goal_pose_->publish(goal_pose_);       // Publish the parking goal pose when vehicle is stopped in front of the parking space
    //         sleep(1);
    //         functions_ = 7;
    //     }
    //     break;

    // case 7:
        
    //     cb_srv_change_operation_mode_();               // Change operation mode to autonomous after determining the goal pose inside the parking space
    //     checkStatus(msg);
    //     if (stop_status)
    //     {
    //         RCLCPP_INFO(this->get_logger(), "Autonomous Park Completed!");
    //         functions_ = 8;
    //     }
    //     break;

    default:
        break;
    }

}



void ParkingSpacePlannerNode::onMap(const Map::ConstSharedPtr msg)
{
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(*msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
    // route_handler_ = std::make_shared<RouteHandler>(*msg)

    route_handler_ = std::make_shared<autoware::route_handler::RouteHandler>(*msg);
    RCLCPP_INFO(this->get_logger(), "The Lanelet Map is alive and running!");
    map_received_ = true;
}



bool ParkingSpacePlannerNode::isStopped(const TwistBuffer & twist_buffer,
                                        const double th_stopped_velocity_mps)
{
    for (const auto & twist : twist_buffer)
    {
        if (std::abs(twist->twist.linear.x) > th_stopped_velocity_mps)
        {
            return false;
        }
    }
    RCLCPP_INFO(this->get_logger(), "STOP");
    RCLCPP_INFO(this->get_logger(), "Current Position Orientation: z = %f, w = %f", current_pose_->pose.pose.orientation.z, current_pose_->pose.pose.orientation.w);

    goal_found = false;
    srv_client_autonomous_ = nullptr;

    return true;
}



void ParkingSpacePlannerNode::checkStatus(const OdometryPtr msg)
{
    new_current_pose_ = msg;
    goal_position_.x() = goal_pose_.pose.position.x;
    goal_position_.y() = goal_pose_.pose.position.y;

    new_check_.x() = new_current_pose_->pose.pose.position.x;
    new_check_.y() = new_current_pose_->pose.pose.position.y;
    stop_status = false;

    double check_distance = boost::geometry::distance(new_check_, goal_position_);

    if (check_distance < 1.0)
    {
        auto twist = std::make_shared<geometry_msgs::msg::TwistStamped>();
        twist->header = msg->header;
        twist->twist = msg->twist.twist;

        twist_ = twist;
        twist_buffer_.push_back(twist);

        while(true)
        {
            const auto time_diff = rclcpp::Time(msg->header.stamp) - rclcpp::Time(twist_buffer_.front()->header.stamp);
            if (time_diff.seconds() < 0.01)
            {
                break;
            }
            twist_buffer_.pop_front();
        }
        // might need return stop_status here
        stop_status = isStopped(twist_buffer_, 0.01);

    }
}


void ParkingSpacePlannerNode::objectPrediction(const PredictedObjects::ConstSharedPtr msg)
{
    int msg_controller = 0;
    sub_predicted_object_ = nullptr;

    // Coordinates of the nearest parking space linestring
    double space_start_x = nearest_space_.front().x();
    double space_start_y = nearest_space_.front().y();
    double space_end_x = nearest_space_.back().x();
    double space_end_y = nearest_space_.back().y();

    // Math for shifting operation both to the left and right hand side of the nearest parking space
    Eigen::Vector2d direction(space_end_x - space_start_x, space_end_y - space_start_y);
    direction.normalize();
    Eigen::Vector2d shift_right = 2.0 * direction;        // Assumed RHS distance is 2 meters
    Eigen::Vector2d shift_left = -2.0 * direction;        // Assumed LHS distance is 2 meters
          
    // Coordinates of the RHS shifted linestring
    double right_line_start_x = space_start_x + shift_right.x();
    double right_line_start_y = space_start_y + shift_right.y();
    double right_line_end_x = space_end_x + shift_right.x();
    double right_line_end_y = space_end_y + shift_right.y();

    // Coordinates of the LHS shifted linestring
    double left_line_start_x = space_start_x + shift_left.x();
    double left_line_start_y = space_start_y + shift_left.y();
    double left_line_end_x = space_end_x + shift_left.x();
    double left_line_end_y = space_end_y + shift_left.y();

    double x_min = std::min( {left_line_start_x, left_line_end_x, right_line_start_x, right_line_end_x} );
    double x_max = std::max( {left_line_start_x, left_line_end_x, right_line_start_x, right_line_end_x} );
    double y_min = std::min( {left_line_start_y, left_line_end_y, right_line_start_y, right_line_end_y} );
    double y_max = std::max( {left_line_start_y, left_line_end_y, right_line_start_y, right_line_end_y} );

    RCLCPP_INFO(this->get_logger(), "Checking the Parking Status");

    for (const auto& predicted_object : msg->objects)
    {
        msg_controller = 1;
        isObjectInParkingSpace = false;

        double object_position_x = predicted_object.kinematics.initial_pose_with_covariance.pose.position.x;
        double object_position_y = predicted_object.kinematics.initial_pose_with_covariance.pose.position.y;

        RCLCPP_INFO(this->get_logger(), "Predicted Object Position x = %f, y = %f", object_position_x, object_position_y);

        if ( (x_min < object_position_x) && (object_position_x < x_max) && (y_min < object_position_y) && (object_position_y < y_max) )
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "An Obstacle is detected inside the Parking Space!");
            functions_ = 4;
            isObjectInParkingSpace = true;
            break;
        }
        else
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "Parking Space is Available for Parking!");
            functions_ = 6;
            isObjectInParkingSpace = false;
        }
    }

    if (msg_controller == 0)
    {
        RCLCPP_INFO(this->get_logger(), "msg_controller = 0, There is no object near here.");
        functions_ = 6;
    }

}



void ParkingSpacePlannerNode::setStatusOfParkingSpace()
{
    double temp_array[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    int c = 0;

    for (const auto& pair : my_parking_spaces_)
    {
        int j = 0;
        list<double> temp_list = pair.second;
        for (const auto& i : temp_list)
        {
            temp_array[j] = i;
            j++;
        }
        if (temp_array[0] == nearest_space_.front().x())
        {
            temp_array[4] = 1.0;
            temp_list.clear();
            temp_list = {temp_array[0], temp_array[1], temp_array[2], temp_array[3], temp_array[4]};
            my_parking_spaces_[c] = temp_list;
            functions_ = 5;
            break;
        }
        else
        {
            c++;
            continue;
        }
    }
}



GoalPose ParkingSpacePlannerNode::calculateGoalPose(const Linestring3D & parking_space)
{
    GoalPose goal_pose;
          
    // Assume parking space is defined by its front and back points
    auto start_point = parking_space.front();
    auto end_point = parking_space.back();
    RCLCPP_INFO(this->get_logger(), "Start Point x = %f, Start Point y =%f, End Point x = %f, End Point y = %f", start_point.x(), start_point.y(), end_point.x(), end_point.y());      
        
    // Calculate distance between front and back of the parking space
    double distance = boost::geometry::distance(end_point, start_point);
    RCLCPP_INFO(this->get_logger(), "Distance between the Front and Back of the Parking Space = %f", distance);

    // Create a 2D Vector from start point to end point and normalize its position and orientation
    Eigen::Vector2d direction(end_point.x() - start_point.x(), end_point.y() - start_point.y());
    Eigen::Vector2d direction_yaw(end_point.x() - start_point.x(), (end_point.y() - start_point.y() ));

    direction.normalize();      // Normalize the direction vector to obtain unit vector
    direction_yaw.normalize();  // Normalize the yaw vector to obtain unit vector


    // Shift vector for normal parking in sample map
    // Eigen::Vector2d shift_vector = 5.0 * direction;
    
    // Shift vector for parallel parking in tofas map
    // Eigen::Vector2d shift_vector = 1.0 * direction;          

    // Mid Point vector of the parking space
    Eigen::Vector2d mid_point((start_point.x() + end_point.x()) / 2.0, (start_point.y() + end_point.y()) / 2.0);

    // auto mid_point_x = (start_point.x() + end_point.x()) / 2.0;
    // auto mid_point_y = (start_point.y() + end_point.y()) / 2.0;

    // goal_pose.pose.position.x = end_point.x() + shift_vector.x();
    // goal_pose.pose.position.y = mid_point_y - shift_vector.y();


    // goal_pose.pose.position.x = mid_point.x() + shift_vector.x();
    // goal_pose.pose.position.y = mid_point.y() + shift_vector.y();
    goal_pose.pose.position.x = mid_point.x();
    goal_pose.pose.position.y = mid_point.y();
    goal_pose.pose.position.z = 0;    // Assume flat ground

    // Calculate the orientation based on the direction of the line segment
    // double yaw = std::atan2(direction_yaw.y(), direction_yaw.x()) + M_PI / 2;
    double yaw = std::atan2(direction_yaw.y(), direction_yaw.x());

    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    q.normalize();

    goal_pose.pose.orientation.x = q.x();
    goal_pose.pose.orientation.y = q.y();
    goal_pose.pose.orientation.z = q.z();
    goal_pose.pose.orientation.w = q.w();

    // Setting frame ID and time stamp
    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->get_clock()->now();

    // Log the calculated goal position
    RCLCPP_INFO(this->get_logger(), "Goal Position: x = %f, y = %f", goal_pose.pose.position.x, goal_pose.pose.position.y);

    // Log the calculated goal orientation
    RCLCPP_INFO(this->get_logger(), "Goal Pose Orientation: x=%f, y=%f, z=%f, w=%f", goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w );

    return goal_pose;
}


void ParkingSpacePlannerNode::printParkingSpaces()
{
    RCLCPP_INFO(this->get_logger(), "Parking Spaces Coordinates: ");

    for (const auto& pair : my_parking_spaces_)
    {
        std::string coordinates_str;
        for (const double coord : pair.second)
        {
            coordinates_str += std::to_string(coord) + " ";
        }
        RCLCPP_INFO(this->get_logger(), "Space %d: %s", pair.first, coordinates_str.c_str());
    }
}


Linestring3D ParkingSpacePlannerNode::findNewNearestSpace(MapList my_map, const Position2D & current_position)
{
    space_found = false;
    double min_distance = std::numeric_limits<double>::max();
    Linestring3D new_nearest_space;
    double end_x_coordinate_check = 0.0;

    for (const auto& pair : my_map)
    {
        int temp_counter = 0;
        double temp_array[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

        for (auto i : pair.second)
        {
            temp_array[temp_counter] = i;
            temp_counter++;
        }
        if (temp_array[4] == 1.0)
        {
            continue;
        }
        double distance = sqrt(pow(temp_array[2] - current_position.x(), 2) + pow(temp_array[3] - current_position.y(), 2));

        if (distance < min_distance)
        {
            min_distance = distance;
            end_x_coordinate_check = temp_array[2];
        }
    }

    for (const auto& parking_space : linked_parking_spaces_for_checking)
    {
        if (end_x_coordinate_check == parking_space.back().x())
        {
            new_nearest_space = parking_space;
            RCLCPP_INFO(this->get_logger(), "The New Nearest Parking Space is Found!");
            printParkingSpaces();
            break;
        }
    }

    space_found = true;
    return new_nearest_space;
}


void ParkingSpacePlannerNode::wait_for_service()
{
    while (!srv_client_autonomous_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service is not available, waiting again...");
    }
}


void ParkingSpacePlannerNode::cb_handle_operation_mode_response_(rclcpp::Client<ChangeOpMode>::SharedFuture future)
{
    auto response = future.get();
    if (response->status.success)
    {
        RCLCPP_INFO_ONCE(this->get_logger(), "Successfully changed the operation mode.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Failed to change the operation mode %s ", response->status.message.c_str());
    }
}

void ParkingSpacePlannerNode::cb_srv_change_operation_mode_()
{
    while (!srv_client_autonomous_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service is not available, waiting again...");
    }

    auto request = std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode_Request>();
    auto result = srv_client_autonomous_->async_send_request(request, std::bind(&ParkingSpacePlannerNode::cb_handle_operation_mode_response_, this, std::placeholders::_1));
    
}


void ParkingSpacePlannerNode::cb_handle_vehicle_dimensions_response_(rclcpp::Client<VehicleDimensions>::SharedFuture future)
{
    auto response = future.get();
    
    if (response->status.success)
    {
        wheel_radius = response->dimensions.wheel_radius;
        wheel_width = response->dimensions.wheel_width;
        wheel_base = response->dimensions.wheel_base;
        wheel_tread = response->dimensions.wheel_tread;
        front_overhang = response->dimensions.front_overhang;   
        rear_overhang = response->dimensions.rear_overhang;
        left_overhang = response->dimensions.left_overhang;
        right_overhang = response->dimensions.right_overhang;
        height = response->dimensions.height;

        RCLCPP_INFO(this->get_logger(), "Successfully accessed the vehicle dimensions.");
        RCLCPP_INFO(this->get_logger(), "Wheel Radius: %f", wheel_radius);
        RCLCPP_INFO(this->get_logger(), "Wheel Width: %f", wheel_width);
        RCLCPP_INFO(this->get_logger(), "Wheel Base: %f", wheel_base);
        RCLCPP_INFO(this->get_logger(), "Wheel Tread: %f", wheel_tread);
        RCLCPP_INFO(this->get_logger(), "Front Overhang: %f", front_overhang);
        RCLCPP_INFO(this->get_logger(), "Rear Overhang: %f", rear_overhang);
        RCLCPP_INFO(this->get_logger(), "Left Overhang: %f", left_overhang);
        RCLCPP_INFO(this->get_logger(), "Right Overhang: %f", right_overhang);
        RCLCPP_INFO(this->get_logger(), "Height: %f", height);

    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Failed to get the vehicle dimensions.");
    }
}

void ParkingSpacePlannerNode::cb_srv_get_vehicle_dimensions_()
{
    while (!srv_client_vehicle_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service is not available, waiting again...");
    }

    auto request = std::make_shared<autoware_adapi_v1_msgs::srv::GetVehicleDimensions_Request>() ;
    auto result = srv_client_vehicle_->async_send_request(request, std::bind(&ParkingSpacePlannerNode::cb_handle_vehicle_dimensions_response_, this, std::placeholders::_1)); 
}



GoalPose ParkingSpacePlannerNode::calculateParkingGoalPose(const Linestring3D & parking_space)
{
    GoalPose goal_pose;
    goal_found = false;

    auto start_point = parking_space.front();
    auto end_point = parking_space.back();
    RCLCPP_INFO(this->get_logger(), "Start Point of the Parking Space: x = %f, y = %f", start_point.x(), start_point.y());
    RCLCPP_INFO(this->get_logger(), "End Point of the Parking Space: x = %f, y = %f", end_point.x(), end_point.y());

    double distance = boost::geometry::distance(end_point, start_point);
    RCLCPP_INFO(this->get_logger(), "The length of the parking space is %f", distance);

    double vehicle_length = front_overhang + wheel_base + rear_overhang;
    double difference = distance - vehicle_length;
    RCLCPP_INFO(this->get_logger(), "The vehicle length is %f", vehicle_length);
    RCLCPP_INFO(this->get_logger(), "Difference between the Parking Space and the Vehicle: %f", difference);

    Eigen::Vector3d direction(end_point.x()-start_point.x(), end_point.y()-start_point.y(), end_point.z()-start_point.z());
    RCLCPP_INFO(this->get_logger(), "Direction Vector: x = %f, y = %f, z = %f", direction.x(), direction.y(), direction.z());


    direction.normalize();          // Normalize the vector to get a unit vector

    Eigen::Vector3d shift = direction * rear_overhang;
    RCLCPP_INFO(this->get_logger(), "Shift Vector: x = %f, y = %f", shift.x(), shift.y());


    // Calculate the midpoint of the parking space
    Eigen::Vector3d midpoint((start_point.x() + end_point.x()) / 2.0, (start_point.y() + end_point.y()) / 2.0, (start_point.z() + end_point.z()) / 2.0);
    RCLCPP_INFO(this->get_logger(), "Midpoint of Parking Space: x = %f, y = %f, z = %f", midpoint.x(), midpoint.y(), midpoint.z());


    // Eigen::Vector3d goal_point = midpoint + 2 * shift;
    Eigen::Vector3d goal_point = midpoint - 2 * shift;

    RCLCPP_INFO(this->get_logger(), "Goal Point: x = %f, y = %f, z = %f", goal_point.x(), goal_point.y(), goal_point.z());

    // Use midpoint as goal pose for the press bos kasa parking
    // Use goal_point as goal pose for normal parking scenario
    goal_pose.pose.position.x = goal_point.x();
    goal_pose.pose.position.y = goal_point.y();
    goal_pose.pose.position.z = 0.0;      // Assume flat ground

    double yaw = std::atan2(end_point.y() - start_point.y(), end_point.x() - start_point.x());
    tf2::Quaternion rot;
    rot.setRPY(0, 0, yaw);                // Roll and pitch are 0 and yaw is calculated

    goal_pose.pose.orientation.x = rot.x();
    goal_pose.pose.orientation.y = rot.y();
    goal_pose.pose.orientation.z = rot.z();
    goal_pose.pose.orientation.w = rot.w();

    goal_pose.header.frame_id = "map";
    goal_pose.header.stamp = this->get_clock()->now();

    RCLCPP_INFO(this->get_logger(), "Parking Goal Position: x = %f, y = %f, z = %f", goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
    RCLCPP_INFO(this->get_logger(), "Parking Goal Pose Orientation: x = %f, y = %f, z = %f, w = %f", goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);
          
    goal_found = true;

    return goal_pose;
}






void ParkingSpacePlannerNode::cb_srv_execute_parking_operation_(const std::shared_ptr<ParkingTrigger::Request> request,
                                                                const std::shared_ptr<ParkingTrigger::Response> response)
{
    RCLCPP_INFO_STREAM(get_logger(), "New Trigger Request received! Status: " << request->trigger);

    if (request->trigger == true)
    {
        RCLCPP_INFO_STREAM(get_logger(), "Parking Planner Trigger is active");
        response->result = response->SUCCESS;
        response->message = "Parking Planner Service has been initialized successfully!";
        cb_srv_change_operation_mode_();
    }
    
    else
    {
        RCLCPP_INFO_STREAM(get_logger(), "Parking Planner Trigger is disabled");
        response->result = response->FAIL;
        response->message = "Parking Planner Service Failure";
    }

}




void ParkingSpacePlannerNode::cb_handle_parking_operation_response_(rclcpp::Client<ParkingTrigger>::SharedFuture future)
{
    auto request = std::make_shared<ParkingTrigger::Request>();

    while (!srv_client_parking_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

    auto result = srv_client_parking_->async_send_request(request);

    try
    {
        auto response = future.get();
        RCLCPP_INFO_STREAM(get_logger(), "Service Response: Result - " << (response->result == response->SUCCESS ? "SUCCESS" : "FAIL") 
                                                                       << "; Message - " << response->message);
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "Service Call failed: %s", e.what());
    }
    
}








// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ParkingSpacePlannerNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParkingSpacePlannerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}