#include <rclcpp/rclcpp.hpp>
#include <rototui_interfaces/srv/parking_trigger.hpp>



using namespace std::placeholders;


class ParkingPlannerService : public rclcpp::Node
{
public:
    
    ParkingPlannerService() : Node("parking_planner_service")   // name of the node
    {
        parking_service_ = create_service<rototui_interfaces::srv::ParkingTrigger>("call_parking_planner",       // name of the service
            std::bind(&ParkingPlannerService::handleServiceCallback, this, _1, _2));


        RCLCPP_INFO_STREAM(get_logger(), "Parking Planner Service is ready!");
    }



private:

    void handleServiceCallback(const std::shared_ptr<rototui_interfaces::srv::ParkingTrigger::Request> request,
                               const std::shared_ptr<rototui_interfaces::srv::ParkingTrigger::Response> response)
    {
        
        RCLCPP_INFO_STREAM(get_logger(), "New Trigger Request received! Status: " << request->trigger);


        if (request->trigger == true)
        {
            RCLCPP_INFO_STREAM(get_logger(), "Parking Planner Trigger is active");
            response->result = response->SUCCESS;
            response->message = "Parking Planner Service has been initialized successfully"; 
        }

        else {
            RCLCPP_INFO_STREAM(get_logger(), "Parking Planner Trigger is disabled");
            response->result = response->FAIL;
            response->message = "Parking Planner Service Failure";
        }
    
    }

    
    rclcpp::Service<rototui_interfaces::srv::ParkingTrigger>::SharedPtr parking_service_;

};



int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ParkingPlannerService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}