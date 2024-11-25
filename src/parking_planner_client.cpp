#include <rclcpp/rclcpp.hpp>
#include <rototui_interfaces/srv/parking_trigger.hpp>


#include <chrono>


using namespace std::placeholders;
using namespace std::chrono_literals;


class ParkingPlannerClient : public rclcpp::Node
{
public:
    ParkingPlannerClient(bool trigger_status) : Node("parking_planner_client")
    {
        // Create a parking operation client for calling the executable
        parking_client_ = create_client<rototui_interfaces::srv::ParkingTrigger>("call_parking_planner");   // done

        // Create the trigger request here
        auto request = std::make_shared<rototui_interfaces::srv::ParkingTrigger::Request>();
        request->trigger = trigger_status;



        while (!parking_client_->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto result = parking_client_->async_send_request(request, std::bind(&ParkingPlannerClient::responseCallback, this, _1));

    }




private:

    rclcpp::Client<rototui_interfaces::srv::ParkingTrigger>::SharedPtr parking_client_;


    void responseCallback(rclcpp::Client<rototui_interfaces::srv::ParkingTrigger>::SharedFuture future)
    {
        try
        {
            auto response = future.get();
            RCLCPP_INFO_STREAM(get_logger(), "Service Response: Result - " << (response->result == response->SUCCESS ? "SUCCESS" : "FAIL") 
                                                                           << "; Message - " << response->message);
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
        }
    }        

};





int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    if (argc != 2)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Usage: parking_planner_client 1 or 0 (1 for activate, 0 for disable)");
        return 1;
    }

    bool trigger_status = (std::string(argv[1]) == "1");


    auto node = std::make_shared<ParkingPlannerClient>(trigger_status);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}