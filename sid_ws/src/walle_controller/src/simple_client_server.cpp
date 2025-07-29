#include <rclcpp/rclcpp.hpp>

#include "walle_msg/srv/add_two_ints.hpp"

#include <chrono>
#include <memory>

using namespace std::chrono_literals;

using std::placeholders::_1;

class SimpleServerClient : public rclcpp::Node 
{
public:
    SimpleServerClient(int a, int b) : Node("Simple_server_client")
    {
        client_ = create_client<walle_msg::srv::AddTwoInts>("add_two_ints");


        auto request = std::make_shared<walle_msg::srv::AddTwoInts::Request>();
        request-> a = a;
        request-> b = b;

        while (!client_->wait_for_service(1s))
        {
            if(!rclcpp::ok())
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "the service node was interupted.....");
                return;
            }

            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "the service didn't started pls wait......");
        }


        auto result =  client_->async_send_request(request, std::bind(&SimpleServerClient::responseCallback, this, _1)); 
    }

private:
    rclcpp::Client<walle_msg::srv::AddTwoInts>::SharedPtr client_;



    void responseCallback(rclcpp::Client<walle_msg::srv::AddTwoInts>::SharedFuture future)
    {
        if(future.valid())
        {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "service response : "<<future.get()->sum);
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "service failure , try again!!!");
        }
        
    }

};



int main (int argc, char* argv[])
{
    rclcpp::init(argc , argv);

    if( argc != 3)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "the service is having more than 2 args a and b");
        return 1;
    }

    auto node = std::make_shared<SimpleServerClient>(atoi(argv[1]), atoi(argv[2]));

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}