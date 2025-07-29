#include<rclcpp/rclcpp.hpp>
#include<walle_msg/srv/add_two_ints.hpp>


using namespace std::placeholders;

class SimpleServiceServer : public rclcpp::Node
{
public:
    SimpleServiceServer() : Node("simple_service_server")
    {
        service_ = create_service<walle_msg::srv::AddTwoInts>("add_two_ints", std::bind(&SimpleServiceServer::serviceCallback, this, _1, _2));

        RCLCPP_INFO_STREAM(get_logger(), "Service to add two ints is ready");
    }

private:
    rclcpp::Service<walle_msg::srv::AddTwoInts>::SharedPtr service_;

    void serviceCallback(std::shared_ptr<walle_msg::srv::AddTwoInts::Request> req,
                         std::shared_ptr<walle_msg::srv::AddTwoInts::Response> res)
    {
        RCLCPP_INFO_STREAM(get_logger(), "new request recieved a: " << req->a <<" b : " << req->b);
        res->sum = req->a + req->b;

        RCLCPP_INFO_STREAM(get_logger(), "the sum of the two ints is : "<<res->sum);
    }

};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleServiceServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}