#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <iostream>
#include <memory>
#include <thread>
#include <csignal>

class ControlManager : public rclcpp::Node
{
public:
    ControlManager()
        : Node("control_manager")
    {
        // Create clients
        security_stop_client_   = this->create_client<std_srvs::srv::Trigger>("security_stop");
        regulation_mode_client_ = this->create_client<std_srvs::srv::Trigger>("regulation_mode");
        start_filter_client_    = this->create_client<std_srvs::srv::Trigger>("start_filter");
        whole_body_mode_client_ = this->create_client<std_srvs::srv::Trigger>("whole_body_mode");

        // Save pointer for static signal handler
        instance_ = this;

        // Register Ctrl+C handler
        std::signal(SIGINT, &ControlManager::sigintHandler);

        RCLCPP_INFO(this->get_logger(), "Control Manager started.");
        RCLCPP_INFO(this->get_logger(), "Press 0 → Security Stop");
        RCLCPP_INFO(this->get_logger(), "Press 1 → Regulation Mode");
        RCLCPP_INFO(this->get_logger(), "Press 2 → Start Filter");
        RCLCPP_INFO(this->get_logger(), "Press 3 → Whole Body Mode (after starting filter)");

        // Start terminal input loop in a separate thread
        input_thread_ = std::thread(&ControlManager::readInput, this);
    }

    ~ControlManager()
    {
        if (input_thread_.joinable())
            input_thread_.detach();
    }

private:
    // Static instance for signal handler
    static ControlManager* instance_;

    static void sigintHandler(int)
    {
        if (instance_) {
            RCLCPP_WARN(instance_->get_logger(), "Ctrl+C pressed! Calling Security Stop...");
            instance_->callServiceSync(instance_->security_stop_client_);
        }

        rclcpp::shutdown();
    }

    void readInput()
    {
        while (rclcpp::ok())
        {
            int input;
            std::cin >> input;

            switch (input)
            {
                case 0:
                    callService(security_stop_client_);
                    break;
                case 1:
                    callService(regulation_mode_client_);
                    break;
                case 2:
                    callService(start_filter_client_);
                    break;
                case 3:
                    callService(whole_body_mode_client_);
                    break;
                default:
                    std::cout << "Invalid input! Use 0, 1, 2, or 3." << std::endl;
            }
        }
    }

    // Async service call for runtime input
    void callService(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client)
    {
        if (!client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "Service not available.");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

        client->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result) {
                if (result.get()->success) {
                    RCLCPP_INFO(this->get_logger(), "Service response: %s", result.get()->message.c_str());
                } else {
                    RCLCPP_WARN(this->get_logger(), "Service call failed");
                }
            });
    }

    // Synchronous service call for shutdown / Ctrl+C
    void callServiceSync(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client)
    {
        if (!client->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(this->get_logger(), "Service not available!");
            return;
        }

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto result_future = client->async_send_request(request);

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future)
            == rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Security stop response: %s", result_future.get()->message.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Security stop failed!");
        }
    }

    std::thread input_thread_;

    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr security_stop_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr regulation_mode_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_filter_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr whole_body_mode_client_;
};

// Initialize static member
ControlManager* ControlManager::instance_ = nullptr;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlManager>();
    rclcpp::spin(node);
    return 0;
}