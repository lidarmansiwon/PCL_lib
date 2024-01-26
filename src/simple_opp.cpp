#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//rclcpp/rclcpp.hpp
//표준 c++ 헤더파일 인크루드, 위의 ROS2 시스템을 사용할 수 있게 해준다
//std_msgs/msg/string.hpp
//빌트인 메세지 타입을 포함해서 publish 하는데 사용한다
//include에 추가를 하게 되면 package.xml 파일에 의존성을 추가해줘야한다 아래에서 살펴보자

// Node 클래스 상속
class Talker : public rclcpp::Node {
private:
// Node를 주기적으로 실행시켜 줄 timer  rclcpp::TimerBase
    rclcpp::TimerBase::SharedPtr m_timer;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t m_count;
    size_t count_;

    //콜백함수
    void timer_callback() {
        m_count++;
        auto message = std_msgs::msg::String();
        message.data = "Hello, world! " + std::to_string(count_++);

        // log를 남기기
        RCLCPP_INFO(this->get_logger(), "OPP example, count : %d", m_count);

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        //publish하기
        publisher_->publish(message);
    }

public:
    // constructor
    Talker() : Node("simple_opp_node") {
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        // create_wall_timer 함수에 timer와 실행시킬 함수를 전달하면 주기적 실행을 할 수 있음
        // create_wall_timer는 3개의 매개변수를 std::bind를 이용해서 넘겨야 함
        // o.5초
        m_timer = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Talker::timer_callback, this));
    }

};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);  //초기화

    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}