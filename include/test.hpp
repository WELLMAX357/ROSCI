#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include "tourobo2022_msg/msg/air4to7.hpp"

#include <can_on_ros2/can_client_ros.hpp>
#include "can_on_ros2/fflib/devices/can/can_servo_v3.h"
#include "can_on_ros2/fflib/devices/can/can_air_rc20_v1_driver.h"

#include "definition/ps5con.hpp"
#include "control/oncebutton.hpp"

#define PI 3.1415926535
constexpr float DegToRad = PI / 180;

class CZHandNode : public rclcpp::Node
{
  typedef sensor_msgs::msg::Joy Joy;

  using air_msg = tourobo2022_msg::msg::Air4to7;

  enum class Phase
  {
    NONE,
    PREPARE,
    INIT,
    PREPARE_HOLD,
    HOLD,
    STORAGE,
    PREPARE_RELEASE,
    RELEASE,
    PREPARE_HOUSE,
    HOUSE,
  };

public:
  CZHandNode()
    : Node("cz_hand")
    , can_(this, "can0_to")
    , servo_(&can_, 0)
    , air_(&can_, 0, false)
  {
    joy_sub_ = this->create_subscription<Joy>(
      "/joy", 50,
      std::bind(&CZHandNode::joyCallback, this, std::placeholders::_1));
    oa_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&CZHandNode::oaCallback, this));
    log_timer = this->create_wall_timer(std::chrono::milliseconds(1000),
                                        std::bind(&CZHandNode::logger, this));
    air_pub = this->create_publisher<air_msg>("/air_4_to_7", 50);

    this->declare_parameter("none0", 0.0);
    this->declare_parameter("none1", 0.0);
    this->declare_parameter("none2", 0.0);

    this->declare_parameter("prepare0", 0.0);
    this->declare_parameter("prepare1", 0.0);
    this->declare_parameter("prepare2", 0.0);

    this->declare_parameter("init0", 0.0);
    this->declare_parameter("init1", 0.0);
    this->declare_parameter("init2", 0.0);

    this->declare_parameter("prepare_hold0", 0.0);
    this->declare_parameter("prepare_hold1", 0.0);
    this->declare_parameter("prepare_hold2", 0.0);

    this->declare_parameter("hold0", 0.0);
    this->declare_parameter("hold1", 0.0);
    this->declare_parameter("hold2", 0.0);

    this->declare_parameter("storage0", 0.0);
    this->declare_parameter("storage1", 0.0);
    this->declare_parameter("storage2", 0.0);

    this->declare_parameter("prepare_release0", 0.0);
    this->declare_parameter("prepare_release1", 0.0);
    this->declare_parameter("prepare_release2", 0.0);

    this->declare_parameter("release0", 0.0);
    this->declare_parameter("release1", 0.0);
    this->declare_parameter("release2", 0.0);

    this->declare_parameter("prepare_house0", 0.0);
    this->declare_parameter("prepare_house1", 0.0);
    this->declare_parameter("prepare_house2", 0.0);

    this->declare_parameter("house0", 0.0);
    this->declare_parameter("house1", 0.0);
    this->declare_parameter("house2", 0.0);
  }

private:
  CanClientROS can_;
  fortefibre::CanServoV1Driver servo_;
  fortefibre::CanAirRC20V1Driver air_;

  rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<air_msg>::SharedPtr air_pub;
  rclcpp::TimerBase::SharedPtr oa_timer_, log_timer;

  Phase phase_ = Phase::NONE;

  OnceButton circle, triangle;

  void oaCallback()
  {
    auto message = tourobo2022_msg::msg::Air4to7();
    switch (phase_)
    {
      case Phase::NONE:
        // 0 脱力
        message.air_4 = 1;
        //                servo_.sendRadICS(0,
        //                this->get_parameter("none0").as_double());
        //                servo_.sendRadICS(1,
        //                this->get_parameter("none1").as_double());
        //                servo_.sendRadICS(2,
        //                this->get_parameter("none2").as_double());
        servo_.sendFreeICS(0);
        servo_.sendFreeICS(1);
        servo_.sendFreeICS(2);
        break;

      case Phase::PREPARE:
        // 1 OB保持
        message.air_4 = 0;
        //                servo_.sendRadICS(0,
        //                this->get_parameter("prepare0").as_double());
        //                servo_.sendRadICS(1,
        //                this->get_parameter("prepare1").as_double());
        //                servo_.sendRadICS(2,
        //                this->get_parameter("prepare2").as_double());
        servo_.sendFreeICS(0);
        servo_.sendFreeICS(1);
        servo_.sendFreeICS(2);
        break;

      case Phase::INIT:
        // 2 初期格納
        message.air_4 = 0;
        servo_.sendRadICS(0, this->get_parameter("init0").as_double());
        servo_.sendRadICS(1, this->get_parameter("init1").as_double());
        servo_.sendRadICS(2, this->get_parameter("init2").as_double());
        //                servo_.sendRadICS(0, 45 * DegToRad);
        //                servo_.sendRadICS(1, -115 * DegToRad);
        //                servo_.sendRadICS(2, 0 * DegToRad);
        break;

      case Phase::PREPARE_HOLD:
        // 3 鉄板入れる
        message.air_4 = 0;
        servo_.sendRadICS(0, this->get_parameter("prepare_hold0").as_double());
        servo_.sendRadICS(1, this->get_parameter("prepare_hold1").as_double());
        servo_.sendRadICS(2, this->get_parameter("prepare_hold2").as_double());
        //                servo_.sendRadICS(0, -23 * DegToRad);
        //                servo_.sendRadICS(1, -80 * DegToRad);
        //                servo_.sendRadICS(2, 0 * DegToRad);
        break;

      case Phase::HOLD:
        // 4 アーム下ろす
        message.air_4 = 0;
        servo_.sendRadICS(0, this->get_parameter("hold0").as_double());
        servo_.sendRadICS(1, this->get_parameter("hold1").as_double());
        servo_.sendRadICS(2, this->get_parameter("hold2").as_double());
        //                servo_.sendRadICS(0, -23 * DegToRad);
        //                servo_.sendRadICS(1, -98 * DegToRad);
        //                servo_.sendRadICS(2, 0 * DegToRad);
        break;

      case Phase::STORAGE:
        // 5 OA保持
        message.air_4 = 0;
        servo_.sendRadICS(0, this->get_parameter("storage0").as_double());
        servo_.sendRadICS(1, this->get_parameter("storage1").as_double());
        servo_.sendRadICS(2, this->get_parameter("storage2").as_double());
        //                servo_.sendRadICS(0, 35 * DegToRad);
        //                servo_.sendRadICS(1, -100 * DegToRad);
        //                servo_.sendRadICS(2, 0 * DegToRad);
        break;

      case Phase::PREPARE_RELEASE:
        // 6 鉄板解除
        message.air_4 = 0;
        servo_.sendRadICS(0,
                          this->get_parameter("prepare_release0").as_double());
        servo_.sendRadICS(1,
                          this->get_parameter("prepare_release1").as_double());
        servo_.sendRadICS(2,
                          this->get_parameter("prepare_release2").as_double());
        //                servo_.sendRadICS(0, -23 * DegToRad);
        //                servo_.sendRadICS(1, -95 * DegToRad);
        //                servo_.sendRadICS(2, -90 * DegToRad);z
        break;

      case Phase::RELEASE:
        // 7 OB開放
        message.air_4 = 1;
        servo_.sendRadICS(0, this->get_parameter("release0").as_double());
        servo_.sendRadICS(1, this->get_parameter("release1").as_double());
        servo_.sendRadICS(2, this->get_parameter("release2").as_double());
        //                servo_.sendRadICS(0, -23 * DegToRad);
        //                servo_.sendRadICS(1, -40 * DegToRad);
        //                servo_.sendRadICS(2, -90 * DegToRad);
        break;

      case Phase::PREPARE_HOUSE:
        // 8 アーム解除
        message.air_4 = 1;
        servo_.sendRadICS(0, this->get_parameter("prepare_house0").as_double());
        servo_.sendRadICS(1, this->get_parameter("prepare_house1").as_double());
        servo_.sendRadICS(2, this->get_parameter("prepare_house2").as_double());
        break;

      case Phase::HOUSE:
        // 9 アーム格納
        message.air_4 = 1;
        servo_.sendRadICS(0, this->get_parameter("house0").as_double());
        servo_.sendRadICS(1, this->get_parameter("house1").as_double());
        servo_.sendRadICS(2, this->get_parameter("house2").as_double());
        //                servo_.sendRadICS(0, 45 * DegToRad);
        //                servo_.sendRadICS(1, -115 * DegToRad);
        //                servo_.sendRadICS(2, 0 * DegToRad);
        break;
    }
    air_pub->publish(message);
  }

  void logger()
  {
    //        RCLCPP_INFO(this->get_logger(), "Phase: %d", phase_);
  }

  void joyCallback(const Joy::SharedPtr joy)
  {
    if (circle.update(joy->buttons[BTN_CIRCLE]))
    {
      switch (phase_)
      {
        case Phase::NONE:
          phase_ = Phase::PREPARE;
          break;

        case Phase::PREPARE:
          phase_ = Phase::INIT;
          break;

        case Phase::INIT:
          phase_ = Phase::PREPARE_HOLD;
          break;

        case Phase::PREPARE_HOLD:
          phase_ = Phase::HOLD;
          break;

        case Phase::HOLD:
          phase_ = Phase::STORAGE;
          break;

        case Phase::STORAGE:
          phase_ = Phase::PREPARE_RELEASE;
          break;

        case Phase::PREPARE_RELEASE:
          phase_ = Phase::RELEASE;
          break;

        case Phase::RELEASE:
          phase_ = Phase::PREPARE_HOUSE;
          break;

        case Phase::PREPARE_HOUSE:
          phase_ = Phase::HOUSE;
          break;

        case Phase::HOUSE:
          break;
      }
    }

    if (triangle.update(joy->buttons[BTN_TRIANGLE]))
    {
      switch (phase_)
      {
        case Phase::HOUSE:
          phase_ = Phase::PREPARE_HOUSE;
          break;

        case Phase::PREPARE_HOUSE:
          phase_ = Phase::RELEASE;
          break;

        case Phase::RELEASE:
          phase_ = Phase::PREPARE_RELEASE;
          break;

        case Phase::PREPARE_RELEASE:
          phase_ = Phase::STORAGE;
          break;

        case Phase::STORAGE:
          phase_ = Phase::HOLD;
          break;

        case Phase::HOLD:
          phase_ = Phase::PREPARE_HOLD;
          break;

        case Phase::PREPARE_HOLD:
          phase_ = Phase::INIT;
          break;

        case Phase::INIT:
          phase_ = Phase::PREPARE;
          break;

        case Phase::PREPARE:
          phase_ = Phase::NONE;
          break;

        case Phase::NONE:
          break;
      }
    }
  }
};