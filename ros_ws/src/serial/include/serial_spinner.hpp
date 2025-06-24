/** \file serial_spinner.hpp
 * \brief Serial spinner class to interface with the boards
 *
 * \author Sébastien Darche <sebastien.darche@polymtl.ca>
 */

#pragma once

// Local includes

#include "protocol.hpp"

// Std includes

#include <string>

// ROS includes

#include "rclcpp/rclcpp.hpp"

#include "polystar_msgs/msg/game_stage.hpp"
#include "polystar_msgs/msg/game_status.hpp"
#include "polystar_msgs/msg/movement.hpp"
#include "polystar_msgs/msg/position_feedback.hpp"
#include "polystar_msgs/msg/shoot.hpp"
#include "polystar_msgs/msg/target.hpp"
#include "polystar_msgs/msg/turret_feedback.hpp"

class SerialSpinner : public rclcpp::Node {
  public:
    /** \brief Constructor
     */
    SerialSpinner(double frequency = 500.);

    /** \brief Destructor
     */
    ~SerialSpinner();

    /** \fn callbackTarget
     * \brief Callback for new target coordinates
     */
    void callbackTarget(const polystar_msgs::msg::Target::SharedPtr);

    /** \fn callbackTarget
     * \brief Callback for new target coordinates
     */
    void callbackMovement(const polystar_msgs::msg::Movement::SharedPtr);

    /** \fn callbackShoot
     * \brief Callback for shoot orders
     */
    void callbackShoot(const polystar_msgs::msg::Shoot::SharedPtr);

    /** \fn spin
     * \brief Spins, waiting for requests and listens to the serial port
     */
    void spin();

    // ----- Testing methods ----- //

    /** \fn serializeMessage
     * \brief Serialize a message to a buffer
     */
    static std::vector<uint8_t>
    serializeMessage(const serial::msg::OutgoingMessage& message);

    /** \fn deserializeMessage
     * \brief Deserialize a message from a buffer
     */
    static serial::msg::IncomingMessage
    deseralizeMessage(const std::vector<uint8_t>& buffer);

  private:
    /** \fn initSerial
     * \brief Initializes the serial file descriptor. To be called by the
     * constructor
     */
    void initSerial(const std::string& device);

    /** \fn handleSerial
     * \brief Attempts to read incoming messages from the serial port and
     * dispatches them
     */
    void handleSerial();

    /** \fn handleMessage
     * \brief Handle an incoming serial message depending on its type
     */
    template <typename T> void handleMessage(const T& message);

    /** \fn sendMessage
     * \brief Send an outgoing message
     */
    void sendMessage(const serial::msg::OutgoingMessage& message);

    rclcpp::Publisher<polystar_msgs::msg::GameStatus>::SharedPtr pub_status;
    rclcpp::Publisher<polystar_msgs::msg::GameStage>::SharedPtr pub_stage;
    rclcpp::Publisher<polystar_msgs::msg::TurretFeedback>::SharedPtr pub_turret;
    rclcpp::Publisher<polystar_msgs::msg::PositionFeedback>::SharedPtr
        pub_position;
    rclcpp::Subscription<polystar_msgs::msg::Target>::SharedPtr sub_target;
    rclcpp::Subscription<polystar_msgs::msg::Movement>::SharedPtr sub_movement;
    rclcpp::Subscription<polystar_msgs::msg::Shoot>::SharedPtr sub_shoot;

    rclcpp::TimerBase::SharedPtr timer;

    int fd = -1;
    int baud_rate, length, stop_bits;
    bool parity;
    double frequency;

    int64_t encoder_resolution;

    bool shooting_enabled = true;
};
