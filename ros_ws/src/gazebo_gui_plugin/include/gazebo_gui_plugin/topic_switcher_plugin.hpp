#ifndef TOPIC_SWITCHER_PLUGIN_HPP
#define TOPIC_SWITCHER_PLUGIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <QComboBox>
#include <QVBoxLayout>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>

namespace gazebo
{
  class TopicSwitcherPlugin : public GUIPlugin
  {
    Q_OBJECT

    public:
      TopicSwitcherPlugin();
      virtual ~TopicSwitcherPlugin();

    public slots:
      void OnTopicSelected(int index);

    private:
      rclcpp::Node::SharedPtr node_;
      rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
      std::thread ros_thread_;
  };
}

#endif