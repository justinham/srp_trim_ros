#include <gazebo/gazebo.hh>
#include <gazebo/gui/gui.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <QPushButton>
#include <QComboBox>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace gazebo
{
  class TopicSwitcherPlugin : public GUIPlugin
  {
    Q_OBJECT

  public:
    TopicSwitcherPlugin() : GUIPlugin()
    {
      // --- UI LAYOUT ---
      auto *layout = new QVBoxLayout;

      // 4) Driving Mode
      auto *drivingLabel = new QLabel("Driving Mode:");
      drivingComboBox_ = new QComboBox;
      drivingComboBox_->addItem("Manual");
      drivingComboBox_->addItem("Autonomous");

      layout->addWidget(drivingLabel);
      layout->addWidget(drivingComboBox_);

      // 1) Actor topic selector
      auto *topicLabel = new QLabel("Actor state topic:");
      topicComboBox_ = new QComboBox;
      topicComboBox_->addItem("ego_actor_states");
      topicComboBox_->addItem("infra_actor_states");
      topicComboBox_->addItem("fused_actor_states");

      layout->addWidget(topicLabel);
      layout->addWidget(topicComboBox_);

      // 2) Ingress lane selector
      auto *laneLabel = new QLabel("Ingress lane:");
      laneComboBox_ = new QComboBox;

      laneComboBox_->addItem("North to South (l1)");
      laneComboBox_->addItem("East to West (l3)");
      laneComboBox_->addItem("West to East (l5)");
      laneComboBox_->addItem("South to North (l7)");

      layout->addWidget(laneLabel);
      layout->addWidget(laneComboBox_);

      // 3) Direction selector
      auto *directionLabel = new QLabel("Direction:");
      directionComboBox_ = new QComboBox;
      directionComboBox_->addItem("Left");
      directionComboBox_->addItem("Straight");
      directionComboBox_->addItem("Right");

      layout->addWidget(directionLabel);
      layout->addWidget(directionComboBox_);

      // Attach layout to plugin
      this->setLayout(layout);

      // --- Qt connections ---
      connect(topicComboBox_, QOverload<int>::of(&QComboBox::activated),
              this, &TopicSwitcherPlugin::OnTopicSelected);

      connect(laneComboBox_, QOverload<int>::of(&QComboBox::activated),
              this, &TopicSwitcherPlugin::OnLaneSelected);

      connect(directionComboBox_, QOverload<int>::of(&QComboBox::activated),
              this, &TopicSwitcherPlugin::OnDirectionSelected);

      connect(drivingComboBox_, QOverload<int>::of(&QComboBox::activated),
              this, &TopicSwitcherPlugin::OnDrivingSelected);

      // --- ROS2 setup ---
      int argc = 0;
      char **argv = nullptr;
      rclcpp::init(argc, argv);

      node_ = rclcpp::Node::make_shared("topic_switcher_plugin");

      topic_pub_ =
          node_->create_publisher<std_msgs::msg::String>("/selected_topic", 10);
      lane_pub_ =
          node_->create_publisher<std_msgs::msg::String>("/selected_ingress_lane", 10);
      direction_pub_ =
          node_->create_publisher<std_msgs::msg::String>("/selected_direction", 10);
      driving_pub_ =
          node_->create_publisher<std_msgs::msg::String>("/selected_driving_mode", 10);

      ros_thread_ = std::thread([this]() {
        rclcpp::spin(this->node_);
      });
    }

    ~TopicSwitcherPlugin() override
    {
      rclcpp::shutdown();
      if (ros_thread_.joinable())
      {
        ros_thread_.join();
      }
    }

  public slots:
    void OnTopicSelected(int /*index*/)
    {
      if (!topic_pub_) return;
      auto msg = std_msgs::msg::String();
      msg.data = topicComboBox_->currentText().toStdString();
      topic_pub_->publish(msg);
    }

    void OnLaneSelected(int /*index*/)
    {
      if (!lane_pub_) return;
      auto msg = std_msgs::msg::String();
      msg.data = laneComboBox_->currentText().toStdString();
      lane_pub_->publish(msg);
    }

    void OnDirectionSelected(int /*index*/)
    {
      if (!direction_pub_) return;
      auto msg = std_msgs::msg::String();
      msg.data = directionComboBox_->currentText().toStdString();
      direction_pub_->publish(msg);
    }

    void OnDrivingSelected(int /*index*/)
    {
      if (!driving_pub_) return;
      auto msg = std_msgs::msg::String();
      msg.data = drivingComboBox_->currentText().toStdString();
      driving_pub_->publish(msg);
    }

  private:
    // Qt widgets
    QComboBox *topicComboBox_;
    QComboBox *laneComboBox_;
    QComboBox *directionComboBox_;
    QComboBox *drivingComboBox_;

    // ROS2
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr topic_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lane_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr direction_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr driving_pub_;
    std::thread ros_thread_;
  };

  GZ_REGISTER_GUI_PLUGIN(TopicSwitcherPlugin)
}

#include "topic_switcher_plugin.moc"

