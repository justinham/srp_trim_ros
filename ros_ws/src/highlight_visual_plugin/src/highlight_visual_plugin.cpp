#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace gazebo
{
class HighlightVisualPlugin : public WorldPlugin
{
public:
  void Load(physics::WorldPtr world, sdf::ElementPtr) override
  {
    world_ = world;

    // Gazebo transport
    node_.reset(new transport::Node());
    node_->Init(world_->Name());
    visual_pub_ = node_->Advertise<msgs::Visual>("~/visual");

    // ROS 2 node inside plugin
    rcl_node_ = rclcpp::Node::make_shared("highlight_visual_plugin");
    sub_ = rcl_node_->create_subscription<std_msgs::msg::String>(
      "/viz/highlight_visual", 10,
      [this](std_msgs::msg::String::SharedPtr msg){ this->OnCmd(msg->data); });

    // Spin ROS2 in its own thread
    spin_thread_ = std::thread([this](){
      rclcpp::executors::SingleThreadedExecutor exec;
      exec.add_node(rcl_node_);
      exec.spin();
    });
  }

  void OnCmd(const std::string &payload)
  {
    // payload: {"model":"obj_123","on":true,"rgba":[1,0.2,0.2,0.9]}
    try {
      auto j = json::parse(payload);
      const std::string model = j.at("model").get<std::string>();
      const bool on = j.value("on", true);
      std::array<double,4> rgba = {1.0, 1.0, 0.0, 0.6};
      if (j.contains("rgba")) {
        auto v = j["rgba"];
        for (size_t i=0;i<4 && i<v.size();++i) rgba[i] = v[i].get<double>();
      }
      this->TintModel(model, on, rgba);
    } catch (const std::exception &e) {
      std::cerr << "[highlight_visual] bad payload: " << e.what() << "\n";
    }
  }

  void TintModel(const std::string &model, bool on, const std::array<double,4> &rgba)
  {
    // Try common visual names: model::link::visual for each link; also model::visual
    auto modelPtr = world_->ModelByName(model);
    if (!modelPtr) {
      // Still try a top-level guess
      this->SendVisualMsg(model + "::visual", on, rgba);
      return;
    }

    // All links -> guess each has a "visual" named visual
    auto links = modelPtr->GetLinks();
    bool any=false;
    for (auto &link : links) {
      const std::string guess = model + "::" + link->GetName() + "::visual";
      any = this->SendVisualMsg(guess, on, rgba) || any;
    }
    // Also try a simple top-level visual name
    any = this->SendVisualMsg(model + "::visual", on, rgba) || any;

    // If none matched, we’re still safe (Gazebo ignores unknown visuals)
    (void)any;
  }

  bool SendVisualMsg(const std::string &visualScopedName, bool on, const std::array<double,4> &rgba)
  {
    // Compose a Visual message targeting an existing visual by scoped name
    msgs::Visual vis;
    vis.set_name(visualScopedName);          // target visual
    vis.set_parent_name("");                 // not required
    vis.set_transparency(on ? 0.0 : 0.0);    // keep opaque; we tint via emissive/diffuse
    vis.set_delete_me(false);

    auto *mat = vis.mutable_material();
    if (on) {
      auto *em = mat->mutable_emissive();
      em->set_r(rgba[0]); em->set_g(rgba[1]); em->set_b(rgba[2]); em->set_a(rgba[3]);
      auto *df = mat->mutable_diffuse();
      df->set_r(rgba[0]); df->set_g(rgba[1]); df->set_b(rgba[2]); df->set_a(rgba[3]);
    } else {
      // "Unhighlight": set very small emissive; leave diffuse a bit toned to avoid flicker
      auto *em = mat->mutable_emissive();
      em->set_r(0.0); em->set_g(0.0); em->set_b(0.0); em->set_a(1.0);
      // (Optionally omit diffuse to keep original; here we keep minimal change)
    }

    visual_pub_->Publish(vis);
    return true; // fire-and-forget; unknown names are simply ignored by Gazebo
  }

  ~HighlightVisualPlugin() override
  {
    if (rcl_node_) {
      rclcpp::shutdown();
    }
    if (spin_thread_.joinable()) spin_thread_.join();
  }

private:
  physics::WorldPtr world_;
  std::unique_ptr<transport::Node> node_;
  transport::PublisherPtr visual_pub_;

  rclcpp::Node::SharedPtr rcl_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::thread spin_thread_;
};

GZ_REGISTER_WORLD_PLUGIN(HighlightVisualPlugin)
} // namespace gazebo

