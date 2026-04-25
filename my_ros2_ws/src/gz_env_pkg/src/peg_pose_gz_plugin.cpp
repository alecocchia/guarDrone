#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/math/Pose3.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/pose.pb.h>
#include <gz/plugin/Register.hh>

namespace gz_env_pkg
{
  class SetPegPosePlugin : public gz::sim::System,
                           public gz::sim::ISystemConfigure,
                           public gz::sim::ISystemPreUpdate
  {
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &/*_eventMgr*/) override
    {
      this->model = gz::sim::Model(_entity);
      // Non impostiamo più una posa di default hard-coded qui

      // Recupera il nome del modello per creare il topic dinamicamente
      std::string modelName = this->model.Name(_ecm);
      std::string topic = "/model/" + modelName + "/pose";

      // Sottoscrivi al topic corretto
      this->gzNode.Subscribe(topic, &SetPegPosePlugin::OnPoseMsg, this);
    }

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                           gz::sim::EntityComponentManager &_ecm) override
    {
      if (_info.paused || !this->receivedFirstMsg)
        return;

      // Aggiorna la posa con l'ultima ricevuta dal subscriber
      this->model.SetWorldPoseCmd(_ecm, this->pose);
    }

  void OnPoseMsg(const gz::msgs::Pose &_msg)
  {
    // Estrai la posizione
    const auto &pos = _msg.position();
    gz::math::Vector3d position(pos.x(), pos.y(), pos.z());
  
    // Estrai l'orientazione (quaternion)
    const auto &ori = _msg.orientation();
    
    // ATTENZIONE BUG RISOLTO: 
    // Nel codice originale avevo (ori.x(), ori.y(), ori.z(), ori.w())
    // Il costruttore matematico di Gazebo/Ignition richiede rigorosamente 
    // l'ordine (W, X, Y, Z)
    gz::math::Quaterniond rotation(ori.w(), ori.x(), ori.y(), ori.z());
  
    // Imposta la pose (position + rotation) sull'oggetto
    this->pose.Set(position, rotation);
    this->receivedFirstMsg = true;
  }

    private:
      gz::sim::Model model{gz::sim::kNullEntity};
      gz::math::Pose3d pose;
      gz::transport::Node gzNode {};
      bool receivedFirstMsg{false};
  };
}

// I macro di registrazione per Gazebo Garden
GZ_ADD_PLUGIN(gz_env_pkg::SetPegPosePlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz_env_pkg::SetPegPosePlugin,
                    "gz_env_pkg::SetPegPosePlugin")
