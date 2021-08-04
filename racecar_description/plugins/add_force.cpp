#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/Model.hh>

namespace gazebo
{
    class ForcePlugin : public ModelPlugin
    {
        protected:
            physics::ModelPtr model;
            gazebo::physics::LinkPtr base_link_;
            gazebo::physics::LinkPtr fl_wheel_;
            gazebo::physics::LinkPtr fr_wheel_;
            gazebo::physics::LinkPtr rl_wheel_;
            gazebo::physics::LinkPtr rr_wheel_;
        public: 
            ForcePlugin() : ModelPlugin() {
            }

            void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)  {
                this->model = _model;
                std::string namespace_ = "";
                if (_sdf->HasElement("namespace")) {
                    namespace_ = _sdf->Get<std::string>("namespace");
                }

                printf("Force plugin loaded: %s\n",_model->GetName().c_str());
                std::stringstream ss1;
                ss1 << namespace_ << "base_link";
                std::string base_link = ss1.str();
                std::stringstream ss2;
                ss2 << namespace_ << "front_left_wheel_link";
                std::string fl_wheel = ss2.str();
                std::stringstream ss3;
                ss3 << namespace_ << "front_right_wheel_link";
                std::string fr_wheel = ss3.str();
                std::stringstream ss4;
                ss4 << namespace_ << "rear_left_wheel_link";
                std::string rl_wheel = ss4.str();
                std::stringstream ss5;
                ss5 << namespace_ << "rear_right_wheel_link";
                std::string rr_wheel = ss5.str();

                base_link_ = _model->GetChildLink(base_link);
                fl_wheel_ = _model->GetChildLink(fl_wheel);
                fr_wheel_ = _model->GetChildLink(fr_wheel);
                rl_wheel_ = _model->GetChildLink(rl_wheel);
                rr_wheel_ = _model->GetChildLink(rr_wheel);

                printf("Applying Force on joints: %s, %s, %s, %s, %s\n", base_link.c_str(), fl_wheel.c_str(), fr_wheel.c_str(), rl_wheel.c_str(), rr_wheel.c_str());
                // Listen to the update event. This event is broadcast every
                // simulation iteration.
                this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                        std::bind(&ForcePlugin::OnUpdate, this));
            }

            void OnUpdate() {
                ignition::math::Vector3d force = ignition::math::Vector3d(0, 0, -4000);
                base_link_->AddForce(force / 2);
                fl_wheel_->AddForce(force / 4);
                fr_wheel_->AddForce(force / 4);
                rl_wheel_->AddForce(force / 4);
                rr_wheel_->AddForce(force / 4);
            }

        private: 
            // Pointer to the update event connection
            event::ConnectionPtr updateConnection;
    };
    GZ_REGISTER_MODEL_PLUGIN(ForcePlugin);
}
