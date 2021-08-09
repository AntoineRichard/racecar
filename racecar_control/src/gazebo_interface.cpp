#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class GZInterface {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber drive_sub_;
        ros::Publisher left_steering_hinge_pub_;
        ros::Publisher right_steering_hinge_pub_;
        ros::Publisher rear_left_wheel_pub_;
        ros::Publisher rear_right_wheel_pub_;

        // Vehicle configuration
        float wheel_radius_;
        float wheel_lr_dist_;
        float wheel_fr_dist_;
        // Steering parameters
        float steer_max_;
        float steer_max_speed_;
        // Throttle parameters
        float vlin_max_;
        float vlin_min_;
        float vlin_max_acc_;
        float vlin_min_acc_;
        // Update_rate
        float update_rate_;
        float dt_;
        // Internal State
        float req_vel_;
        float cur_vel_;
        float req_steer_;
        float cur_steer_;
        float left_steer_;
        float right_steer_;
        float left_throttle_;
        float right_throttle_;
        ros::Time last_left_steer_update_;
        ros::Time last_right_steer_update_;
        ros::Time last_left_throttle_update_;
        ros::Time last_right_throttle_update_;

        void updateVelocity();
        void updateSteering();
        void applyModel();
        void cmdCallback(const ackermann_msgs::AckermannDriveStampedConstPtr&);

    public:
        GZInterface();
        void run();
};

GZInterface::GZInterface() : nh_("~") {

    nh_.param("wheel_radius", wheel_radius_, .05f); //Meters
    nh_.param("wheel_lr_dist", wheel_lr_dist_, .1f); //Meters
    nh_.param("wheel_fr_dist", wheel_fr_dist_, .325f); //Meters

    nh_.param("max_steering_angle", steer_max_, 0.5f); //Radians
    nh_.param("max_steering_speed", steer_max_speed_, 4.0f); //Radians per seconds

    nh_.param("max_linear_speed", vlin_max_, 10.f); //Meters per seconds
    nh_.param("min_linear_speed", vlin_min_, -2.5f); //Meters per seconds
    nh_.param("max_linear_acceleration", vlin_max_acc_, 2.5f); //Meter per seconds square
    nh_.param("min_linear_acceleration", vlin_min_acc_, -2.5f); //Meters per seconds square

    nh_.param("update_rate", update_rate_, 100.f); //Update rate Hz
    dt_ = 1/update_rate_;

    drive_sub_ = nh_.subscribe("cmd_drive",1,&GZInterface::cmdCallback,this);
    left_steering_hinge_pub_ = nh_.advertise<std_msgs::Float64>("left_steering_hinge_position_ctrl",1,true);
    right_steering_hinge_pub_ = nh_.advertise<std_msgs::Float64>("right_steering_hinge_position_ctrl",1,true);
    rear_left_wheel_pub_ = nh_.advertise<std_msgs::Float64>("rear_left_wheel_vel_ctrl",1,true);
    rear_right_wheel_pub_ = nh_.advertise<std_msgs::Float64>("rear_right_wheel_vel_ctrl",1,true);

    cur_steer_ = 0;
    req_steer_ = 0;
    cur_vel_ = 0;
    req_vel_ = 0;
    last_left_steer_update_     = ros::Time::now();
    last_right_steer_update_    = ros::Time::now();
    last_left_throttle_update_  = ros::Time::now();
    last_right_throttle_update_ = ros::Time::now();
}

void GZInterface::cmdCallback(const ackermann_msgs::AckermannDriveStampedConstPtr& data){
    float tmp_vel = data->drive.speed;
    float tmp_steer = data->drive.steering_angle;
    // Constraint values to the [-1;1] range.
    if (tmp_vel > 1.0){
        tmp_vel = 1.0;
    } else if (tmp_vel < - 1.0){
        tmp_vel = -1.0;
    }
    if (tmp_steer > 1.0){
        tmp_steer = 1.0;
    } else if (tmp_steer < - 1.0){
        tmp_steer = -1.0;
    }
    // Apply scaling based on max velocities
    if (tmp_vel > 0) {
        tmp_vel = tmp_vel*vlin_max_;
    } else if (tmp_vel < 0) {
        tmp_vel = - tmp_vel*vlin_min_;
    }
    tmp_steer = tmp_steer*steer_max_;
    // Save
    req_vel_ = tmp_vel;
    req_steer_ = tmp_steer;
}

void GZInterface::updateVelocity() {
    float dv = req_vel_ - cur_vel_;
    float acc = dv/dt_;
    if (acc > vlin_max_acc_) {
        acc = vlin_max_acc_;
    } else if (acc < vlin_min_acc_) {
        acc = vlin_min_acc_;
    }
    float new_vel = cur_vel_ + acc*dt_;
    if (new_vel > vlin_max_) {
        new_vel = vlin_max_;
    } else if (new_vel < vlin_min_) {
        new_vel = vlin_min_;
    }
    cur_vel_ = new_vel;
}

void GZInterface::updateSteering() {
    float dss = req_steer_ - cur_steer_;
    float speed = dss/dt_;

    if (speed > steer_max_speed_) {
        speed = steer_max_speed_;
    } else if (speed < - steer_max_speed_) {
        speed =  - steer_max_speed_;
    }
    float new_steer = cur_steer_ + speed*dt_;
    if (new_steer > steer_max_) {
        new_steer = steer_max_;
    } else if (new_steer < - steer_max_) {
        new_steer = - steer_max_;
    }
    cur_steer_ = new_steer;
}

void GZInterface::applyModel() {
    float r = wheel_fr_dist_/(std::tan(cur_steer_) + 1e-9);
    float omega = cur_vel_/r;
    float v_wx, v_wy, steer, throttle, delta_steer, delta_time, delta_throttle;
    std_msgs::Float64 data;
    // Front Left Wheel
    v_wx = cur_vel_ - omega*wheel_lr_dist_;
    v_wy = omega*wheel_fr_dist_;
    steer = std::atan2(v_wy, v_wx);
    steer = std::remainder(steer, M_PI);
    delta_steer = fabs(steer - left_steer_);
    delta_time = ros::Time::now().toSec() - last_left_steer_update_.toSec();
    if ((delta_steer > 0.01) || (delta_time > 0.25)) {
        data.data = steer;
        left_steer_ = steer;
        left_steering_hinge_pub_.publish(data);
        last_left_steer_update_ = ros::Time::now();
    }
    // Front Right Wheel
    v_wx = cur_vel_ + omega*wheel_lr_dist_;
    v_wy = omega*wheel_fr_dist_;
    steer = std::atan2(v_wy, v_wx);
    steer = std::remainder(steer, M_PI);
    delta_steer = fabs(steer - right_steer_);
    delta_time = ros::Time::now().toSec() - last_right_steer_update_.toSec();
    if ((delta_steer > 0.01) || (delta_time > 0.25)) {
        data.data = steer;
        right_steer_ = steer;
        right_steering_hinge_pub_.publish(data);
        last_right_steer_update_ = ros::Time::now();
    }
    // Rear Left Wheel
    v_wx = cur_vel_ - omega*wheel_lr_dist_;
    v_wy = 0;
    throttle = sgn(cur_vel_)*std::hypot(v_wx,v_wy)/wheel_radius_;
    delta_throttle = fabs(throttle - left_throttle_);
    delta_time = ros::Time::now().toSec() - last_left_throttle_update_.toSec();
    if ((delta_throttle > 0.5) || (delta_time > 0.25)) {
        data.data = throttle;
        left_throttle_ = throttle;
        rear_left_wheel_pub_.publish(data);
        last_left_throttle_update_ = ros::Time::now();
    }
    // Rear Right Wheel
    v_wx = cur_vel_ + omega*wheel_lr_dist_;
    v_wy = 0;
    throttle = sgn(cur_vel_)*std::hypot(v_wx, v_wy)/wheel_radius_;
    delta_throttle = fabs(throttle - right_throttle_);
    delta_time = ros::Time::now().toSec() - last_right_throttle_update_.toSec();
    if ((delta_throttle > 0.5) || (delta_time > 0.25)) {
        data.data = throttle;
        right_throttle_ = throttle;
        rear_right_wheel_pub_.publish(data);
        last_right_throttle_update_ = ros::Time::now();
    }
}

void GZInterface::run() {
    ros::Rate r(update_rate_);
    ros::AsyncSpinner spinner(2);
    spinner.start();
    while (ros::ok()){
        updateVelocity();
        updateSteering();
        applyModel();
        r.sleep();
    }
}

int main(int argc, char * argv[]) {
    ros::init(argc,argv,"gz_racecar_interface");
    GZInterface GZI;
    GZI.run();
}