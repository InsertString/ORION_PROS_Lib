#ifndef _ODOMETRY_HPP_
#define _ODOMETRY_HPP_

// Object for 2wheel Odometry
class Odom {
private:
    // sensors
    pros::Imu * imu;
    pros::ADIEncoder * YEncoder;
    pros::ADIEncoder * XEncoder;
    // angle variables
    double delta_angle;
    double past_angle;
    double angular_velocity;
    // encoder variables
    double delta_y_encoder;
    double past_y_encoder;
    double delta_x_encoder;
    double past_x_encoder;
    // vectors
    Vector2D local_offset;
    Vector2D global_offset;
    Vector2D global_position;
    Vector2D velocity;
    Vector2D past_velocity;
    Vector2D acceleration;
    // robot parameters
    double y_encoder_dist;
    double x_encoder_dist;
    double y_wheel_circumfrance;
    double x_wheel_circumfrance;
    Vector2D initial_position;
    double initial_angle;
    double tracking_delay;
public:
    #define ODOM_DEBUG_NONE 0
    #define ODOM_DEBUG_ENCODER_RAW 1
    #define ODOM_DEBUG_ENCODER_CM 2
    #define ODOM_DEBUG_LOCAL_OFFSET 3
    #define ODOM_DEBUG_GLOBAL_POSITION 4
    #define ODOM_DEBUG_VELOCITY 5
    #define ODOM_DEBUG_ACCEL 6

    Odom(pros::Imu * imu_obj, pros::ADIEncoder * XEncoder_obj, pros::ADIEncoder * YEncoder_obj);

    Vector2D getPosition();
    Vector2D getVelocity();
    Vector2D getAcceleration();
    double getAngle();
    double rad_angle();
    double getAngularVelocity();

    void configure(double x_e_dist, double x_wheel_c, double y_e_dist, double y_wheel_c, double delay);
    void configure_starting(Vector2D init_pos, double init_angle);

    void collect_data(int debug_flag);
    void calculate_position(int debug_flag);
};

#endif
