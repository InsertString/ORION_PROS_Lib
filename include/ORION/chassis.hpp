#ifndef _CHASSIS_HPP_
#define _CHASSIS_HPP_

#include "main.h"

using namespace pros;

enum ChassisType {
    STANDARD,
    HOLONOMIC,
    UNDEFINED
};

class Chassis {
private:
    // the chassis type
    ChassisType type;

    // PIDs
    PID forward_PID;
    PID sideways_PID;
    PID turn_PID;

    // Odom stuff
    Odom * odom;
    pros::Imu * imu;

    // Motor Configurations
    Motor ** LeftMotors;
    unsigned int num_L;
    Motor ** RightMotors;
    unsigned int num_R;
    Motor ** LeftFrontMotors;
    unsigned int num_LF;
    Motor ** LeftBackMotors;
    unsigned int num_LB;
    Motor ** RightFrontMotors;
    unsigned int num_RF;
    Motor ** RightBackMotors;
    unsigned int num_RB;

    // motor position variables
    double wheel_circumfrence_cm;
    double gear_ratio;
    motor_gearset_e motor_gearing;
public:

    /**
     * Constructor for chassis object
     * 
     * NOTE: does not configure odometry or an imu
     */
    Chassis();

    /**
     * Constructor for chassis object
     * 
     * \param odom
     *        the odometry object
     */
    Chassis(Odom * odom);

    /**
     * Constructor for chassis object
     * 
     * \param imu
     *        the imu object
     */
    Chassis(Imu * imu);

    /**
     * Configures the type of chassis
     * 
     * \param type
     *        the type of chassis, either STANDARD or HOLONOMIC
     */
    void set_type(ChassisType type);

    void configure_chassis_variables(double wheel_circumfrence, 
                                     double gear_ratio, 
                                     motor_gearset_e motor_gearbox);

    /**
     * Configures the motors for the motors in two groups. NOTE: motor directions can't be configured here
     * 
     * \param Left
     *        An array of motor pointers that are found on the left side of the chassis
     * \param num_L
     *        The number of motor pointers in the Left array
     * \param Right
     *        An array of motor pointers that are found on the right side of the chassis
     * \param num
     *        The number of motor pointers in the Right array
     */
    void configure_motors(Motor * Left[],  unsigned int num_L, 
                          Motor * Right[], unsigned int num_R);

    /**
     * Configures the motors for the motors in four groups. NOTE: motor directions can't be configured here
     * 
     * \param FrontLeft
     *        An array of motor pointers that are found on the left front side of the chassis
     * \param num_LF
     *        The number of motor pointers in the Fronteft array
     * \param BackLeft
     *        An array of motor pointers that are found on the left back side of the chassis
     * \param num_LB
     *        The number of motor pointers in the BackLeft array
     * \param FrontRight
     *        An array of motor pointers that are found on the right front side of the chassis
     * \param num_RF
     *        The number of motor pointers in the FrontRight array
     * \param BackRight
     *        An array of motor pointers that are found on the right back side of the chassis
     * \param num_RB
     *        The number of motor pointers in the BackRight array
     */
    void configure_motors(Motor * FrontLeft[],  unsigned int num_LF, 
                          Motor * BackLeft[],   unsigned int num_LB, 
                          Motor * FrontRight[], unsigned int num_RF, 
                          Motor * BackRight[],  unsigned int num_RB);

    /**
     * Sets the power of the drive motors based on direction, also known as Arcade drive
     * 
     * \param forward
     *        the forward direction power
     * \param sideways
     *        the sideways direction power
     * \param turn
     *        the turn direction power
     */
    void set_directional_power(double forward, double sideways, double turn);

    /**
     * Sets the power of the drive motors based for the left and right side seperately, also known as Tank drive
     * 
     * \param left
     *        the left side power
     * \param right
     *        the right side power
     */
    void set_differential_power(double left, double right);

    void tare_drive_motors();

    double average_drive_position();

    void drive_straight(double target, 
                        PIDConstants forward_constants, 
                        PIDConstants turn_constants, 
                        double max_power, 
                        double max_runtime,
                        double accuracy);

    void drive_to_position(Vector2D target,
                           PIDConstants forward_constants, 
                           PIDConstants turn_constants, 
                           double max_power, 
                           double max_runtime,
                           double accuracy);
};

#endif