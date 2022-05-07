#include "main.h"

Chassis::Chassis() {
    // set pointers to null
    odom = NULL;
    imu = NULL;
    LeftMotors = NULL;
    RightMotors = NULL;
    LeftFrontMotors = NULL;
    LeftBackMotors = NULL;
    RightFrontMotors = NULL;
    RightBackMotors = NULL;
    // set ints to zero
    num_L = 0;
    num_R = 0;
    num_LF = 0;
    num_LB = 0;
    num_RF = 0;
    num_RB = 0;
    // set default chassis parameters
    wheel_circumfrence_cm = 0;
    gear_ratio = 0;
    motor_gearing = MOTOR_GEARSET_18;
}

Chassis::Chassis(Odom * odom) {
    // set pointers to null
    this->odom = odom;
    imu = NULL;
    LeftMotors = NULL;
    RightMotors = NULL;
    LeftFrontMotors = NULL;
    LeftBackMotors = NULL;
    RightFrontMotors = NULL;
    RightBackMotors = NULL;
    // set ints to zero
    num_L = 0;
    num_R = 0;
    num_LF = 0;
    num_LB = 0;
    num_RF = 0;
    num_RB = 0;
    // set default chassis parameters
    wheel_circumfrence_cm = 0;
    gear_ratio = 0;
    motor_gearing = MOTOR_GEARSET_18;
}

Chassis::Chassis(Imu * imu) {
    // set pointers to null
    odom = NULL;
    this->imu = imu;
    LeftMotors = NULL;
    RightMotors = NULL;
    LeftFrontMotors = NULL;
    LeftBackMotors = NULL;
    RightFrontMotors = NULL;
    RightBackMotors = NULL;
    // set ints to zero
    num_L = 0;
    num_R = 0;
    num_LF = 0;
    num_LB = 0;
    num_RF = 0;
    num_RB = 0;
    // set default chassis parameters
    wheel_circumfrence_cm = 0;
    gear_ratio = 0;
    motor_gearing = MOTOR_GEARSET_18;
}

void Chassis::set_type(ChassisType type) {
    this->type = type;
}

void Chassis::configure_chassis_variables(double wheel_circumfrence, double gear_ratio, motor_gearset_e motor_gearbox) {
    this->wheel_circumfrence_cm = wheel_circumfrence;
    this->gear_ratio = gear_ratio;
    this->motor_gearing = motor_gearbox;
}

void Chassis::configure_motors(Motor * Left[],  unsigned int num_L, Motor * Right[], unsigned int num_R) {
    LeftMotors = new Motor * [num_L];
    this->num_L = num_L;
    RightMotors = new Motor * [num_R];
    this->num_R = num_R;
    
    for (int i = 0; i < num_L; i++) {
        LeftMotors[i] = Left[i];
    }

    for (int i = 0; i < num_R; i++) {
        RightMotors[i] = Right[i];
    }
}

void Chassis::configure_motors(Motor * FrontLeft[],  unsigned int num_LF, Motor * BackLeft[],   unsigned int num_LB, Motor * FrontRight[], unsigned int num_RF, Motor * BackRight[],  unsigned int num_RB) {
    LeftFrontMotors = new Motor * [num_LF];
    this->num_LF = num_LF;
    LeftBackMotors = new Motor * [num_LB];
    this->num_LB = num_LB;
    RightFrontMotors = new Motor * [num_RF];
    this->num_RF = num_RF;
    RightBackMotors = new Motor * [num_RB];
    this->num_RB = num_RB;

    for (int i = 0; i < num_LF; i++) {
        LeftFrontMotors[i] = FrontLeft[i];
    }

    for (int i = 0; i < num_LF; i++) {
        LeftBackMotors[i] = BackLeft[i];
    }

    for (int i = 0; i < num_LF; i++) {
        RightFrontMotors[i] = FrontRight[i];
    }

    for (int i = 0; i < num_LF; i++) {
        RightBackMotors[i] = BackRight[i];
    }
}

void Chassis::set_directional_power(double forward, double sideways, double turn) {
    if (num_L == 0 || num_R == 0) {
        for (int i = 0; i < num_LF; i++) {
            LeftFrontMotors[i]->move(forward + sideways - turn);
        }

        for (int i = 0; i < num_LB; i++) {
            LeftBackMotors[i]->move(forward - sideways - turn);
        }

        for (int i = 0; i < num_RF; i++) {
            RightFrontMotors[i]->move(forward - sideways + turn);
        }

        for (int i = 0; i < num_RB; i++) {
            RightBackMotors[i]->move(forward + sideways + turn);
        }
    }
    else {
        for (int i = 0; i < num_L; i++) {
            LeftMotors[i]->move(forward - turn);
        }

        for (int i = 0; i < num_R; i++) {
            RightMotors[i]->move(forward + turn);
        }
    }
}

void Chassis::set_differential_power(double left, double right) {
    if (num_L == 0 || num_R == 0) {
        for (int i = 0; i < num_LF; i++) {
            LeftFrontMotors[i]->move(left);
        }

        for (int i = 0; i < num_LB; i++) {
            LeftBackMotors[i]->move(left);
        }

        for (int i = 0; i < num_RF; i++) {
            RightFrontMotors[i]->move(right);
        }

        for (int i = 0; i < num_RB; i++) {
            RightBackMotors[i]->move(right);
        }
    }
    else {
        for (int i = 0; i < num_L; i++) {
            LeftMotors[i]->move(left);
        }

        for (int i = 0; i < num_R; i++) {
            RightMotors[i]->move(right);
        }
    }
}

void Chassis::tare_drive_motors() {
    if (num_L == 0 || num_R == 0) {
        for (int i = 0; i < num_LF; i++) {
            LeftFrontMotors[i]->tare_position();
        }

        for (int i = 0; i < num_LB; i++) {
            LeftBackMotors[i]->tare_position();
        }

        for (int i = 0; i < num_RF; i++) {
            RightFrontMotors[i]->tare_position();
        }

        for (int i = 0; i < num_RB; i++) {
            RightBackMotors[i]->tare_position();
        }
    }
    else {
        for (int i = 0; i < num_L; i++) {
            LeftMotors[i]->tare_position();
        }

        for (int i = 0; i < num_R; i++) {
            RightMotors[i]->tare_position();
        }
    }
}

double Chassis::average_drive_position() {
    double sum_left = 0;
    int left_motor_total = 0;
    double average_left = 0;
    double cm_left = 0;

    double sum_right = 0;
    int right_motor_total = 0;
    double average_right = 0;
    double cm_right = 0;

    double gearbox_ratio = motor_gearing == MOTOR_GEARSET_18 ? 900 : motor_gearing == MOTOR_GEARSET_36 ? 1800 : 300;

    if (num_L == 0 || num_R == 0) {
        for (int i = 0; i < num_LF; i++) {
            sum_left += LeftFrontMotors[i]->get_position();
        }

        for (int i = 0; i < num_LB; i++) {
            sum_left += LeftBackMotors[i]->get_position();
        }

        for (int i = 0; i < num_RF; i++) {
            sum_right += RightFrontMotors[i]->get_position();
        }

        for (int i = 0; i < num_RB; i++) {
            sum_right += RightBackMotors[i]->get_position();
        }

        left_motor_total = num_LF + num_LB;
        right_motor_total = num_RF + num_RB;
    }
    else {
        for (int i = 0; i < num_L; i++) {
            sum_left += LeftMotors[i]->get_position();
        }

        for (int i = 0; i < num_R; i++) {
            sum_right += RightMotors[i]->get_position();
        }

        left_motor_total = num_L;
        right_motor_total = num_R;
    }

    average_left = sum_left / left_motor_total;
    average_right = sum_right / right_motor_total;

    cm_left =  (average_left / motor_gearing)  * gear_ratio * wheel_circumfrence_cm;
    cm_right = (average_right / motor_gearing) * gear_ratio * wheel_circumfrence_cm;

    return (cm_left + cm_right) / 2.0;
}

void Chassis::drive_straight(double target, PIDConstants forward_constants, PIDConstants turn_constants, double max_power, double max_runtime, double accuracy) {
    tare_drive_motors();
    
    forward_PID.set_variables(target, max_power, -max_power, 0);
    forward_PID.set_constants(forward_constants);

    if (imu != NULL && !imu->is_calibrating()) {
        imu->tare();
        turn_PID.set_variables(0, max_power, -max_power, 0);
    }
    else if (odom != NULL) {
        turn_PID.set_variables(odom->getAngle(), max_power, -max_power, 0);
    }
    turn_PID.set_constants(turn_constants);

    double error = target - average_drive_position();
    bool exit = 0;

    Timer runtime_timer;
    Timer exit_timer;

    while (exit == false && runtime_timer.delta_time() < max_runtime) {
        error = target - average_drive_position();

        if (fabs(error) < accuracy && exit_timer.delta_time() > 300) 
            exit = true;
        else 
            exit_timer.reset();

        if (imu != NULL  && !imu->is_calibrating())
            set_directional_power(forward_PID.output(average_drive_position()), 0, turn_PID.output(imu->get_rotation()));
        else if (odom != NULL)
            set_directional_power(forward_PID.output(average_drive_position()), 0, turn_PID.output(odom->getAngle()));
        else 
            set_directional_power(0,0,0);

        delay(20);
    }

    set_directional_power(0,0,0);
}

void Chassis::drive_to_position(Vector2D target, PIDConstants forward_constants, PIDConstants turn_constants, double max_power, double max_runtime, double accuracy) {
    forward_PID.set_variables(target.getHeadingBased(odom->rad_angle()).y, max_power, -max_power, 0);
    forward_PID.set_constants(forward_constants);
    sideways_PID.set_variables(target.getHeadingBased(odom->rad_angle()).x, max_power, -max_power, 0);
    sideways_PID.set_constants(forward_constants);
    turn_PID.set_variables(odom->getAngle(), max_power, -max_power, 0);
    turn_PID.set_constants(turn_constants);

    Vector2D error = target - odom->getPosition();
    bool exit = 0;

    Timer runtime_timer;
    Timer exit_timer;

    while (exit == false && runtime_timer.delta_time() < max_runtime) {
        error = target - odom->getPosition();

        if (error.getLength() < accuracy && exit_timer.delta_time() > 300) 
            exit = true;
        else 
            exit_timer.reset();
        
        set_directional_power(forward_PID.output(odom->getPosition().getHeadingBased(odom->rad_angle()).y), sideways_PID.output(odom->getPosition().getHeadingBased(odom->rad_angle()).x), turn_PID.output(odom->getAngle()));

        delay(20);
    }

    set_directional_power(0,0,0);
}
