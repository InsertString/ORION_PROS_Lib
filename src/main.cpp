#include "main.h"

// Robot Spesific Objects
Motor LeftFrontDrive(1);
Motor LeftBackDrive(2);
Motor RightFrontDrive(3, true);
Motor RightBackDrive(4, true);

Motor * LeftMotors[2] = {&LeftFrontDrive, &LeftBackDrive};
Motor * RightMotors[2] = {&RightFrontDrive, &RightBackDrive};

Imu imu(5);

Chassis chassis(&imu);

void initialize() {
	chassis.set_type(STANDARD);
	chassis.configure_chassis_variables(23.5, 1.0, MOTOR_GEARSET_18);
	chassis.configure_motors(LeftMotors, 2, RightMotors, 2);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	chassis.drive_straight(150, PIDConstants(1,0,0,2), PIDConstants(1,0,0,2), 127, 2000, 2);
}

void opcontrol() {

	while (true) {
		delay(10);
	}
}
