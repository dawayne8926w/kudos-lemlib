#include "main.h"
#include "lemlib/api.hpp"
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */



pros::Motor left_front_motor(18, pros::E_MOTOR_GEARSET_06, true); // port 6, blue gearbox, not reversed
pros::Motor left_middle_motor(19, pros::E_MOTOR_GEARSET_06, true); // port 6, blue gearbox, not reversed
pros::Motor left_back_motor(20, pros::E_MOTOR_GEARSET_06, true); // port 1, blue gearbox, not reversed
pros::Motor right_front_motor(16, pros::E_MOTOR_GEARSET_06, false); // port 9, blue gearbox, reversed
pros::Motor right_middle_motor(15, pros::E_MOTOR_GEARSET_06, false); // port 8, blue gearbox, reversed
pros::Motor right_back_motor(17, pros::E_MOTOR_GEARSET_06, false); // port 10, blue gearbox, reversed
pros::Motor cata_left(7, pros::E_MOTOR_GEARSET_36, true);
pros::Motor cata_right(8, pros::E_MOTOR_GEARSET_36, false);
pros::Motor intake_arm(14, pros::E_MOTOR_GEARSET_36, false);
pros::Motor intake(1, pros::E_MOTOR_GEARSET_06, true);
pros::Motor_Group cata({cata_left, cata_right});
pros::Controller master (pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_side_motors({left_front_motor, left_middle_motor, left_back_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_middle_motor, right_back_motor});
pros::Optical intake_optical_sensor(5);
pros::ADIDigitalOut right_wing(2);
pros::ADIDigitalOut Left_wing(3);

pros::Rotation cata_rot(9);
lemlib::Drivetrain_t drivetrain {
    &left_side_motors, // left drivetrain motors
    &right_side_motors, // right drivetrain motors
    15.4, // track width
    3.25, // wheel diameter
    360, // wheel rpm
};

pros::Rotation odom_right(10, false); // Port 10, not reversed
pros::Rotation odom_left(4, true); // Port 13, reversed
//pros::Imu inertial_sensor(11); // port 2
 
lemlib::TrackingWheel left_tracking_wheel(&odom_left, 2.75, -5.3125); // 2.75 tracking wheel diameter, -1 inches from tracking center
lemlib::TrackingWheel right_tracking_wheel(&odom_right, 2.75, 5.3125); // 2.75 tracking wheel diameter, 1 inches from tracking center
// odometry struct
lemlib::OdomSensors_t sensors {
    &left_tracking_wheel, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // we don't have a second tracking wheel, so we set it to nullptr
    nullptr, // inertial sensor
};
 
// forward/backward PID
lemlib::ChassisController_t lateralController {
    10, // kP
    30, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
};
 
// turning PID
lemlib::ChassisController_t angularController {
    4, // kP
    40, // kD
    1, // smallErrorRange
    100, // smallErrorTimeout
    3, // largeErrorRange
    500, // largeErrorTimeout
    40 // slew rate
};
 
 
// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	chassis.calibrate(); // calibrate the chassis
    chassis.setPose(0, 0, 0);
	pros::ADIDigitalOut right_wing();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void move_cata(int degrees, int velocity = 60){
    int error = 5;
    cata.move_relative(degrees, velocity);
    if (cata.tare_position()==1){
        pros::delay(1);
    }
    while(!((cata_right.get_position() < (degrees + error)) && (cata_right.get_position() > (degrees - error)))){
        pros::delay(5);
        //master.rumble("-*-");
        //std::vector<double> positions = cata.get_positions();
        master.print(0,0,"position: %f", cata_right.get_position());
        //std::cout<<positions[0];
    }
}

bool triball_detected(){
    float threshold = 200;
    return (intake_optical_sensor.get_proximity()> threshold);
}

void triball_sensor_test(){
    pros::c::optical_rgb_s_t rgb_value;
    while(true){
        rgb_value = intake_optical_sensor.get_rgb();
        master.clear();
        master.print(0,0,"Green: %lf", rgb_value.green);
        master.rumble("-*-");
        pros::delay(100);
    }
}

void reset_cata(){
    int limit = 8000;
    int target = 4750;
    cata.move_velocity(-60);
    while(cata_rot.get_position()>limit){
        pros::delay(10);
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            break;
        }
        master.rumble("- * -");
    }
    cata.brake();
    //pros::delay(500);
    move_cata((target-cata_rot.get_position())/5, 5);
}
void autonomous() {
    cata_rot.reset();

    //chassis.moveTo(10.0, 0.0, 10000);

    // CATAPULT MATH
    // Catapult is geared 2:1, so 720 degrees results in one rotation of the slip gear
    // Slip gear has 10 of 24 teeth, so 14 teeth are shaved down
    // 14/24 = .5833, so 58.33% of the slip gear is shaved
    // Given 720 degrees, 720*.5833 = 420 degrees to rotate past slip portion of slip gear. 
    //

    // Reset
    cata.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    //move_cata(-720);

    //pros::delay(10000000);

    // PSEUDOCODE
    // Extend intake
    //
    // Begin loop
    // Spin intake until triball detected with optical sensor
    // Once optical sensor stops detecting triball, delay to ensure triball is in catapult
    // Begin rotating cata, launching triball 
    // Stop spinning the intake to ensure no triballs get stuck under the cata arm
    // Continue rotating cata until end of slip gear reached, avoiding engaging gear teeth
    // Wait until catapult arm has settled using rotation sensor to ensure consistent range of motion
    // Rotate cata until catapult arm is almost completely down
    // Start spinning intake
    // Finish rotating cata back to primed position
    // End loop
    //triball_sensor_test();
    //pros::c::optical_rgb_s_t rgb_value;

    //pros::delay(1000000000);
    intake_arm.move_relative(-1300, 100); // Extend intake assembly
    intake.move_velocity(600); // Begin Spinning intake

    for (int i = 0; i < 23; i++){ // Loop for each matchload
        while (!triball_detected()){ // Wait for triball to be detected
            pros::delay(10);
        }
        //master.rumble("- * -"); // Rumble controller to signal triball detected
        pros::delay(400); // Delay to ensure triball is in catapult
        intake.brake();
        move_cata(-300); // Launch triball and rotate to the end of the slip

        pros::delay(500); // Wait for catapult to finish launching and settle.

        reset_cata();

        intake.move_velocity(600); // Begin intaking again
         // Complete reset
    }
    
    
    /*
    intake_arm.move_relative(-1200, 100);
    intake.move_velocity(-600);
    pros::delay(1000);
    cata.move_relative(-200, 50);
    pros::delay(1000);
    for(int i = 0; i<20; i++){
        
        cata.move_relative(-2400, 100);
        intake.move_velocity(-600);
        cata.move_relative(-1200, 100);
        pros::delay(500);
        cata.move_relative(-1200, 100);
        master.rumble("* - *");
        intake.brake();
        pros::delay(3000);
    }
    */
	
}

void opcontrol() {
    cata_rot.reset();
	while (true){
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        left_side_motors.move(leftY + rightX);
        right_side_motors.move(leftY - rightX);
        master.clear();
        master.print(1, 1, "rot: %d", cata_rot.get_angle());
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            intake.move_velocity(600);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            intake.move_velocity(-600);
        }
        else{
            intake.brake();
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            intake_arm.move_velocity(100);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            intake_arm.move_velocity(-100);
        }
        else{
            intake_arm.brake();
        }
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            cata.move_velocity(50);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            cata.move_velocity(-50);
        }
        else{
            cata.brake();
        }
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            reset_cata();
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){
            right_wing.set_value(true);
            Left_wing.set_value(true);
        }
        else {
            right_wing.set_value(false);
            Left_wing.set_value(false);
        }
    }
}