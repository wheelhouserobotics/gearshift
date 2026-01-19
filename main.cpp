 #include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
// #include "pros/optical.h"


// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-18, 16, -19},
                           pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({10, -9, 8}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)


// Inertial Sensor on port 10
pros::Imu imu(15);
pros::Distance DistX(20);
pros::Distance DistY(2);



//Motors
pros::Motor Intake17(17); //INTAKE
pros::Motor Outtake14(14); //OUTTAKE



//PNEUMATICS
pros::adi::Pneumatics middleP('C', false);
pros::adi::Pneumatics highP('D', false);
pros::adi::Pneumatics leftW('G', false);
pros::adi::Pneumatics rightW('H', false);


// tracking wheels


// horizontal pod encoder -- not used, so port 1
pros::Rotation horizontalEnc(6);
// vertical pod encoder -- port 5
pros::Rotation verticalEnc(5);
// horizontal pod -- not used, so random settings
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_2, 0);
// vertical pod -- 2" wheel, no offset
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_275, 2.5);


// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                             &rightMotors, // right motor group
                             12.694518, // 10 inch track width
                             lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                             400, // drivetrain rpm is 400
                             2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);


// lateral motion controller
lemlib::ControllerSettings linearController(8.5, // proportional gain (kP)
                                           0, // integral gain (kI)
                                           10, // derivative gain (kD)
                                           3, // anti windup
                                           1, // small error range, in inches
                                       100, // small error range timeout, in milliseconds
                                           3, // large error range, in inches
                                           500, // large error range timeout, in milliseconds
                                           0 // maximum acceleration (slew)
);


// angular motion controller
lemlib::ControllerSettings angularController(5.25, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            40, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in degrees
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in degrees
                                            500, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);


// sensors for odometry
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel
                           nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                           &horizontal, // horizontal tracking wheel
                           nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                           &imu // inertial sensor
);



// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                    10, // minimum output where drivetrain will move out of 127
                                    1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                 10, // minimum output where drivetrain will move out of 127
                                 1.019 // expo curve gain
);



// create the chassis
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


/**
* Runs initialization code. This occurs as soon as the program is started.
*
* All other competition modes are blocked by initialize; it is recommended
* to keep execution time for this mode under a few seconds.
*/
void initialize() {
   pros::lcd::initialize(); // initialize brain screen
   chassis.calibrate(); // calibrate sensors



   // the default rate is 50. however, if you need to change the rate, you
   // can do the following.
   // lemlib::bufferedStdout().setRate(...);
   // If you use bluetooth or a wired connection, you will want to have a rate of 10ms



   // for more information on how the formatting for the loggers
   // works, refer to the fmtlib docs

   // thread to for brain screen and position logging
   pros::Task screenTask([&]() {
       while (true) {
           // print robot location to the brain screen
           pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
           pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
           pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
           // log position telemetry
           lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
           // delay to save resources
           
           pros::delay(50);

           pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
           pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
           pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading

       }
   });
}



/**
* Runs while the robot is disabled
*/
void disabled() {}


float getDistX = DistX.get_distance();
float getDistY = DistY.get_distance();


/**
* runs after initialize if the robot is connected to field control
*/
void competition_initialize() {}


// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy


/**
* Runs during auto
*
* This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
*/
void autonomous() {  
   middleP.extend();
   highP.retract();
   leftW.retract();
   rightW.retract();
   // Intake17.move(100);
   // chassis.moveToPoint(-24,-23,1000);
   // chassis.moveToPose(-22,22,0,1);
   // chassis.moveToPose(-9,10,180, 1);



//pid tuning code
   chassis.setPose(0,0,0); // peak.
   // chassis.moveToPoint(0, 20, 100000,{.maxSpeed = 20});
   //chassis.turnToHeading(90, 100000);


//skills auton`
    //kids approach
    // chassis.setPose(0,0,0);
    // chassis.moveToPoint(0, 23, 2000, {.maxSpeed = 60});
    //   pros::delay(250);
    // chassis.moveToPoint(-37.8,20,4000, {.maxSpeed = 70});
    //   pros::delay(250);
    // chassis.turnToHeading(180, 2000);
    // chassis.setPose(getDistX, getDistY, 180);
    // leftW.extend();
    // rightW.extend();
    // chassis.moveToPoint(-37.8,6.4,2000, { .maxSpeed = 75});
    // chassis.moveToPoint(-37.8,8,2000, {.forwards = false, .maxSpeed = 75});
    // chassis.moveToPoint(-37.8,6.4,2000, { .maxSpeed = 75});
    // Intake17.move(127);
    //   pros::delay(3000);
    // Intake17.move(0);
    //chassis.moveToPoint(-37.8,45,4000, {.forwards = false, .maxSpeed = 75});
    
    //uncle approach
    //chassis.moveToPoint()


    //highP.extend();
    //pros::delay(200);
    //Intake17.move(127);
    //Outtake14.move(-127);
      //pros::delay(5000);
    /*
    chassis.moveToPoint(-37.5,20,4000, {.forwards = true,.maxSpeed = 70});
    chassis.turnToHeading(90, 2000);
    chassis.moveToPoint(95,20,2000, {.maxSpeed = 60});
      pros::delay(250);
    */
    
    // 
    // Intake17.move(127);
    // 
    // chassis.moveToPoint(-4.76,51, 4000, {.maxSpeed = 17.5});
    //   
    //  Intake17.move(0);
    // 
    //   pros::delay(250);
    // leftW.extend();
    // rightW.extend();
    // 
    // 
    // 




    // pros::delay(1500);
    //   chassis.moveToPoint(-42.04,46.5,4000, {.forwards = false, .maxSpeed = 75});
    //   highP.extend();
    //   Intake17.move(127);
    //   Outtake14.move(-127);
    //   pros::delay(4000);
    //   chassis.moveToPoint(-42.04,40,1000);
    //   highP.retract();
    //   chassis.moveToPoint(-42.04,46.5,1000, {.forwards = false});


    // chassis.moveToPoint(-4.76,41.56, 4000, {.maxSpeed = 70});
    // chassis.moveToPoint(-4.76,46.56, 4000, {.maxSpeed = 25});
    //  Intake17.move(127);
    //   pros::delay(2000);
    //  Intake17.move(0);
    //  chassis.moveToPoint(-34.04,23,4000, {.maxSpeed = 70});
    //   pros::delay(500);
    // chassis.moveToPoint(-34.04,7.75,3000, {.maxSpeed = 70});
    // chassis.turnToHeading(180, 4000);
    //  leftW.extend();
    //  rightW.extend();
    //  Intake17.move(127);


    //   pros::delay(3000);
    // chassis.moveToPoint(-34.04,19.37,4000, {.forwards = false, .maxSpeed = 70});
    //  Intake17.move(0);
    // chassis.turnToHeading(180, 4000);
    //  Intake17.move(127);
    //  Outtake14.move(-127);
    //   pros::delay(3000);


    // middleP.extend();
    // Intake17.move(127);
    // chassis.moveToPoint(-4.76,45, 4000, {.maxSpeed = 55});
    // chassis.moveToPoint(-4.76,54, 4000, {.maxSpeed = 20});
    //   pros::delay(1250);
    //  Intake17.move(0);
    // chassis.moveToPoint(-37.04,23,4000, {.maxSpeed = 70});
    //   pros::delay(1500);
    // chassis.turnToHeading(180, 4000);
    //  leftW.extend();
    //  rightW.extend();
    //  middleP.extend();
    //  Intake17.move(127);
    // chassis.moveToPoint(-37.04,9.75,4000,{.maxSpeed = 95});
    // pros::delay(1500);
    // chassis.moveToPoint(-37.04,46.5,4000, {.forwards = false, .maxSpeed = 75});
    //  Intake17.move(0);
    // chassis.turnToHeading(180, 4000);
    // highP.extend();
    //  Intake17.move(127);
    //  Outtake14.move(-127);
    //   pros::delay(3000);
    //   chassis.moveToPoint(-37.04,23,1000);
    //   pros::delay(100);
    //   highP.retract();
    //   Intake17.move(0);
    //  Outtake14.move(0);
    //   chassis.moveToPoint(-37.04,46.5,1000, {.forwards = false});



// right long goal -- 7 block -- pre + mid + loader -- slot 1: TUNE DONING


    // middleP.extend();
    // Intake17.move(127);
    // chassis.moveToPoint(4.76,45, 4000, {.maxSpeed = 55});
    // chassis.moveToPoint(4.76,51, 4000, {.maxSpeed = 20});
    // pros::delay(2500);
    // Intake17.move(0);
    // chassis.moveToPoint(46.04,23,4000, {.maxSpeed = 70});
    //   pros::delay(3000);
    // chassis.turnToHeading(180, 4000);
    //  leftW.extend();
    //  rightW.extend();
    //  middleP.extend();
    //  Intake17.move(127);
    // chassis.moveToPoint(46.06,7.75,4000);
    // pros::delay(3000);
    // chassis.moveToPoint(48.04,46.5,4000, {.forwards = false, .maxSpeed = 75});
    //  Intake17.move(0);
    // chassis.turnToHeading(180, 4000);
    // highP.extend();
    // middleP.extend();
    // Intake17.move(-127);
    // pros::delay(750);
    //  Intake17.move(127);
    //  Outtake14.move(-127);
    //   pros::delay(3000);




    //good here
    // middleP.extend();
    // Intake17.move(127);
    // chassis.moveToPoint(4.76,45, 4000, {.maxSpeed = 55});
    // chassis.moveToPoint(4.76,54, 4000, {.maxSpeed = 20});
    //   pros::delay(1250);
    //  Intake17.move(0);
    // chassis.moveToPoint(46.04,25,4000, {.maxSpeed = 70});
    //   pros::delay(1500);
    // chassis.turnToHeading(180, 4000);
    //  leftW.extend();
    //  rightW.extend();
    //  middleP.extend();
    //  Intake17.move(127);
    // chassis.moveToPoint(46.04,6.35,4000);
    // pros::delay(1500);
    // highP.retract();
    // chassis.moveToPoint(46.04,46.5,4000, {.forwards = false, .maxSpeed = 75});
    //  Intake17.move(0);
    // chassis.turnToHeading(180, 4000);
    // highP.extend();
    //  Intake17.move(127);
    //  Outtake14.move(-127);
    //   pros::delay(3000);
    //   chassis.moveToPoint(46.04,23,1000);
    //   pros::delay(100);
    //   highP.retract();
    //   Intake17.move(0);
    //  Outtake14.move(0);




// ---------------------------------------




// left long goal -- 7 block -- pre + mid + loader -- slot 2: DONE TUNING


    // middleP.extend();
    // Intake17.move(127);
    // chassis.moveToPoint(-4.76,45, 4000, {.maxSpeed = 60});
    // chassis.moveToPoint(-4.76,51, 4000, {.maxSpeed = 17.5});
    //   pros::delay(250);
    //  Intake17.move(0);
    // chassis.moveToPoint(-42.04,23,4000, {.maxSpeed = 70});
    //   pros::delay(250);
    // chassis.moveToPoint(-42.04,7.75,4000, { .maxSpeed = 75});
    // chassis.turnToHeading(180, 2000);
    // Intake17.move(127);
    // leftW.extend();
    // rightW.extend();
    // pros::delay(1500);
    //   chassis.moveToPoint(-42.04,46.5,4000, {.forwards = false, .maxSpeed = 75});
    //   highP.extend();
    //   Intake17.move(127);
    //   Outtake14.move(-127);
    //   pros::delay(4000);
    //   chassis.moveToPoint(-42.04,40,1000);
    //   highP.retract();
    //   chassis.moveToPoint(-42.04,46.5,1000, {.forwards = false});


    // chassis.moveToPoint(-4.76,41.56, 4000, {.maxSpeed = 70});
    // chassis.moveToPoint(-4.76,46.56, 4000, {.maxSpeed = 25});
    //  Intake17.move(127);
    //   pros::delay(2000);
    //  Intake17.move(0);
    //  chassis.moveToPoint(-34.04,23,4000, {.maxSpeed = 70});
    //   pros::delay(500);
    // chassis.moveToPoint(-34.04,7.75,3000, {.maxSpeed = 70});
    // chassis.turnToHeading(180, 4000);
    //  leftW.extend();
    //  rightW.extend();
    //  Intake17.move(127);


    //   pros::delay(3000);
    // chassis.moveToPoint(-34.04,19.37,4000, {.forwards = false, .maxSpeed = 70});
    //  Intake17.move(0);
    // chassis.turnToHeading(180, 4000);
    //  Intake17.move(127);
    //  Outtake14.move(-127);
    //   pros::delay(3000);




    //good here
    // middleP.extend();
    // Intake17.move(127);
    // chassis.moveToPoint(-4.76,45, 4000, {.maxSpeed = 55});
    // chassis.moveToPoint(-4.76,54, 4000, {.maxSpeed = 20});
    //   pros::delay(1250);
    //  Intake17.move(0);
    // chassis.moveToPoint(-37.04,23,4000, {.maxSpeed = 70});
    //   pros::delay(1500);
    // chassis.turnToHeading(180, 4000);
    //  leftW.extend();
    //  rightW.extend();
    //  middleP.extend();
    //  Intake17.move(127);
    // chassis.moveToPoint(-37.04,9.75,4000,{.maxSpeed = 95});
    // pros::delay(1500);
    // chassis.moveToPoint(-37.04,46.5,4000, {.forwards = false, .maxSpeed = 75});
    //  Intake17.move(0);
    // chassis.turnToHeading(180, 4000);
    // highP.extend();
    //  Intake17.move(127);
    //  Outtake14.move(-127);
    //   pros::delay(3000);
    //   chassis.moveToPoint(-37.04,23,1000);
    //   pros::delay(100);
    //   highP.retract();
    //   Intake17.move(0);
    //  Outtake14.move(0);
    //   chassis.moveToPoint(-37.04,46.5,1000, {.forwards = false});








// ---------------------------------------


// skills rahhhh


    // middleP.extend();
    // Intake17.move(127);
    // chassis.moveToPoint(4.76,45, 4000, {.maxSpeed = 55});
    // chassis.moveToPoint(4.76,51, 4000, {.maxSpeed = 20});
    //   pros::delay(2500);
    //  Intake17.move(0);
    // chassis.moveToPoint(46.04,23,4000, {.maxSpeed = 70});
    //   pros::delay(3000);
    //   chassis.moveToPoint(48.04,46.5,4000, {.forwards = false, .maxSpeed = 75});
    //  Intake17.move(0);
    // chassis.turnToHeading(180, 4000);
    // highP.extend();
    // middleP.extend();
    // Intake17.move(-127);
    // pros::delay(750);
    //  Intake17.move(127);
    //  Outtake14.move(-127);
    //  //4 blocks
    //   pros::delay(3000);
    // chassis.turnToHeading(180, 4000);
    //  leftW.extend();
    //  rightW.extend();
    //  middleP.extend();
    //  Intake17.move(127);
    // chassis.moveToPoint(46.06,7.75,4000);
    // pros::delay(3000);
    // chassis.moveToPoint(48.04,46.5,4000, {.forwards = false, .maxSpeed = 75});
    //  Intake17.move(0);
    // chassis.turnToHeading(180, 4000);
    // highP.extend();
    // middleP.extend();
    // Intake17.move(-127);
    // pros::delay(750);
    //  Intake17.move(127);
    //  Outtake14.move(-127);
    //   pros::delay(3000);
    // chassis.moveToPoint(46.04,23,4000, {.maxSpeed = 70});
    // //match load




// ---------------------------------------








// 4+3_low_long_right -- 4+3 block -- pre + mid + loader -- slot 3: DONE TUNING




//    chassis.moveToPoint(4.76,39.56,4000, {.maxSpeed = 60}); // go to block
//    Intake17.move(100); // intake mid blocks
//    pros::delay(2000); // pause 2 sec
//    Intake17.move(0); // stop intake
//    chassis.moveToPoint(-2.18, 41, 4000, {.maxSpeed = 60}); // turn into mid goal
//    chassis.turnToHeading(-45, 2000); // turn to heading 45 for low goal
//    Intake17.move(-80); // reverse intake for low goal scoring
//    pros::delay(2000); // pause 2 sec
//    Intake17.move(0); // stop intake
//    chassis.moveToPoint(34.04,10.18,4000, {.maxSpeed = 60}); // line up for matchload
//    leftW.extend();
//    rightW.extend();
//    chassis.moveToPoint(34.04,0.18,4000, {.maxSpeed = 40}); // go into matchloader
//    pros::delay(2500);
//    chassis.turnToHeading(180, 4000);
//    pros::delay(300);
//    chassis.moveToPoint(34.04,19.37,4000, {.forwards = false, .maxSpeed = 60});
//    Intake17.move(0);
//    chassis.turnToHeading(180, 4000);
//    highP.extend();
//    Intake17.move(120);
//    Outtake14.move(120);
//    pros::delay(3000);




// 4+3_upper_long_right -- 4+3 block -- pre + mid + loader -- slot 4: DONE TUNING




//    Intake17.move(100); // intake mid blocks
//    chassis.moveToPoint(-4.76,39.56,4000, {.maxSpeed = 60}); // go to block
//    pros::delay(2000); // pause 2 sec
//    chassis.moveToPoint(2.18, 41, 4000, {.maxSpeed = 60}); // turn into mid goal
//    chassis.turnToHeading(-135, 2000); // turn to heading 45 for low goal
//    pros::delay(2000);
//    middleP.retract();
//    pros::delay(2000); // pause 2 sec
//    Intake17.move(0); // stop intake
//    chassis.moveToPoint(-34.04,10.18,4000, {.maxSpeed = 60}); // line up for matchload
//    chassis.moveToPoint(-34.04,0.18,4000, {.maxSpeed = 40}); // go into matchloader
//    leftW.extend();
//    rightW.extend();
//    middleP.extend(); // close mid goal piston
//    pros::delay(2500);
//    chassis.turnToHeading(180, 4000);
//    Intake17.move(120);
//    pros::delay(3000);
//    Intake17.move(0);
//    chassis.moveToPoint(-34.04,19.37,4000, {.forwards = false, .maxSpeed = 60});
//    chassis.turnToHeading(180, 4000);
//    highP.extend();
//    Intake17.move(120);
//    pros::delay(250);
//    Outtake14.move(-120);






//sawp auto v5
// chassis.setPose(12,23,-90);
// middleP.extend();
//     Intake17.move(127);
//     chassis.moveToPoint(-52.04,25,2000, {.maxSpeed = 90});
//       pros::delay(1500);
//     chassis.turnToHeading(180, 2000);
//     leftW.extend();
//     rightW.extend();
//     chassis.moveToPoint(-50.04, 4.25, 2000);
//     Intake17.move(127);
//     pros::delay(1500);
//       chassis.moveToPoint(-52.54,46.5,2000, {.forwards = false, .maxSpeed = 95});
//       leftW.retract();
//       rightW.retract();
//       highP.extend();
//       pros::delay(1000);
//       Intake17.move(127);
//       Outtake14.move(-127);
//       pros::delay(2000);
//       chassis.moveToPoint(-52.54,37,1000, { .maxSpeed = 95});
//       chassis.moveToPoint(-42.04,40,2000);
//       pros::delay(1000);
//       chassis.moveToPoint(-46.04,36,2000);
//       highP.retract();
//       middleP.retract();
//       Intake17.move(127);
//       Outtake14.move(127);
//       pros::delay(1000);
//       chassis.moveToPoint(-42.04,40,2000);






    //   chassis.moveToPoint(-12.76,59, 2000, {.maxSpeed = 85});
    //   chassis.moveToPoint(44.5,59, 2000, {.maxSpeed = 85});
    //   chassis.moveToPoint(40.28,68, 2000, {.maxSpeed = 85});
    //   chassis.turnToHeading(-45,1000);
    //   pros::delay(1000);
    //   Intake17.move(-127);












// sawp auto v4


    // middleP.extend();
    // Intake17.move(127);
    // chassis.moveToPoint(-4.76,45, 2000);
    // pros::delay(1250);
    // chassis.moveToPoint(29.76,45, 2000);
    //   pros::delay(1250);
    //   chassis.moveToPoint(25.76,59, 2000, {.maxSpeed = 70});
    //   chassis.turnToHeading(-45, 750);
    //   Intake17.move(-127);
    //   pros::delay(1250);
    //  Intake17.move(0);
    // chassis.moveToPoint(-37.04,23,2000);
    //   pros::delay(1500);
    // chassis.turnToHeading(180, 2000);
    //  leftW.extend();
    //  rightW.extend();
    //  middleP.extend();
    //  Intake17.move(127);
    // chassis.moveToPoint(-37.04,7.75,2000);
    // pros::delay(1500);
    // chassis.moveToPoint(-37.04,46.5,2000, {.forwards = false});
    //  Intake17.move(0);
    // chassis.turnToHeading(180, 2000);
    // highP.extend();
    //  Intake17.move(127);
    //  Outtake14.move(-127);
    //   pros::delay(1500);
    //   chassis.moveToPoint(-37.04,23,1000);
    //   pros::delay(100);
    //   highP.retract();
    //   Intake17.move(0);
    //  Outtake14.move(0);
    //   chassis.moveToPoint(-37.04,46.5,1000, {.forwards = false});




// sawp auto v3


   
    // chassis.setPose(0,0,-90);
    // middleP.extend();
    // chassis.moveToPoint(-48, 0 , 1000,{.maxSpeed = 90});
    // Intake17.move(127);
    // chassis.turnToHeading(180, 100,{.maxSpeed = 55});
    // leftW.extend();
    // rightW.extend();
    // chassis.moveToPoint(-48, -12, 1000,{.maxSpeed = 90});
    // chassis.turnToHeading(180, 120,{.maxSpeed = 55});
    // leftW.retract();
    // rightW.retract();
    // pros::delay(1000);
    // chassis.turnToHeading(180, 120,{.maxSpeed = 55});
    // chassis.moveToPoint(-48,10,1000, {.forwards = false, .maxSpeed = 90});
    // chassis.turnToHeading(180, 120,{.maxSpeed = 55});
    // highP.extend();
    // Intake17.move(127);
    // Outtake14.move(-127);
    // pros::delay(1200);
    // chassis.moveToPoint(-48, 0 , 1000,{.maxSpeed = 90});
    // Outtake14.move(0);
    // highP.retract();
    // chassis.turnToHeading(45,120,{.maxSpeed = 55});
    // chassis.moveToPoint(-24,26,1000,{.maxSpeed = 90});
    // pros::delay(250);
    // chassis.moveToPoint(24,26,1500,{.maxSpeed = 90});
    // pros::delay(250);
    // chassis.turnToHeading(-45, 120,{.maxSpeed = 55});
    // Intake17.move(-100);
    // pros::delay(1000);
    // chassis.moveToPoint(52, 0, 1200,{.maxSpeed = 90});
    // Intake17.move(127);
    // chassis.turnToHeading(180, 120,{.maxSpeed = 55});
    // leftW.extend();
    // rightW.extend();
    // chassis.moveToPoint(52, -12, 1200,{.maxSpeed = 90});
    // pros::delay(1000);
    // leftW.retract();
    // rightW.retract();
    // chassis.moveToPoint(52,10,1200, {.forwards = false, .maxSpeed = 90});
    // highP.extend();
    // Intake17.move(127);
    // Outtake14.move(-127);
    // pros::delay(1200);










// sawp auto right v2:




//    chassis.moveToPoint(4.76,39.56,4000, {.maxSpeed = 80}); // go to block
//    Intake17.move(100); // intake mid blocks
//    pros::delay(1000); // pause 2 sec
//    Intake17.move(0); // stop intake
//    chassis.moveToPoint(-2.18, 41, 1500, {.maxSpeed = 75}); // turn into mid goal
//    chassis.turnToHeading(-45, 750);
//    Intake17.move(-100); // reverse intake for low goal scoring
//    pros::delay(100); // pause 2 sec
//    Intake17.move(0); // stop intake
//    chassis.moveToPoint(34.04,10.18,2000, {.maxSpeed = 80}); // line up for matchload
//    chassis.moveToPoint(34.04,0.18,2000, {.maxSpeed = 40}); // go into matchloader
//    leftW.extend();
//    rightW.extend();
//    pros::delay(2000);
//    chassis.turnToHeading(180, 1000);
//    pros::delay(250);
//    chassis.moveToPoint(34.04,19.37,2000, {.forwards = false, .maxSpeed = 70});
//    Intake17.move(0);
//    chassis.turnToHeading(180, 1000);
//    highP.extend();
//    Intake17.move(120);
//    Outtake14.move(120);
//    pros::delay(2500);
//    Outtake14.move(0);
//    leftW.retract();
//    rightW.retract();
//    highP.retract();
//    chassis.moveToPoint(34.04,10,2000, {.forwards = false, .maxSpeed = 70});
//    pros::delay(250);
//    Intake17.move(100);
//    chassis.moveToPoint(-37.76,39.56,2750, {.maxSpeed = 90}); // go to block
//    pros::delay(1000); // pause 2 sec
//    chassis.moveToPoint(-65.04,10.18,2500, {.maxSpeed = 70}); // line up for matchload
//    chassis.moveToPoint(-65.04,0.18,2500, {.maxSpeed = 40}); // go into matchloader
//    leftW.extend();
//    rightW.extend();
//    middleP.extend(); // close mid goal piston
//    pros::delay(250);
//    chassis.turnToHeading(180, 1000);
//    Intake17.move(120);
//    pros::delay(2500);
//    Intake17.move(0);
//    chassis.moveToPoint(-65.04,19.37,2000, {.forwards = false, .maxSpeed = 70});
//    chassis.turnToHeading(180, 4000);
//    highP.extend();
//    Intake17.move(120);
//    pros::delay(250);
//    Outtake14.move(-120);
   




// sawp auto right v1:
//    chassis.setPose(0,0,0);
//    chassis.moveToPoint(37, 36, 4000,{.maxSpeed = 60});
//    Intake17.move(100);
//    chassis.moveToPoint(50, 0, 4000,{.maxSpeed = 60});
//    Intake17.move(0);
//    chassis.moveToPoint(50,-5,4000,{.maxSpeed = 60});
//    chassis.turnToHeading(181, 4000);
//    leftW.extend();
//    rightW.extend();
//    chassis.turnToHeading(180, 3000);
//    pros::delay(2000);
//    chassis.moveToPoint(50,20,4000,{.forwards = false, .maxSpeed = 60});
//    highP.extend();
//    Intake17.move(127);
//    Outtake14.move(127);
//    pros::delay(2000);
//    highP.retract();
//    Intake17.move(0);
//    Outtake14.move(0);
//    chassis.moveToPoint(50, 0, 4000,{.maxSpeed = 60});
//    Intake17.move(100);
//    chassis.moveToPoint(18, 35, 4000,{.maxSpeed = 60});
//    Intake17.move(0);
//    chassis.turnToHeading(205,3000);
//    middleP.retract();
//    Intake17.move(127);
























// testing:
   // chassis.setPose(0,0,0);
   // chassis.moveToPoint(-29.682, 13.587, 3000,{.maxSpeed = 60});
   // chassis.moveToPoint(-6.48, 36.58, 3000,{.maxSpeed = 60});
   // chassis.moveToPoint(9.197, 36.371, 3000,{.maxSpeed = 60});
   // chassis.moveToPoint(-25.293, 36.58, 3000,{.maxSpeed = 60});
   // chassis.moveToPoint(-24.038, -33.863, 3000,{.maxSpeed = 60});
   // chassis.moveToPoint(-40.761, -22.575, 3000,{.maxSpeed = 60});
















// unused:








// right mid goal -- both middle goals:
   // chassis.setPose(0,0,0);
   // chassis.moveToPoint(37, 36, 4000,{.maxSpeed = 60});
   // Intake17.move(100);
   // chassis.moveToPoint(18, 36, 4000,{.maxSpeed = 60});
   // Intake17.move(0);
   // chassis.turnToHeading(205,3000);
   // middleP.retract();
   // Intake17.move(127);








// left mid goal auto:
   // chassis.setPose(0,0,0);
   // chassis.moveToPoint(-37, 35, 4000,{.maxSpeed = 60});
   // Intake17.move(100);
   // chassis.moveToPoint(-18, 35, 4000,{.maxSpeed = 60});
   // chassis.moveToPoint(-37, 35, 4000,{.maxSpeed = 60});
   // Intake17.move(0);
   // chassis.turnToHeading(-155,3000);
   // middleP.retract();
   // Intake17.move(127);








// sawp auto left:
   // chassis.setPose(0,0,0);
   // chassis.moveToPoint(-37, 36, 4000,{.maxSpeed = 60});
   // Intake17.move(100);
   // chassis.moveToPoint(-50, 0, 4000,{.maxSpeed = 60});
   // Intake17.move(0);
   // chassis.moveToPoint(-50,-5,4000,{.maxSpeed = 60});
   // chassis.turnToHeading(181, 4000);
   // leftW.extend();
   // rightW.extend();
   // chassis.turnToHeading(180, 3000);
   // pros::delay(2000);
   // chassis.moveToPoint(-50,20,4000,{.forwards = false, .maxSpeed = 60});
   // highP.extend();
   // Intake17.move(127);
   // Outtake14.move(127);
   // pros::delay(2000);
   // highP.retract();
   // Intake17.move(0);
   // Outtake14.move(0);
   // chassis.moveToPoint(-50, 0, 4000,{.maxSpeed = 60});
   // Intake17.move(100);
   // chassis.moveToPoint(-37, 35, 4000,{.maxSpeed = 60});
   // Intake17.move(0);
   // chassis.turnToHeading(-155,3000);
   // middleP.retract();
   // Intake17.move(127);








};
















/**
* Runs in driver control
*/
void opcontrol() {
   // controller
   // loop to continuously update motors














   chassis.setPose(0,0,0);
   bool midToggle = true;
   bool topToggle = false;
   bool ToggleWill = true;
















   while (true) {
       // get joystick positions
       int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
       int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
       // move the chassis with curvature drive
       chassis.arcade(leftY, rightX);
       // delay to save resources
     
       if (controller.get_digital(DIGITAL_R1)){ // BOTTOM INTAKE SPIN NORMAL
           Intake17.move(110);
       }
       if (controller.get_digital(DIGITAL_R2)){ // BOTTOM INTAKE SPIN REVERSE
           Intake17.move(-100);
       }


       if (controller.get_digital(DIGITAL_L1)){ // TOP INTAKE SPIN NORMAL
            if(topToggle == false || topToggle == true){
                Outtake14.move(-110);
            }
           
       }
       if (controller.get_digital(DIGITAL_L2)){ // TOP INTAKE SPIN REVERSE
           if(topToggle == false || topToggle){
                Outtake14.move(100);
           }
       }


       if(controller.get_digital(DIGITAL_A)){ // ALL INTAKE KILL
           Intake17.move(0);
           Outtake14.move(0);
       }


       if (controller.get_digital(DIGITAL_X)){ // TOP GOAL PISTONS
           if (midToggle==true || midToggle==false){
               middleP.extend();
               midToggle = false;
           }
           if (topToggle==true || midToggle==false){
               highP.extend();
               topToggle = false;
           }
       }


       if (controller.get_digital(DIGITAL_B)){ // MID GOAL PISTONS
           if (midToggle==false || midToggle==true){
               middleP.retract();
               midToggle = true;
           }
           if (topToggle==false || midToggle==true){
               highP.retract();
               topToggle = true;
           }
           }
       if (controller.get_digital(DIGITAL_Y)){ // STORAGE PISTONS
           if (midToggle==true || midToggle==false || topToggle==true || topToggle==false){
               middleP.extend();
               highP.retract();
           }
        }
        if(controller.get_digital(DIGITAL_UP)){ //Little Will
           if(ToggleWill == true){
               leftW.extend();
               rightW.extend();
           }
        }
        if(controller.get_digital(DIGITAL_DOWN)){ //Little Will
           if(ToggleWill == true){
               leftW.retract();
               rightW.retract();
           }


       }
    }
     




//         if(controller.get_digital(DIGITAL_LEFT) && chassis.getPose().y <= 15){ //Auto Aliner/Adaptive Button Test *works yay
//             chassis.turnToHeading(180, 1000);
//         }else if (controller.get_digital(DIGITAL_LEFT) && chassis.getPose().y >= 50) {
//             chassis.turnToHeading(0, 1000);
//         }else if (controller.get_digital(DIGITAL_LEFT) && (chassis.getPose().y >=15 && chassis.getPose().y <= 50)){
//             chassis.turnToHeading(45, 1000);
//         }


    //    if (controller.get_digital(DIGITAL_R1)){ // BOTTOM INTAKE SPIN NORMAL
    //         Intake17.move(127);
    //    }
    //    else if (controller.get_digital(DIGITAL_R2)){ // BOTTOM INTAKE SPIN REVERSE
    //         Intake17.move(-100);
    //    }else{
    //         highP.retract();
    //         middleP.extend();
    //         Intake17.move(0);
    //         Outtake14.move(0);
    //    }
    //    if (controller.get_digital(DIGITAL_L1)){ // TOP GOAL OUTTAKE
    //         highP.extend();
    //         Intake17.move(127);
    //         Outtake14.move(-127);
    //    }
       
    //    if (controller.get_digital(DIGITAL_L2)){ // TOP GOAL OUTTAKE
    //         middleP.retract();
    //         Intake17.move(127);
    //    }
       


    //     if(controller.get_digital(DIGITAL_UP)){ //Little Will
    //        if(ToggleWill == true){
    //            leftW.extend();
    //            rightW.extend();
    //        }
    //    }


    //     if(controller.get_digital(DIGITAL_DOWN)){ //Little Will
    //        if(ToggleWill == true){
    //            leftW.retract();
    //            rightW.retract();
    //        }
    //    }


       
    //    if(controller.get_digital(DIGITAL_A) && chassis.getPose().y <= 28){ //Auto Aliner/Adaptive Button Test *works yay
    //         chassis.turnToHeading(180, 500,{.maxSpeed = 75});
    //     }else if (controller.get_digital(DIGITAL_A) && (chassis.getPose().y >= 28 && chassis.getPose().y <= 37)){
    //         chassis.turnToHeading(225, 500,{.maxSpeed = 75});
    //     }else if (controller.get_digital(DIGITAL_A) && chassis.getPose().y >= 37) {
    //         chassis.turnToHeading(0, 500,{.maxSpeed = 75});
    //     }














     
     
       pros::delay(10);
   }
















 













