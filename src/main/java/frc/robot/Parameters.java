/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * @author Christian Piper (@CAP1Sup), Mohammed Durrani (@mdurrani808), Jadon Trackim
 *     (@JadonTrackim), Krishna Dihora (@kjdih2)
 * @since 5/8/20
 */
package frc.robot;

// Imports
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.utilityClasses.JoystickOutputTypes;
import frc.robot.utilityClasses.TuneableNumber;

/**
 * The Parameters class provides a convenient place for teams to hold robot-wide numerical or
 * boolean variables. This class should not be used for any other purpose. All parameters should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * parameters are needed, to reduce verbosity.
 */
public final class Parameters {

    // Enables all debug statements
    public static final boolean debug = false;
    public static final boolean tuningMode = false;
    public static final boolean telemetryMode = false;

    // Competition configurations
    // Flashing the controllers degrades them, so we should limit the number
    // of times that we flash. Basically, only competition scenarios
    public static final boolean flashControllers = false;

    // All of the fun parameters
    public static final class funParameters {

        public static final int SSN = 123352357; // Social Security Number [DO NOT LOSE]
        public static final double USD_TO_GHS = 5.748; // US Dollar to Gana Cedi conversion rate
        public static final double MM_TO_IK =
                2.15; // Mozambican metical to Icelandic Krona conversion rate
        public static final double MIN_IN_HR = 60; // Minutes in an hour
        public static final int BUILD_TEAM_BRAIN_CELLS = 1; // Brain cells owned by the build team
        public static final int CODING_TEAM_BRAIN_CELLS =
                15; // Same as the amount of coding team members
        public static final int SHRIMP_ON_THE_BARBIE = 3; // Number of shrimp on the barbecue
        public static final int ANDREWS_PROGRESS_WHEN_AROUND_SAFETY_TEAM =
                -10; // What happens when Andrew is around the safety team... backwards progress
        public static final int CHRISTIAN_FORTNITE_WINS =
                39; // The number of the lead programmer's Fortnite wins EASY DUBS LETS GO
        public static final int DIO_COMMENTS =
                7; // You thought this was a comment that would explain what the parameter means,
        // but it was me, DIO!
        public static final int JOJO_PARTS = 8; // parts in JOJO
        public static final int RIDICULOUS_QUESTIONS =
                1; // How many times people have asked what this parameter is for
        public static final String AMONG_US = "sus"; // sus
    }

    // All of the driver parameters
    public static final class driver {

        /**
         * A quick way of referencing driver parameters
         *
         * @param name The name of the driver
         * @param joystickParams The joystick parameters to use
         * @param fastSteerRate Maximum deg/s of rotational velocity
         * @param lockemUp If the swerve should lock the modules at 45 degrees, effectively hitting
         *     the brakes. Hard on the modules, but worth it in competition
         * @param fieldCentric If the robot should treat itself as forward or if the field's forward
         *     should be forward
         * @param maxModVelocity Maximum velocity of modules in m/s
         * @param driveIdleMode If the drive motors should coast or brake after they exceed the
         *     current set velocity. Coasting makes the driving smoother, but braking makes it more
         *     aggressive
         * @param steerIdleMode If the steering motor should coast of brake after they exceed the
         *     current set velocity. Modules will most likely only work with braking enabled
         * @param inputType The devices used to control the robot
         */
        public static final String name = "CAP1Sup";

        public static final double slowSteerRate = 180;
        public static final double fastSteerRate = 540;
        public static final boolean lockemUp = true;
        public static final boolean fieldCentric = true;
        public static final double maxModVelocity = 8;
        public static final IdleMode driveIdleMode = IdleMode.kBrake;

        // Joystick settings
        public static final class controllers {
            public static final boolean usingQuadController = false;
            public static final double deadzone = 0.075;
            public static final JoystickOutputTypes clampingType = JoystickOutputTypes.ZEROED_QUAD;
        }
    }

    // A place for general, robot wide parameters
    public static final class general {

        // Nominal voltage
        public static final double nominalVoltage = 12;
    }

    // All of the drivetrain parameters
    public static final class driveTrain {

        // Tolerances for completing movements
        public static final double angleTolerance = 2; // deg
        public static final double velocityTolerance = 0.01; // m/s

        // Main table for all swerve data
        public static final NetworkTable SWERVE_TABLE =
                NetworkTableInstance.getDefault().getTable("Swerve");

        // All of the CAN IDs
        public static final class can {

            // CAN parameters
            public static final int FL_STEER_ID = 1;
            public static final int FR_STEER_ID = 2;
            public static final int BL_STEER_ID = 3;
            public static final int BR_STEER_ID = 4;

            public static final int FL_DRIVE_ID = 5;
            public static final int FR_DRIVE_ID = 6;
            public static final int BL_DRIVE_ID = 7;
            public static final int BR_DRIVE_ID = 8;

            public static final int FL_CODER_ID = 9;
            public static final int FR_CODER_ID = 10;
            public static final int BL_CODER_ID = 11;
            public static final int BR_CODER_ID = 12;
        }

        // All of the chassis dimensions
        public static final class dimensions {

            // Swerve calculation parameters (in meters)
            public static final double DRIVE_LENGTH = Units.inchesToMeters(26.5);
            public static final double DRIVE_WIDTH = Units.inchesToMeters(20);
            public static final double MODULE_WHEEL_DIA_IN = 3.95; // Inches
            public static final double MODULE_WHEEL_DIA_M =
                    Units.inchesToMeters(MODULE_WHEEL_DIA_IN); // Meters (for odometry calculations)
        }

        // All of the maximums
        public static final class maximums {
            public static final double MAX_TRANS_VELOCITY = 4; // (m/s)
            public static final double MAX_VELOCITY = 10000; // (RPM)
            public static final double MAX_ROT_VELOCITY = 3.75;
            public static final double MAX_ACCEL = 500000000; // (RPMM)
            public static final int MAX_STEER_CURRENT = 20; // Amps
            public static final int MAX_DRIVE_CURRENT = 40; // Amps
        }

        // All of the PID parameters
        public static final class pid {
            /**
             * PID parameters Gains used in each module's steering motor, to be adjusted accordingly
             * Gains(kp, ki, kd, feedforward, iZone, peak output);
             */
            public static class steer {
                public static final TuneableNumber kP =
                        new TuneableNumber(SWERVE_TABLE, "Steer kP", 0.05);
                public static final TuneableNumber kD =
                        new TuneableNumber(SWERVE_TABLE, "Steer kD", 0.01);
                public static final double kMAX_OUTPUT = 8; // TODO: Fix later
                public static final ControlType CONTROL_TYPE = ControlType.kPosition;
            }

            public static class drive {

                // PID (I isn't needed)
                public static final TuneableNumber kP =
                        new TuneableNumber(SWERVE_TABLE, "Drive kP", .15);
                public static final TuneableNumber kD =
                        new TuneableNumber(SWERVE_TABLE, "Drive kD", 0);

                // Feedforward
                public static final double kFFS = .055; // kS
                public static final double kFFV = 3.248; // kV

                public static final double kMAX_OUTPUT = 8; // TODO: Fix later
                public static final ControlType CONTROL_TYPE = ControlType.kVelocity;
            }

            // Drivetrain PID tuning table
            public static final NetworkTable DRIVE_PID_TABLE =
                    NetworkTableInstance.getDefault().getTable("Driving PID");

            // PID controller (rotation constraints are max velocity and max acceleration)
            public static final TuneableNumber LINEAR_MOVE_P =
                    new TuneableNumber(DRIVE_PID_TABLE, "Linear kP", 1);
            public static final double LINEAR_MOVE_I = 0;
            public static final TuneableNumber LINEAR_MOVE_D =
                    new TuneableNumber(DRIVE_PID_TABLE, "Linear kD", 0);

            public static final TuneableNumber ROT_MOVE_P =
                    new TuneableNumber(DRIVE_PID_TABLE, "Rot kP", 3);
            public static final double ROT_MOVE_I = 0;
            public static final TuneableNumber ROT_MOVE_D =
                    new TuneableNumber(DRIVE_PID_TABLE, "Rot kD", 0);
            public static final double DEFAULT_ROT_TOLERANCE = 5; // TODO: What units?
        }

        // All of the movement control parameters
        public static final class movement {

            // Timeout for all movements (break if position not reached in time)
            public static final double TIMEOUT = 10; // seconds
        }

        // The gear ratios of the module
        public static final class ratios {

            // For converting CANCoder data to steer motor data
            public static final double STEER_GEAR_RATIO = 12.8;
            public static final double DRIVE_GEAR_RATIO = 8.14;
        }

        // Auton Constants
        public static final class auton {

            public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(RobotContainer.driveTrain.FL_POS, RobotContainer.driveTrain.FR_POS, RobotContainer.driveTrain.BL_POS, RobotContainer.driveTrain.BR_POS);
            public static final double TURN_180_STEER_RATE_PERCENT =
                    0.5; // The percentage of maxSteerRate (based on driver profile)
        }
    }

    public static final class climber {

        public static final int TUBE_CURRENT_LIMIT = 80;
        public static final int TUBE_HOME_CURRENT = 3;

        public static final class lift {

            // CAN IDs for lift motors
            public static final int RIGHT_MOTOR_ID = 20;
            public static final int LEFT_MOTOR_ID = 21;

            // The gearbox ratio
            public static final double GEARBOX_RATIO = 48;

            // Limit switch ports used for lift
            public static final int RIGHT_LIMIT_SWITCH_PORT = 1;
            public static final int LEFT_LIMIT_SWITCH_PORT = 2;

            // PID constants (I not used)
            public static final double kP = 20;
            public static final double kD = 0;
            public static final ControlType CONTROL_TYPE = ControlType.kPosition;

            // The tolerance for positioning the tubes (in m)
            public static final double POS_TOLERANCE = 0.005; // 5 mm of tolerance

            // Maximum motor duty
            public static final double MAX_DUTY = 1;

            // Circumference of the winch spool
            public static final double SPOOL_CIRCUMFERENCE =
                    (Math.PI
                            * Units.inchesToMeters(
                                    1)); // Diameter is 1 inch, circumference is in meters

            // Distances to move to
            public static final double UP_LEGAL_DISTANCE =
                    Units.inchesToMeters(12); // The distance of the string in the up position
            public static final double DOWN_DISTANCE =
                    Units.inchesToMeters(
                            5); // The distance at which the hook grabs the bar, but doesn't lift
            // ground
            public static final double GRAB_DISTANCE = Units.inchesToMeters(10);

            // Homing info
            public static final double HOME_SPEED = -0.25;
            public static final double HOME_DISTANCE =
                    Units.inchesToMeters(3.25); // The distance at home
        }

        public static final class tilt {

            // CAN IDs for lift motors
            public static final int RIGHT_MOTOR_ID = 18;
            public static final int LEFT_MOTOR_ID = 19;

            // The gearbox ratio
            public static final double GEARBOX_RATIO = 25;

            // Limit switch ports used for tilt
            public static final int RIGHT_LIMIT_SWITCH_PORT = 3;
            public static final int LEFT_LIMIT_SWITCH_PORT = 4;

            // PID constants (I not used)
            public static final double kP = 20;
            public static final double kD = 0;
            public static final ControlType CONTROL_TYPE = ControlType.kPosition;

            // The tolerance for positioning the tubes (in m)
            public static final double POS_TOLERANCE = 0.005; // 5 mm of tolerance

            // Maximum motor duty
            public static final double MAX_DUTY = 1;

            // Circumference of the winch spool
            public static final double SPOOL_CIRCUMFERENCE =
                    (Math.PI
                            * Units.inchesToMeters(
                                    1)); // Diameter is 1 inch, circumference is in meters

            // Distances to move to
            public static final double LEFT_LEGAL_DISTANCE = Units.inchesToMeters(16.375);
            public static final double RIGHT_LEGAL_DISTANCE = Units.inchesToMeters(15.45);
            
            public static final double DOWN_DISTANCE = Units.inchesToMeters(5); // The distance of the climber when the robot is fully off the
            // ground

            // Homing info
            public static final double HOME_SPEED = -0.25;
            public static final double HOME_DISTANCE =
                    Units.inchesToMeters(4.75); // The distance at home
        }

        // The speed of the drivetrain (in m/s) to move when tilting the robot
        public static final double DRIVE_TILT_SPEED = .25;

        // The angle to tilt the robot to before lifting the climbers
        public static final double ROBOT_TILT_ANGLE = 20;
    }

    public static final class intake {
        public static final double INTAKE_SPEED = .8;
        public static final int INTAKE_MOTOR_ID = 16;
        public static final int INTAKE_MOTOR_CURRENT_LIMIT = 40;

        public static final class spool {
            // Ports
            // TODO set these
            public static final int MOTOR_ID = 17;
            public static final int LS_PORT = 9;
            public static final int MOTOR_CURRENT_LIMIT = 40;

            // Homing info
            public static final double HOME_SPEED = 0.25;
            public static final double HOME_DISTANCE = 0.31; // The distance at home
            public static final int HOME_CURRENT = 3;

            // Basic info
            public static final double GEARBOX_RATIO =
                    12; // Ratio of motor turns to gearbox output turns
            public static double CIRCUMFRENCE =
                    (Math.PI
                            * Units.inchesToMeters(
                                    1)); // Diameter is 1 inch, circumfrence is in meters
            public static final double UP_DISTANCE =
                    0.175; // The distance of the string from the spool in the up position
            public static final double DOWN_DISTANCE =
                    0.35; // The distance of the string from the spool in the down position
            public static final double MAX_MOTOR_DUTY =
                    1; // The maximum output of the motor when moving

            public static class pid {
                public static final NetworkTable SPOOL_TABLE =
                        NetworkTableInstance.getDefault().getTable("Spool");
                public static final TuneableNumber kP = new TuneableNumber(SPOOL_TABLE, "kP", 20);
                public static final TuneableNumber kD = new TuneableNumber(SPOOL_TABLE, "kD", 0.00);
                public static final ControlType CONTROL_TYPE = ControlType.kPosition;
            }
        }
    }

    public static final class shooter {

        // Velocity conversion factor
        // Converts from RPM to m/s of linear speed on the wheels of the shooter
        // TODO: Calculate this
        public static final double WHEEL_DIA_IN = 5;
        public static final double WHEEL_DIA_M = Units.inchesToMeters(WHEEL_DIA_IN);

        // The time for a shot to take place (in s)
        public static final double SHOT_TIME = 5;

        // TODO: set this to a real port
        public static final int BOTTOM_SENSOR_PORT = 18;

        // ! TESTING ONLY
        public static final double SHOT_SPEED = 1; // In m/s
        public static final double LOAD_SPEED = 0.25; // In percent

        // Speed of shooter (in m/s of linear wheel speed)
        public static final double MAX_SPEED = 35; // verified using julia calc
        public static final double DEFAULT_SPEED = 2;
        public static final double SPIT_SPEED = 0.5;
        public static final int ID = 13;

        // Current limit
        public static final int CURRENT_LIMIT = 40;

        public static final double VELOCITY_TOLERANCE = .15;

        // Game-specific parameters (meters and degrees)
        public static final class camera {
            public static final double HEIGHT = 0;
            public static final double TARGET_HEIGHT = 0;
            public static final double PITCH = 0;

            // Camera-specific parameters (pixels)
            public static final double CAMERA_FOCAL_LENGTH = 333.82;
        }
    }

    public static final class hood {

        // TODO set these
        // Ports
        public static final int MOTOR_ID = 15;
        public static final int LS_PORT = 0;

        // Current limit
        public static final int CURRENT_LIMIT = 5;
        public static final int HOME_CURRENT = 1;

        // Homing info
        public static final double HOME_SPEED = 0.15;
        public static final double HOME_ANGLE = 110; // The angle at home

        // The default angle (if there isn't a shot interpolation available)
        public static final double DEFAULT_ANGLE = 75;

        // The angle tolerance (before shooting)
        public static final double ANGLE_TOLERANCE = 2; // deg

        // Basic info
        public static final double GEARBOX_RATIO =
                25.0; // Ratio of motor turns to gearbox output turns
        public static final double CHAIN_RATIO =
                (64.0 / 22.0); // Ratio of motor turns to hood movement
        public static final double ALLOWABLE_RANGE = 70; // The range of motion, in degrees
        public static final double MAX_MOTOR_DUTY =
                1; // The maximum output of the motor when moving

        public static class pid {
            public static final NetworkTable HOOD_TABLE =
                    NetworkTableInstance.getDefault().getTable("Hood");
            public static final TuneableNumber kP = new TuneableNumber(HOOD_TABLE, "kP", 0.25);
            public static final TuneableNumber kD = new TuneableNumber(HOOD_TABLE, "kD", 0.00);
            public static final ControlType CONTROL_TYPE = ControlType.kPosition;
        }
    }

    public static final class indexer {
        public static final int PROXIMITY_THRESHOLD = 200;
        public static final int ID = 14;
        public static final double MOTOR_SPEED = 0.15; // Duty
        public static final int CURRENT_LIMIT = 40; // A
        public static final double SHOT_TIME = 3; // s
    }

    public static final class led {
        public static final int PWM_PORT = 0;
        public static final double LAVA_RAINBOW = -.87;
        public static final double STROBE_RED = -.11;
        public static final double PARTY = -.43;
        public static final double PINK = .57;
        public static final double GLITTER_RAINBOW = -.89;
        public static final double OCEAN = -.95;
        public static final double WHITE_HB = .25;
        public static final double BLUE_VIOLET = .89;
        public static final double SKY_BLUE = .83;

        /*
            Color documentation:

            When the shooter is up to speed and ready for a ball to be shot:
            Ocean

            When a ball is detected after being intook:
            flash white

            when the robot is lined up for a shot:
            lava rainbow

            when the robot is lined up for a shot and the shooter is sped up:
            glitter rainbow
        */
    }

    // All of the relevant vision information
    public static class vision {

        // The name of the camera (from the network)
        public static final String CAMERA_NAME = "PiCam";

        // The distance to the camera from the floor (m)
        public static final double CAMERA_HEIGHT = Units.inchesToMeters(30.25);
        // The pitch of the camera from the floor (deg)
        public static final double CAMERA_PITCH = 52;

        // The height of the goal (m)
        // Converted 8ft 8in to meters
        public static final double GOAL_HEIGHT = 2.6416;

        // How far can the robot be from a target? (deg)
        public static final double YAW_TOLERANCE = 2;

        // The maximum turning speed when turning to face a target (in deg/s)
        public static final double MAX_TURNING_SPEED = 45;

        // Spin speed - used when looking for a target to lock on to (in deg/s)
        public static final double SPIN_SPEED = 0;
    }
}
