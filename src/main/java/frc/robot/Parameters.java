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
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
         * @param maxSteerRate Maximum deg/s of rotational velocity
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

        public static final double maxSteerRate = 180;
        public static final boolean lockemUp = true;
        public static final boolean fieldCentric = true;
        public static final double maxModVelocity = 8;
        public static final IdleMode driveIdleMode = IdleMode.kBrake;

        // Joystick settings
        public static final class joysticks {
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
            public static final double DRIVE_LENGTH = Units.inchesToMeters(22.4);
            public static final double DRIVE_WIDTH = Units.inchesToMeters(22.4);
            public static final double MODULE_WHEEL_DIA_IN = 4; // Inches
            public static final double MODULE_WHEEL_DIA_M =
                    Units.inchesToMeters(MODULE_WHEEL_DIA_IN); // Meters (for odometry calculations)
        }

        // All of the maximums
        public static final class maximums {
            public static final double MAX_MODULE_VELOCITY = 8; // (m/s)
            public static final double MAX_VELOCITY = 10000; // (RPM)
            public static final double MAX_ACCEL = 500000000; // (RPMM)
            public static final int MAX_STEER_CURRENT = 20; // Amps
            public static final int MAX_DRIVE_CURRENT = 50; // Amps
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
                public static final TuneableNumber kP =
                        new TuneableNumber(SWERVE_TABLE, "Drive kP", .5);
                public static final TuneableNumber kD =
                        new TuneableNumber(SWERVE_TABLE, "Drive kD", 0);
                public static final double kMAX_OUTPUT = 8; // TODO: Fix later
                public static final ControlType CONTROL_TYPE = ControlType.kVelocity;
            }

            // PID controller (rotation constraints are max velocity and max acceleration)
            public static final double DEFAULT_LINEAR_MOVE_P = 1;
            public static final double DEFAULT_LINEAR_MOVE_I = 0;
            public static final double DEFAULT_LINEAR_MOVE_D = 0;

            public static final double DEFAULT_ROT_MOVE_P = 1;
            public static final double DEFAULT_ROT_MOVE_I = 0;
            public static final double DEFAULT_ROT_MOVE_D = 0;
            public static final double DEFAULT_ROT_MAX_VELOCITY = 360; // deg/s
            public static final double DEFAULT_ROT_MAX_ACCEL = 180; // deg/s
            public static final double DEFAULT_ROT_TOLERANCE = 5; // TODO: What units?
        }

        // All of the movement control parameters
        public static final class movement {

            // Timeout for all movements (break if position not reached in time)
            public static final double TIMEOUT = 10; // seconds

            // Pose estimator parameters (units are m, m, radians)
            public static final Matrix<N3, N1> POSE_STD_DEV =
                    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, Math.toRadians(0.125));
            public static final Matrix<N1, N1> ENCODER_GYRO_DEV =
                    new MatBuilder<>(Nat.N1(), Nat.N1()).fill(Math.toRadians(0.125));
            public static final Matrix<N3, N1> VISION_DEVIATION =
                    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, Math.toRadians(0.125));
        }

        // The gear ratios of the module
        public static final class ratios {

            // For converting CANCoder data to steer motor data
            public static final double STEER_GEAR_RATIO = 12.8;
            public static final double DRIVE_GEAR_RATIO = 8.14;
        }

        // Auton Constants
        public static final class auton {

            // Must be updated for new games!
            public static final double DRIVE_SPEED = -2; // m/s
            public static final double TIME_OFF_LINE = 2.25; // s

            public static final double LINE_UP_SPEED = 1.5; // m/s
            public static final double LINEUP_TIME = .5; // s

            public static final double TURN_180_STEER_RATE_PERCENT =
                    0.5; // The percentage of maxSteerRate (based on driver profile)
        }
    }

    // All of the starting position data
    public static final class positions {

        // All of the possible starting positions (and their angles)
        // In format (default (no station assigned), 1st station, 2nd station, 3rd station)
        public static final Pose2d[] POSSIBLE_STARTING_POSITIONS = {
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0))
        };

        // Actual starting position (declared in the global scope)
        public static Pose2d STARTING_POS = new Pose2d(0, 0, new Rotation2d());
        // Parameters.positions.POSSIBLE_STARTING_POSITIONS[DriverStation.getLocation()];
    }

    // All of the joystick variables
    public static final class joysticks {

        // Dynamically allocated Joysticks
        // Joysticks have 11 buttons
        public static final int JOYSTICK_BUTTON_COUNT = 11;
    }

    public static final class climber {

        // Position conversion factor (from encoder counts to meters)
        // TODO: Find this factor
        public static final double POS_CONV_FACTOR = 1;

        // TODO: Find distance of a full climber move
        public static final double MOVE_DISTANCE = 0;

        // The allowable tolerance between the two climbing hooks while climbing (in m)
        public static final double ALLOWABLE_DEVIATION = 0.05;

        // The default speed for running the climber
        public static final double DEFAULT_SPEED = 0.75;

        // The adjustment to the speed of the climbers if they aren't equal
        // This is added to the slower motor and subtracted from the faster one
        public static final double SPEED_REDUCTION = 0.05;

        public static final class right {
            public static final class motor {
                public static final int ID = 14;
            }

            public static final class limitSwitch {
                public static final int DIO_CHAN = 0;
            }
        }

        public static final class left {
            public static final class motor {
                public static final int ID = 15;
            }

            public static final class limitSwitch {
                public static final int DIO_CHAN = 1;
            }
        }
    }

    public static final class intake {
        public static final class motor {
            public static final double SPEED = .5;
            public static final int ID = 16;
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

        public static final class motor {
            // Speed of shooter (in m/s of linear wheel speed)
            public static final double STD_SPEED = 2;
            public static final double SPIT_SPEED = 0.5;
            public static final int ID = 17;
        }

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

        // Homing info
        public static final double HOME_SPEED = 0.05;
        public static final double HOME_ANGLE = 0; // The angle at home

        // Basic info
        public static final double GEARBOX_RATIO =
                100; // Ratio of motor turns to gearbox output turns
        public static final double CHAIN_RATIO =
                (64.0 / 22.0); // Ratio of motor turns to hood movement
        public static final double ALLOWABLE_RANGE = 30; // The range of motion, in degrees

        // Temporary movement info
        public static final double MOVE_SPEED = 0.05;

        public static class pid {
            public static final NetworkTable HOOD_TABLE =
                    NetworkTableInstance.getDefault().getTable("Hood");
            public static final TuneableNumber kP = new TuneableNumber(HOOD_TABLE, "kP", 0.05);
            public static final TuneableNumber kD = new TuneableNumber(HOOD_TABLE, "kD", 0.01);
            public static final ControlType CONTROL_TYPE = ControlType.kPosition;
        }
    }

    public static final class indexer {
        public static final class colorSensor {
            public static final int PROXIMITY_THRESHOLD = 200;
        }

        public static final class motor {
            public static final int ID = 18;
        }
    }

    public static final class led{
        public static final int PORT = 9;
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
}
