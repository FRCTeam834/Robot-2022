package frc.robot.commands.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;

@SuppressWarnings("MemberName")
public class FollowPath extends CommandBase {
    private final Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private final HolonomicDriveController m_controller;

    /**
     * Constructs a new PPSwerveControllerCommand that when executed will follow the provided
     * trajectory. This command will not return output voltages but rather raw module states from
     * the position controllers which need to be put into a velocity PID.
     *
     * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
     * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * @param trajectory The trajectory to follow.
     */
    @SuppressWarnings("ParameterName")
    public FollowPath(PathPlannerTrajectory trajectory) {
        m_trajectory = trajectory;
        PIDController xPID = new PIDController(Parameters.driveTrain.pid.LINEAR_MOVE_P.get(), 0, 0);
        PIDController yPID = new PIDController(Parameters.driveTrain.pid.LINEAR_MOVE_P.get(), 0, 0);
        ProfiledPIDController rotPID =
                new ProfiledPIDController(
                        Parameters.driveTrain.pid.ROT_MOVE_P.get(),
                        0,
                        0,
                        new Constraints(Math.PI, Math.PI * Math.PI));

        m_controller = new HolonomicDriveController(xPID, yPID, rotPID);
        rotPID.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(RobotContainer.driveTrain);
    }

    @Override
    public void initialize() {
        m_timer.stop();
        m_timer.reset();
        m_timer.start();
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {

        // Get the current time for the path
        double curTime = m_timer.get();

        // Look up where we need to be
        PathPlannerState desiredState = (PathPlannerState) m_trajectory.sample(curTime);

        var targetChassisSpeeds =
        m_controller.calculate(
                RobotContainer.driveTrain.getEstPose2D(), desiredState, desiredState.holonomicRotation);

        System.out.println(desiredState.velocityMetersPerSecond);
        RobotContainer.driveTrain.setModuleStates(targetChassisSpeeds);
        
    }

    public Pose2d getStartingPose(PathPlannerTrajectory trajectory) {

        // Calculate the first pose of the trajectory
        return new Pose2d(trajectory.getInitialState().poseMeters.getTranslation(), trajectory.getInitialState().holonomicRotation);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}
