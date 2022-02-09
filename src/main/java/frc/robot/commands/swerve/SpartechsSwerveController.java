package frc.robot.commands.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Parameters;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Vision;

@SuppressWarnings("MemberName")
public class SpartechsSwerveController extends CommandBase {
    private final Timer m_timer = new Timer();
    private final PathPlannerTrajectory m_trajectory;
    private final HolonomicDriveController m_controller;
    private Rotation2d desiredRotation2d;
    private final boolean targetLock;

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
    public SpartechsSwerveController(PathPlannerTrajectory trajectory, boolean targetLock) {
        m_trajectory = trajectory;
        this.targetLock = targetLock;
        PIDController xPID =
                new PIDController(Parameters.driveTrain.pid.DEFAULT_LINEAR_MOVE_P, 0, 0);
        PIDController yPID =
                new PIDController(Parameters.driveTrain.pid.DEFAULT_LINEAR_MOVE_P, 0, 0);
        ProfiledPIDController rotPID =
                new ProfiledPIDController(
                        Parameters.driveTrain.pid.DEFAULT_ROT_MOVE_P, 0, 0, new Constraints(8, 5));

        m_controller = new HolonomicDriveController(xPID, yPID, rotPID);
        rotPID.enableContinuousInput(-180, 180);
        addRequirements(RobotContainer.driveTrain);
    }

    @Override
    public void initialize() {

        m_timer.reset();
        m_timer.start();
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = (PathPlannerState) m_trajectory.sample(curTime);
        System.out.println(desiredState.positionMeters);
        if (!targetLock) {
            desiredRotation2d = desiredState.holonomicRotation;
        } else {
            desiredRotation2d =
                    RobotContainer.driveTrain
                            .getEstPose2D()
                            .getRotation()
                            .rotateBy(Rotation2d.fromDegrees(Vision.getYaw()));
        }
        var targetChassisSpeeds =
                m_controller.calculate(
                        RobotContainer.driveTrain.getEstPose2D(), desiredState, new Rotation2d());

        RobotContainer.driveTrain.setModuleStates(targetChassisSpeeds);
    }

    public State getStartingState(PathPlannerTrajectory trajectory) {
        return trajectory.sample(0);
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
