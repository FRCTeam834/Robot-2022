// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootBall extends CommandBase {

    // Shot timing (shooter runs for specified amount of time)
    Timer timer = new Timer();
    boolean increment = false;

    /**
     * Creates a new ShootBall. public ShootBall() { // Use addRequirements() here to declare
     * subsystem dependencies. addRequirements(RobotContainer.shooter); }
     *
     * <p>// Called when the command is initially scheduled. @Override public void initialize() {
     *
     * <p>// Check to make sure that this logic is processed at the right time if
     * (!RobotContainer.shooter.getColor().equals(Robot.getOurBallColor())) {
     * RobotContainer.shooter.setDesiredSpeed(Parameters.shooter.motor.LOW_SPEED); } else {
     * RobotContainer.shooter.setDesiredSpeed(Parameters.shooter.motor.STD_SPEED); }
     *
     * <p>// Reset the timer back to zero, then start it timer.stop(); timer.reset(); timer.start();
     * }
     *
     * <p>// Called every time the scheduler runs while the command is scheduled. // If ball is the
     * wrong color, squib the shot if not shoot it properly @Override public void execute() {
     * //Changes ball count when the sensor stops detecting a ball. If the top sensor doesn't detect
     * a ball //the lower motor will spin until the ball reaches the top sensor.
     *
     * <p>if (!RobotContainer.shooter.getTopSensor()) {
     * RobotContainer.shooter.setBottomMotorSpeed(Parameters.shooter.motor.BOTTOM_SPEED);
     *
     * <p>if (increment) { increment = false; RobotContainer.shooter.addBallCount(-1); } } else {
     * RobotContainer.shooter.setBottomMotorSpeed(0); increment = true; } }
     *
     * <p>// Called once the command ends or is interrupted. @Override public void end(boolean
     * interrupted) { RobotContainer.shooter.stop(); }
     *
     * <p>// Returns true when the command should end. @Override public boolean isFinished() {
     * return timer.hasElapsed(Parameters.shooter.SHOT_TIME); }
     */
}
