package frc.robot.utilityClasses.timedCommandScheduling;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandAtTime {

    // Variables to store the command and time
    private Command command;
    private double time;

    /**
     * A command that is scheduled to run at a specific time
     *
     * @param command The command to run
     * @param runAt The time (ms, according to FPGA) to run the command at
     */
    public CommandAtTime(Command command, double runAt) {
        this.command = command;
        time = runAt;
    }

    /**
     * Gets the time that the command should run at
     *
     * @return time to run at (ms)
     */
    public double getTime() {
        return time;
    }

    /**
     * Gets the command to run
     *
     * @return the command to run
     */
    public Command getCommand() {
        return command;
    }
}
