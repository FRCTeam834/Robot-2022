package frc.robot.utilityClasses.timedCommandScheduling;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ArrayList;

public class TimedCommandSchedule {
    private ArrayList<CommandAtTime> commands;

    public TimedCommandSchedule() {
        commands = new ArrayList<CommandAtTime>();
    }

    public void add(CommandAtTime commandAtTime) {

        // Get the size of the commands list
        int size = commands.size();

        // If the arraylist is empty, just insert it
        if (size == 0) {
            commands.add(commandAtTime);
        }
        // ! WRITE YET
        else {
            // We gotta do a whole binary search for the right location
            commands.add(binarySearch(commandAtTime.getTime()), commandAtTime);
        }
    }

    private int binarySearch(double newTime) {
        int low = 0;
        int high = commands.size() - 1;

        while (low <= high) {

            // Finding the mid using floor division
            int mid = low + ((high - low) / 2);

            // Get the time of the mid command
            double midTime = commands.get(mid).getTime();

            // If the time of the middle command is the same, schedule it directly after the middle
            // command
            if (midTime == newTime) {
                return mid + 1;
            }

            // Check if the mid's time is less than the new command
            else if (midTime < newTime) {
                low = mid;
            }

            // Check if the mid's time is greater than the new command
            else if (midTime > newTime) {
                high = mid;
            }
        }

        return low;
    }

    public void add(Command command, double atTime) {
        add(new CommandAtTime(command, atTime));
    }

    public void add(Runnable runnable, double atTime) {
        add(new InstantCommand(runnable), atTime);
    }

    public void add(Runnable runnable, double atTime, Subsystem... requirements) {
        add(new InstantCommand(runnable, requirements), atTime);
    }

    public void update(double currentTime) {
        while (commands.get(0).getTime() <= currentTime) {
            CommandScheduler.getInstance().schedule(commands.remove(0).getCommand());
        }
    }
}
