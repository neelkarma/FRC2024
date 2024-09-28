package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.DriveForDistanceCommand;

/**
 * Naive Auto - Literally justs drives out of the starting zone to get mobility
 * points.
 */
public class ModilityAutoDelayed extends SequentialCommandGroup {
  public ModilityAutoDelayed() {
    addCommands(
        // Start driving the robot forwards
        new WaitCommand(10),
        new DriveForDistanceCommand(0,-0.2,8));
  }
}
