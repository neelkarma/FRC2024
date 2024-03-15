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
public class AmpAuto extends SequentialCommandGroup {
  public AmpAuto() {
    addCommands(
        // Start driving the robot forwards
        new DriveForDistanceCommand(0,-0.2,1.575),
        // Wait 0.2s while it stops
        new WaitCommand(0.2),
        //drive left
        new DriveForDistanceCommand(0.2,0,0.4),
        // Stop the robot
        new InstantCommand(() -> {
          Subsystems.drive.setX();
        }, Subsystems.drive));
  }
}
