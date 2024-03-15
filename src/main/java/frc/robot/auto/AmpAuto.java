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
        new DriveForDistanceCommand(0,-0.4,1.575),
        // Wait 0.2s while it stops
        new WaitCommand(0.2),
        //drive left
        new DriveForDistanceCommand(0.4,0,0.5),
        // Stop the robot
        //run the intake
        new InstantCommand(() -> {Subsystems.shooter.setSpeed(0.65,0.18);}, Subsystems.shooter),
        new WaitCommand(1),
        new InstantCommand(() -> {Subsystems.shooter.setSpeed(0,0);}, Subsystems.shooter)
        );
  }
}
