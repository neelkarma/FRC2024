package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;

/**
 * Naive Auto - Literally justs drives out of the starting zone to get mobility
 * points.
 */
public class NaiveAuto extends SequentialCommandGroup {
  public NaiveAuto() {
    addCommands(
        // Start driving the robot forwards
        new InstantCommand(() -> {
          // This assumes that the robot is facing towards the field center,
          // otherwise it will drive backwards.
          Subsystems.drive.drive(0.5, 0, 0, false, false);
        }, Subsystems.drive),
        // Wait 1s
        new WaitCommand(1),
        // Stop the robot
        new InstantCommand(() -> {
          Subsystems.drive.setX();
        }, Subsystems.drive));
  }
}
