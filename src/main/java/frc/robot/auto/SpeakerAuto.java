package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems;
import frc.robot.commands.DriveForDistanceCommand;
import frc.robot.subsystems.ShooterSub;
import pabeles.concurrency.ConcurrencyOps.NewInstance;

/**
 * Naive Auto - Literally justs drives out of the starting zone to get mobility
 * points.
 */
public class SpeakerAuto extends SequentialCommandGroup {
  public SpeakerAuto() {
    addCommands(
        new WaitCommand(2),
        new InstantCommand(() -> Subsystems.shooter.setSpeed(0.85, 0.20), Subsystems.shooter),
        new WaitCommand(2),
        new InstantCommand(() -> Subsystems.intake.set(1), Subsystems.shooter),
        new WaitCommand(1),
        new InstantCommand(() -> Subsystems.intake.set(0), Subsystems.shooter),
        new InstantCommand(() -> Subsystems.shooter.setSpeed(0, 0), Subsystems.shooter),
        
        
        new DriveForDistanceCommand(0, -0.6, 8),
        new InstantCommand(() -> Subsystems.intake.set(0), Subsystems.shooter)
        );
  }
}
