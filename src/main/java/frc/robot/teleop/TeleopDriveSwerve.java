package frc.robot.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;

public class TeleopDriveSwerve extends Command {
  public TeleopDriveSwerve() {
    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    var translateX = -OI.applyAxisDeadband(OI.pilot.getLeftY());
    var translateY = -OI.applyAxisDeadband(OI.pilot.getLeftX());
    var rotate = -OI.applyAxisDeadband(OI.pilot.getRightX());

    Subsystems.drive.drive(translateX, translateY, rotate, false, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
