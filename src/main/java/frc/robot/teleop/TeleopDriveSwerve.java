package frc.robot.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.shufflecontrol.ShuffleControl;

public class TeleopDriveSwerve extends Command {
  public TeleopDriveSwerve() {
    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    var translateX  = -OI.applyAxisDeadband(OI.pilot.getLeftX());
    var translateY  =  OI.applyAxisDeadband(OI.pilot.getLeftY());
    var rotate      =  OI.applyAxisDeadband(OI.pilot.getRightX());

    Subsystems.drive.drive(translateX, translateY, rotate, true, true);
    ShuffleControl.driveTab.setControlAxis(translateX, translateY, rotate);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
