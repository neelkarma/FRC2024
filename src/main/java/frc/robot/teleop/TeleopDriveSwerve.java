package frc.robot.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;
import frc.robot.shufflecontrol.ShuffleControl;

public class TeleopDriveSwerve extends Command {
  private int updateShuffleCounter = 0;
  public TeleopDriveSwerve() {
    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    if (updateShuffleCounter > DriveConstants.updateShuffleInterval) {
      ShuffleControl.driveTab.setControlAxis(OI.pilot.getLeftX(), OI.pilot.getLeftY(), OI.pilot.getRightX());
      updateShuffleCounter = 0;
    } else {
      updateShuffleCounter++;
    }

    var translateX  = -OI.applyAxisDeadband(OI.pilot.getLeftX());
    var translateY  =  OI.applyAxisDeadband(OI.pilot.getLeftY());
    var rotate      =  OI.applyAxisDeadband(OI.pilot.getRightX());

    Subsystems.drive.drive(translateX, translateY, rotate, true, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
