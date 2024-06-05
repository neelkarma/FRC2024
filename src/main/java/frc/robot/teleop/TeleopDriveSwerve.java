package frc.robot.teleop;

import org.ejml.equation.Variable;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.Variables;
import frc.robot.constants.DriveConstants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.CurveFit;

public class TeleopDriveSwerve extends Command {
  private final CurveFit throtFit;
  private final CurveFit steerFit;
  private int updateShuffleCounter = 0;

  public TeleopDriveSwerve() {
    addRequirements(Subsystems.drive);
    throtFit = CurveFit.throtFromDriveSettings(DriveConstants.PILOT_SETTINGS);
    steerFit = CurveFit.steerFromDriveSettings(DriveConstants.PILOT_SETTINGS);
  }

  @Override
  public void execute() {
    System.out.println(Math.atan2(OI.applyAxisDeadband(OI.pilot.getLeftY()),OI.applyAxisDeadband(OI.pilot.getLeftX())));
    var translateX  = throtFit.fit(-OI.applyAxisDeadband(OI.pilot.getLeftX()))*0.2;
    var translateY  = throtFit.fit( OI.applyAxisDeadband(OI.pilot.getLeftY()))*0.2;
    var rotate      = steerFit.fit( OI.applyAxisDeadband(OI.pilot.getRightX()));

    if (updateShuffleCounter > DriveConstants.updateShuffleInterval) {
      ShuffleControl.driveTab.setControlAxis(translateX, translateY, rotate);
      updateShuffleCounter = 0;
    } else {
      updateShuffleCounter++;
    }

    Subsystems.drive.drive(translateX, translateY, rotate, true, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
