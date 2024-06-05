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
    var translateX  = throtFit.fit(-OI.applyAxisDeadband(OI.pilot.getLeftX()))*0.15;
    var translateY  = throtFit.fit( OI.applyAxisDeadband(OI.pilot.getLeftY()))*0.15;
    var rotate      = steerFit.fit( OI.applyAxisDeadband(OI.pilot.getRightX()))*0.3;

    translateX = translateX * (Variables.driveSlow ? 0.4 : 1);
    translateY = translateY * (Variables.driveSlow ? 0.4 : 1);
    rotate     = rotate     * (Variables.driveSlow ? 0.4 : 1);


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
