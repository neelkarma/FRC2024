package frc.robot.teleop;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.Variables;
import frc.robot.constants.DriveConstants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.RangeMath.CurveFit;
import frc.robot.utils.RangeMath.RangeSettings;

public class TeleopDriveSwerve extends Command {
  private int updateShuffleCounter = 0;
  public RangeSettings settings = DriveConstants.PILOT_SETTINGS;

  public TeleopDriveSwerve() {
    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    double limiter = 1;//OI.pilot.getLeftTriggerAxis(); //TODO enable this when everything else is tested
    // organise field relitive switch
    double[] control = CurveFit.fitDrive(new double[]{OI.pilot.getLeftX(), OI.pilot.getLeftY(), 
                                         OI.pilot.getRightX(), limiter}, settings);
    var translateX  = control[0];
    var translateY  = control[1];
    var rotate      = control[2];

    if (updateShuffleCounter > DriveConstants.updateShuffleInterval) {
      ShuffleControl.driveTab.setControlAxis(translateX, translateY, rotate);
      updateShuffleCounter = 0;
    } else {
      updateShuffleCounter++;
    }

    Subsystems.drive.drive(translateX, translateY, rotate, Variables.fieldRelative, false); // TODO fix rate limit
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
