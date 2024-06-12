package frc.robot.teleop;

import org.ejml.equation.Variable;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.OI;
import frc.robot.Subsystems;
import frc.robot.Variables;
import frc.robot.constants.DriveConstants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.RangeMath.CurveFit;

public class TeleopDriveSwerve extends Command {
  private int updateShuffleCounter = 0;

  public TeleopDriveSwerve() {
    addRequirements(Subsystems.drive);
  }

  @Override
  public void execute() {
    //System.out.println(Math.atan2(OI.applyAxisDeadband(OI.pilot.getLeftY()),OI.applyAxisDeadband(OI.pilot.getLeftX())));
    double[] control = CurveFit.fitDrive(new double[]{OI.pilot.getLeftX(), OI.pilot.getLeftY(), 
                                         OI.pilot.getRightX()}, DriveConstants.PILOT_SETTINGS);
    var translateX  = control[0];
    var translateY  = control[1];
    var rotate      = control[2];

    if (updateShuffleCounter > DriveConstants.updateShuffleInterval) {
      ShuffleControl.driveTab.setControlAxis(translateX, translateY, rotate);
      updateShuffleCounter = 0;
    } else {
      updateShuffleCounter++;
    }

    Subsystems.drive.drive(translateX, translateY, rotate, true, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
