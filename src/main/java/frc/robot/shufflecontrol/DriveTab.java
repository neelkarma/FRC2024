package frc.robot.shufflecontrol;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Subsystems;
import frc.robot.teleop.TeleopProvider;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.HashMap;
import java.util.Map;

public class DriveTab {
  private ShuffleboardTab drive = Shuffleboard.getTab("Drive");

  private final GenericEntry DriveFLT = drive.add("Turn FL",0)
      .withSize(1, 1).withPosition(4, 0)
      .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -90, "max", 90))
      .getEntry();
  private final GenericEntry DriveFRT = drive.add("Turn FL",0)
      .withSize(1, 1).withPosition(9, 0)
      .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -90, "max", 90))
      .getEntry();
  private final GenericEntry DriveRLT = drive.add("Turn FL",0)
      .withSize(1, 1).withPosition(4, 4)
      .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -90, "max", 90))
      .getEntry();
  private final GenericEntry DriveRRT = drive.add("Turn FL",0)
      .withSize(1, 1).withPosition(9, 4)
      .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -90, "max", 90))
      .getEntry();

  private final GenericEntry DriveFLD = drive.add("Turn FL",0)
      .withSize(1, 1).withPosition(4, 1)
      .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -10, "max", 10))
      .getEntry();
  private final GenericEntry DriveFRD = drive.add("Turn FL",0)
      .withSize(1, 1).withPosition(9, 1)
      .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -10, "max", 10))
      .getEntry();
  private final GenericEntry DriveRLD = drive.add("Turn FL",0)
      .withSize(1, 1).withPosition(4, 3)
      .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -10, "max", 10))
      .getEntry();
  private final GenericEntry DriveRRD = drive.add("Turn FL",0)
      .withSize(1, 1).withPosition(9, 3)
      .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -10, "max", 10))
      .getEntry();

  private final GenericEntry driveX = drive.add("Turn FL",0)
      .withSize(1, 1).withPosition(5, 2)
      .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -1, "max", 1))
      .getEntry();
  private final GenericEntry driveY = drive.add("Turn FL",0)
      .withSize(1, 1).withPosition(6, 3)
      .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -1, "max", 1))
      .getEntry();
  private final GenericEntry driveR = drive.add("Turn FL",0)
      .withSize(1, 1).withPosition(5, 3)
      .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -1, "max", 1))
      .getEntry();
  

  public void setControlAxis(double contX, double contY, double contR) {
    driveX.setDouble(contX);
    driveY.setDouble(contY);
    driveR.setDouble(contR);
  }

  public void setWheelAxes(SwerveModuleState FL, SwerveModuleState FR, SwerveModuleState RL, SwerveModuleState RR){
    DriveFLT.setDouble(FL.angle.getDegrees() % 180);
    DriveFRT.setDouble(FR.angle.getDegrees() % 180);
    DriveRLT.setDouble(RL.angle.getDegrees() % 180);
    DriveRRT.setDouble(RR.angle.getDegrees() % 180);

    DriveFLD.setDouble(FL.speedMetersPerSecond);
    DriveFRD.setDouble(FR.speedMetersPerSecond);
    DriveRLD.setDouble(RL.speedMetersPerSecond);
    DriveRRD.setDouble(RR.speedMetersPerSecond);
  }

  protected DriveTab() {
    // TODO: Add swerve equivalent of the below commented code
    // drive
    // .add("DriveOutput", Subsystems.diffDrive.drive)
    // .withSize(4, 2).withPosition(0, 0)
    // .withWidget(BuiltInWidgets.kDifferentialDrive);

    drive
        .add("TeleopType", TeleopProvider.getInstance().chooser)
        .withSize(2, 1).withPosition(2, 4)
        .withWidget(BuiltInWidgets.kComboBoxChooser);
  }
}
