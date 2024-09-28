package frc.robot.shufflecontrol;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.Map;

public class DriveTab {
  private final ShuffleboardTab drive = Shuffleboard.getTab("Drive");

  private final GenericEntry DriveFLT = drive.add("Turn FL", 0)
      .withSize(1, 1).withPosition(4, 0)
      .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -90, "max", 90))
      .getEntry();
  private final GenericEntry DriveFRT = drive.add("Turn FR", 0)
      .withSize(1, 1).withPosition(9, 0)
      .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -90, "max", 90))
      .getEntry();
  private final GenericEntry DriveRLT = drive.add("Turn RL", 0)
      .withSize(1, 1).withPosition(4, 3)
      .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -90, "max", 90))
      .getEntry();
  private final GenericEntry DriveRRT = drive.add("Turn RR", 0)
      .withSize(1, 1).withPosition(9, 3)
      .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -90, "max", 90))
      .getEntry();

  private final GenericEntry DriveFLD = drive.add("Drive FL", 0)
      .withSize(1, 1).withPosition(4, 1)
      .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -10, "max", 10))
      .getEntry();
  private final GenericEntry DriveFRD = drive.add("Drive FR", 0)
      .withSize(1, 1).withPosition(9, 1)
      .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -10, "max", 10))
      .getEntry();
  private final GenericEntry DriveRLD = drive.add("Drive RL", 0)
      .withSize(1, 1).withPosition(4, 2)
      .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -10, "max", 10))
      .getEntry();
  private final GenericEntry DriveRRD = drive.add("Drive RR", 0)
      .withSize(1, 1).withPosition(9, 2)
      .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -10, "max", 10))
      .getEntry();

  private final GenericEntry driveX = drive.add("Drive X", 0)
      .withSize(1, 1).withPosition(5, 2)
      .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -1, "max", 1))
      .getEntry();
  private final GenericEntry driveY = drive.add("Drive Y", 0)
      .withSize(1, 1).withPosition(6, 3)
      .withWidget(BuiltInWidgets.kNumberBar).withProperties(Map.of("min", -1, "max", 1))
      .getEntry();
  private final GenericEntry driveR = drive.add("Drive Rot", 0)
      .withSize(1, 1).withPosition(5, 3)
      .withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", -1, "max", 1))
      .getEntry();

  private GenericEntry ampS = drive.add("amp spd", Subsystems.shooter.ampSpd).getEntry();
  private GenericEntry ampD = drive.add("amp Dif", Subsystems.shooter.ampDiff).getEntry();
  private GenericEntry SpeS = drive.add("Speaker spd", Subsystems.shooter.speakerSpd).getEntry();
  private GenericEntry SpeD = drive.add("Speaker Dif", Subsystems.shooter.speakerDiff).getEntry();

  public void updateShooterSpd(){
    Subsystems.shooter.ampSpd = ampS.getDouble(Subsystems.shooter.ampSpd);
    Subsystems.shooter.ampDiff = ampD.getDouble(Subsystems.shooter.ampDiff);
    Subsystems.shooter.speakerSpd = SpeS.getDouble(Subsystems.shooter.speakerSpd);
    Subsystems.shooter.speakerDiff = SpeD.getDouble(Subsystems.shooter.speakerDiff);
  }

  public void setControlAxis(double contX, double contY, double contR) {
    driveX.setDouble(contX);
    driveY.setDouble(contY);
    driveR.setDouble(contR);
  }

  public void setWheelAxes(SwerveModuleState FL, SwerveModuleState FR, SwerveModuleState RL, SwerveModuleState RR) {
    DriveFLT.setDouble(FL.angle.getDegrees() % 180 -90);
    DriveFRT.setDouble(FR.angle.getDegrees() % 180 -90);
    DriveRLT.setDouble(RL.angle.getDegrees() % 180 -90);
    DriveRRT.setDouble(RR.angle.getDegrees() % 180 -90);

    DriveFLD.setDouble(FL.speedMetersPerSecond);
    DriveFRD.setDouble(FR.speedMetersPerSecond);
    DriveRLD.setDouble(RL.speedMetersPerSecond);
    DriveRRD.setDouble(RR.speedMetersPerSecond);
  }

  protected DriveTab() {
  }
}
