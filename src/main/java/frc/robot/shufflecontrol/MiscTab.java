package frc.robot.shufflecontrol;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.PivotSub;

public class MiscTab {
  private final ShuffleboardTab misc = Shuffleboard.getTab("Misc");

  private final GenericEntry intakeLocked = misc.add("Intake Locked", false)
      .withSize(1, 1).withPosition(0, 0)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .getEntry();
  private final GenericEntry intakeIsRunning = misc.add("Intake Is Running", false)
      .withSize(1, 1).withPosition(1, 0)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .getEntry();
  private final GenericEntry shooterIsStopped = misc.add("Shooter Is Stopped", false)
      .withSize(2, 1).withPosition(0, 1)
      .withWidget(BuiltInWidgets.kBooleanBox)
      .getEntry();
  private final GenericEntry pivotState = misc.add("Pivot State", PivotSub.State.Idle.toString())
      .withSize(1, 1).withPosition(0, 2)
      .getEntry();

  protected MiscTab() {
  }

  public void setIntakeVars(boolean locked, boolean isRunning) {
    intakeLocked.setBoolean(locked);
    intakeIsRunning.setBoolean(isRunning);
  }

  public void setPivotState(PivotSub.State state) {
    pivotState.setString(state.toString());
  }

  public void setShooterIsStopped(boolean isStopped) {
    shooterIsStopped.setBoolean(isStopped);
  }

}
