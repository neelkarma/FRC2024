package frc.robot;

import frc.robot.subsystems.LEDSub;
import frc.robot.subsystems.PivotSub;
import frc.robot.subsystems.ShooterSub;
import frc.robot.subsystems.ClimberSub;
import frc.robot.subsystems.DriveSub;
import frc.robot.subsystems.IntakeSub;

/**
 * Subsystems - Use this class to initialize and access all subsystems globally.
 */
public class Subsystems {
  public static final DriveSub drive = new DriveSub();
  public static final LEDSub led = new LEDSub();
  public static final IntakeSub intake = new IntakeSub();
  public static final ClimberSub climber = new ClimberSub();
  public static final PivotSub pivot = new PivotSub();
  public static final ShooterSub shooter = new ShooterSub();
}
