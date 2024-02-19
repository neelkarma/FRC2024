package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.motorbuilder.MotorBuilder;
import frc.robot.utils.motorbuilder.TalonMotorBuilder;

public class PivotConstants {
  public static final MotorBuilder<WPI_TalonSRX> PIVOT_MOTOR_ID = new TalonMotorBuilder(1).withSafety().withBrake();
  public static final int HIGHEST_PIVOT_SWITCH_ID = 1;
  public static final int LOWEST_PIVOT_SWITCH_ID = 1;
}
