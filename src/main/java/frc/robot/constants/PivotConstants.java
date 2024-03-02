package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.utils.motorsupplier.MotorSupplier;
import frc.robot.utils.motorsupplier.VictorMotorSupplier;

public class PivotConstants {
  public static final MotorSupplier<WPI_VictorSPX> PIVOT_MOTOR_ID = new VictorMotorSupplier(/*12*/ 36).withSafety().withBrake();

  /**
   * Port for the limit switch that will trigger when the pivot reaches max angle
   * of elevation
   */
  public static final int HIGHEST_PIVOT_SWITCH_ID = 11;

  /**
   * Port for the limit switch that will trigger when the pivot reaches min angle
   * of elevation
   */
  public static final int LOWEST_PIVOT_SWITCH_ID = 12;
}
