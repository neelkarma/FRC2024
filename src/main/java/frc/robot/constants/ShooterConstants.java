package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.utils.EncoderSupplier;
import frc.robot.utils.motorsupplier.MotorSupplier;
import frc.robot.utils.motorsupplier.VictorMotorSupplier;

public class ShooterConstants {
  public static final MotorSupplier<WPI_VictorSPX> MOTOR_1_ID = new VictorMotorSupplier(13);
  public static final MotorSupplier<WPI_VictorSPX> MOTOR_2_ID = new VictorMotorSupplier(14);

  public static final EncoderSupplier ENCODER_ID = new EncoderSupplier(new int[] { 9, 10 }, 1);

  /** Shooter wheel radius in meters */
  public static final double WHEEL_RADIUS = 0.2;

  /**
   * The robot will consider the shooter at the target tangential wheel speed
   * within this tolerance
   */
  public static final double SPEED_TOLERANCE = 0.5;
}
