package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.utils.EncoderSupplier;
import frc.robot.utils.motorsupplier.MotorSupplier;
import frc.robot.utils.motorsupplier.TalonMotorSupplier;
import frc.robot.utils.motorsupplier.VictorMotorSupplier;

public class ShooterConstants {
  public static final MotorSupplier<WPI_TalonSRX> UPPER_MOTOR_ID = new TalonMotorSupplier(7).withInvert();
  public static final MotorSupplier<WPI_TalonSRX> LOWER_MOTOR_ID = new TalonMotorSupplier(8).withInvert();

  public static final EncoderSupplier ENCODER_ID = new EncoderSupplier(new int[] { 9, 10 }, 1);

  /** Shooter wheel radius in meters */
  public static final double WHEEL_RADIUS = 0.2;
  

  public static final double SPIN_DIFF = -0.13;

  /**
   * The robot will consider the shooter at the target tangential wheel speed
   * within this tolerance
   */
  public static final double SPEED_TOLERANCE = 0.5;
}
