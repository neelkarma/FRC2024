package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.motorsupplier.MotorSupplier;
import frc.robot.utils.motorsupplier.TalonMotorSupplier;

public class ShooterConstants {
  public static final MotorSupplier<WPI_TalonSRX> UPPER_MOTOR_ID = new TalonMotorSupplier(7).withInvert();
  public static final MotorSupplier<WPI_TalonSRX> LOWER_MOTOR_ID = new TalonMotorSupplier(8).withInvert();

  public static final double SPIN_DIFF = -0.13;
}
