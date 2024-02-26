package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.motorsupplier.MotorSupplier;
import frc.robot.utils.motorsupplier.TalonMotorSupplier;

public class IntakeConstants {
  // Just a random number
  public static final MotorSupplier<WPI_TalonSRX> MOTOR_1_ID = new TalonMotorSupplier(31);
  public static final MotorSupplier<WPI_TalonSRX> MOTOR_2_ID = new TalonMotorSupplier(30);
  public static final int BEAM_BREAK_SENSOR_ID = 15;
}
