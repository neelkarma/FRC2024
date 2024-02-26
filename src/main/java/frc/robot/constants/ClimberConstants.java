package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.motorsupplier.MotorSupplier;
import frc.robot.utils.motorsupplier.TalonMotorSupplier;

public class ClimberConstants {
  // port needs to be set correctly
  public static final MotorSupplier<WPI_TalonSRX> MOTOR_1_ID = new TalonMotorSupplier(32);

}
