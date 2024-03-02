package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.utils.motorsupplier.MotorSupplier;
import frc.robot.utils.motorsupplier.VictorMotorSupplier;

public class IntakeConstants {
  // Just a random number
  public static final MotorSupplier<WPI_VictorSPX> UPPER_MOTOR_ID = new VictorMotorSupplier(9);
  public static final MotorSupplier<WPI_VictorSPX> LOWER_MOTOR_ID = new VictorMotorSupplier(10).withInvert();
  public static final int BEAM_BREAK_SENSOR_ID = 15;
}
