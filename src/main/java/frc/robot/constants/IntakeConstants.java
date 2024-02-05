package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.motorbuilder.MotorBuilder;
import frc.robot.utils.motorbuilder.TalonMotorBuilder;

public class IntakeConstants {
  // Just a random number
  public static final MotorBuilder<WPI_TalonSRX> MOTOR_1_ID = new TalonMotorBuilder(31);
  public static final MotorBuilder<WPI_TalonSRX> MOTOR_2_ID = new TalonMotorBuilder(30);
  public static final int BEAM_BREAK_SENSOR_ID = 15;
}
