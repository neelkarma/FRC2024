package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.EncoderBuilder;
import frc.robot.utils.motorbuilder.MotorBuilder;
import frc.robot.utils.motorbuilder.TalonMotorBuilder;

public class ShooterConstants {
  public static final MotorBuilder<WPI_TalonSRX> MOTOR_1_ID = new TalonMotorBuilder(2);
  public static final MotorBuilder<WPI_TalonSRX> MOTOR_2_ID = new TalonMotorBuilder(3);

  public static final EncoderBuilder ENCODER_ID = new EncoderBuilder(new int[] { 1, 1 }, 0);

  /** Shooter wheel radius in meters */
  public static final double WHEEL_RADIUS = 0.2;
}
