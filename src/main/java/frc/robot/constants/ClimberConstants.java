package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.utils.motorbuilder.MotorBuilder;
import frc.robot.utils.motorbuilder.TalonMotorBuilder;

public class ClimberConstants {
  // port needs to be set correctly
  public static final MotorBuilder<WPI_TalonSRX> MOTOR_1_ID = new TalonMotorBuilder(32);
  
}
