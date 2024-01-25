package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class ClimberSub extends SubsystemBase {
  private final WPI_TalonSRX masterMotor = IntakeConstants.MOTOR_1_ID.build();

  public ClimberSub() {

    masterMotor.configFactoryDefault();
  }
  
  /**
   * Set the speed of the intake
   * 
   * @param speed Speed from -1 to 1
   */
  public void set(double speed) {
    speed = MathUtil.clamp(speed, -1, 1);
    masterMotor.set(speed);
  }

  /** Stops the intake motor */
  public void stop() {
    masterMotor.stopMotor();
  }
}