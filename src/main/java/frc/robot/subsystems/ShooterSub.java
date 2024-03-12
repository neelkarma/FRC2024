package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSub extends SubsystemBase {
  private final WPI_TalonSRX upperMotor = ShooterConstants.UPPER_MOTOR_ID.get();
  private final WPI_TalonSRX lowerMotor = ShooterConstants.LOWER_MOTOR_ID.get();

  public ShooterSub() {
    addChild("Master Motor", upperMotor);
    addChild("Slave Motor", lowerMotor);
  }

  /**
   * Sets the speed of the shooter wheel.
   * 
   * @param speed Speed, from -1 to 1.
   */
  public void setSpeed(double speed) {
    double speedUp = MathUtil.clamp(speed * (1 + ShooterConstants.SPIN_DIFF), -1, 1);
    double speedDown = MathUtil.clamp(speed * (1 - ShooterConstants.SPIN_DIFF), -1, 1);
    upperMotor.set(speedUp);
    lowerMotor.set(speedDown);
  }

  /** Stops the shooter. Call `setSpeed` to start again. */
  public void stop() {
    upperMotor.stopMotor();
    lowerMotor.stopMotor();
  }
}
