package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSub extends SubsystemBase {
  private final WPI_TalonSRX masterMotor = ShooterConstants.MOTOR_1_ID.build();
  private final WPI_TalonSRX slaveMotor = ShooterConstants.MOTOR_2_ID.build();
  private final Encoder encoder = ShooterConstants.ENCODER_ID.build();
  private final PIDController controller = new PIDController(0, 0, 0);

  private boolean isStopped = false;

  public ShooterSub() {
    slaveMotor.follow(masterMotor);
    controller.setTolerance(ShooterConstants.SPEED_TOLERANCE);
  }

  @Override
  public void periodic() {
    if (isStopped)
      return;

    var rate = encoder.getRate();
    var out = controller.calculate(rate);
    masterMotor.set(MathUtil.clamp(out, -1, 1));
  }

  /**
   * Sets the target tangential speed of the shooter wheel.
   * 
   * @param targetTanSpeed The target tangential speed of the shooter wheel.
   */
  public void setSpeed(double targetTanSpeed) {
    if (isStopped) {
      controller.reset();
      isStopped = false;
    }

    // ω = v/r
    var targetRotSpeed = targetTanSpeed / ShooterConstants.WHEEL_RADIUS;

    // encoder calculates rotations, not radians
    controller.setSetpoint(targetRotSpeed * (2 * Math.PI));
  }

  /** Stops the shooter. Call `setSpeed` to start again. */
  public void stop() {
    masterMotor.stopMotor();
    isStopped = true;
  }

  /**
   * @return true if the tangential wheel speed is within the accepted bounds of
   *         the target wheel speed as defined by
   *         {@link ShooterTolerance.SPEED_TOLERANCE}
   */
  public boolean atSetSpeed() {
    return controller.atSetpoint();
  }
}
