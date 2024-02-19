package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PivotConstants;

public class PivotSub extends SubsystemBase {
  private final WPI_TalonSRX motor = PivotConstants.PIVOT_MOTOR_ID.build();
  private final DigitalInput highestSwitch = new DigitalInput(PivotConstants.HIGHEST_PIVOT_SWITCH_ID);
  private final DigitalInput lowestSwitch = new DigitalInput(PivotConstants.LOWEST_PIVOT_SWITCH_ID);

  @Override
  public void periodic() {
    if (highestSwitch.get() || lowestSwitch.get()) {
      motor.stopMotor();
    }
  }

  /** Start pivoting upwards. Will automatically stop at the limit. */
  public void up() {
    motor.set(1);
  }

  /** Start pivoting downwards. Will automatically stop at the limit. */
  public void down() {
    motor.set(-1);
  }

  /** Stops pivoting immediately. */
  public void stop() {
    motor.stopMotor();
  }

}
