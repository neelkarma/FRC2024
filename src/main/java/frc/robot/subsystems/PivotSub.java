package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PivotConstants;

public class PivotSub extends SubsystemBase {
  private final WPI_TalonSRX motor = PivotConstants.PIVOT_MOTOR_ID.build();
  private final DigitalInput highestSwitch = new DigitalInput(PivotConstants.HIGHEST_PIVOT_SWITCH_ID);
  private final DigitalInput lowestSwitch = new DigitalInput(PivotConstants.LOWEST_PIVOT_SWITCH_ID);

  private State state;

  public static enum State {
    Idle,
    MovingUp,
    MovingDown,
    FullyUp,
    FullyDown
  }

  @Override
  public void periodic() {
    if (highestSwitch.get()) {
      state = State.FullyUp;
      motor.stopMotor();
    }

    if (lowestSwitch.get()) {
      state = State.FullyDown;
      motor.stopMotor();
    }
  }

  /** Start pivoting upwards. Will automatically stop at the limit. */
  public void up() {
    if (state == State.FullyUp)
      return;
    state = State.MovingUp;
    motor.set(1);
  }

  /** Start pivoting downwards. Will automatically stop at the limit. */
  public void down() {
    if (state == State.FullyDown)
      return;
    state = State.MovingDown;
    motor.set(-1);
  }

  /** Stops pivoting immediately. */
  public void stop() {
    state = State.Idle;
    motor.stopMotor();
  }

  public State getState() {
    return state;
  }

}
