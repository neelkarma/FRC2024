package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PivotConstants;
import frc.robot.shufflecontrol.ShuffleControl;

public class PivotSub extends SubsystemBase {
  private final WPI_VictorSPX motor = PivotConstants.PIVOT_MOTOR_ID.get();
  private final DigitalInput highestSwitch = new DigitalInput(PivotConstants.HIGHEST_PIVOT_SWITCH_ID);
  private final DigitalInput lowestSwitch = new DigitalInput(PivotConstants.LOWEST_PIVOT_SWITCH_ID);

  private State state = State.Idle;

  public static enum State {
    Idle,
    MovingUp,
    MovingDown,
    FullyUp,
    FullyDown
  }

  public PivotSub() {
    addChild("Motor", motor);
    addChild("Highest Limit Switch", highestSwitch);
    addChild("Lowest Limit Switch", lowestSwitch);
  }

  @Override
  public void periodic() {
    ShuffleControl.miscTab.setPivotState(state);

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
    //if (state == State.FullyUp)
    //  return;
    state = State.MovingUp;
    motor.set(0.3);
  }

  /** Start pivoting downwards. Will automatically stop at the limit. */
  public void down() {
    //if (state == State.FullyDown)
    //  return;
    state = State.MovingDown;
    motor.set(-0.3);
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
