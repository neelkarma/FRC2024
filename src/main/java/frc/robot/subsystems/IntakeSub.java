package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;
import frc.robot.shufflecontrol.ShuffleControl;

public class IntakeSub extends SubsystemBase {
  private final WPI_VictorSPX masterMotor = IntakeConstants.UPPER_MOTOR_ID.get();
  private final WPI_VictorSPX slaveMotor = IntakeConstants.LOWER_MOTOR_ID.get();
  private final DigitalInput beamBreakSensor = new DigitalInput(IntakeConstants.BEAM_BREAK_SENSOR_ID);

  private boolean locked = false;

  public IntakeSub() {
    masterMotor.configFactoryDefault();
    slaveMotor.configFactoryDefault();

    slaveMotor.follow(masterMotor);

    addChild("Master Motor", masterMotor);
    addChild("Slave Motor", slaveMotor);
    addChild("Beam Broken", beamBreakSensor);
  }

  @Override
  public void periodic() {
    final var isRunning = masterMotor.get() != 0.0;

    ShuffleControl.miscTab.setIntakeVars(locked, isRunning);
    locked = noteIsPresent();
    if (isRunning && locked)
      stop();
  }

  /**
   * Set the speed of the intake
   * 
   * @param speed Speed from -1 to 1
   */
  public void set(double speed) {
    set(speed, false);
  }

  /**
   * Set the speed of the intake
   * 
   * @param speed        Speed from -1 to 1
   * @param overrideLock overrides the lock imposed by the beam break sensor.
   *                     mainly used for shooting.
   */
  public void set(double speed, boolean overrideLock) {
    // if (locked && !overrideLock)
    // return;

    // isRunning = true;
    speed = MathUtil.clamp(speed, -1, 1);
    masterMotor.set(speed);
  }

  /** Stops the intake motor */
  public void stop() {
    masterMotor.stopMotor();
  }

  public boolean noteIsPresent() {
    return beamBreakSensor.get();
  }

}
