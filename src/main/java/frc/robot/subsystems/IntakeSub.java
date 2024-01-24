package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.IntakeConstants;

public class IntakeSub extends SubsystemBase {
  private final WPI_TalonSRX masterMotor = IntakeConstants.MOTOR_1_ID.build();
  private final WPI_TalonSRX slaveMotor = IntakeConstants.MOTOR_2_ID.build();
  private final DigitalInput beamBreakSensor = new DigitalInput(IntakeConstants.BEAM_BREAK_SENSOR_ID);
  private boolean locked = false;

  public IntakeSub() {
    masterMotor.configFactoryDefault();
    slaveMotor.configFactoryDefault();

    slaveMotor.follow(masterMotor);
  }

  @Override
  public void periodic() {
    boolean robotContainsNote = beamBreakSensor.get();
    locked = robotContainsNote;
  }

  public void set(double newSpeed) {
    if (locked) return;
    masterMotor.set(newSpeed);
  }

  public void stop() {
    if (locked) return;
    masterMotor.stopMotor();
  }
}
