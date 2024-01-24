package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.IntakeConstants;

public class IntakeSub extends SubsystemBase {
  private final WPI_TalonSRX masterMotor = IntakeConstants.MOTOR_1_ID.build();
  private final WPI_TalonSRX slaveMotor = IntakeConstants.MOTOR_2_ID.build();

  public IntakeSub() {
    masterMotor.configFactoryDefault();
    slaveMotor.configFactoryDefault();

    slaveMotor.follow(masterMotor);
  }

  public void set(double newSpeed) {
    masterMotor.set(newSpeed);
  }

  public void stop() {
    masterMotor.stopMotor();
  }
}
