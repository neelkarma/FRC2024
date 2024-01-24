package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.IntakeConstants;

public class IntakeSub extends SubsystemBase {
    private TalonSRX motor1 = new TalonSRX(IntakeConstants.MOTOR_1_ID);
    private TalonSRX motor2 = new TalonSRX(IntakeConstants.MOTOR_2_ID);

    private double speed = 0.5;

    public IntakeSub() {
      motor1.configFactoryDefault();
      motor2.configFactoryDefault();
    }

    public void startIntake() {
      motor1.set(ControlMode.PercentOutput, speed);
      motor2.set(ControlMode.PercentOutput, -speed);
    }

    public void startOuttake() {
      motor1.set(ControlMode.PercentOutput, -speed);
      motor2.set(ControlMode.PercentOutput, speed);
    }

    public void setSpeed(double newSpeed) {
      speed = newSpeed;
    }

    public void end() {
      motor1.set(ControlMode.PercentOutput, 0);
      motor2.set(ControlMode.PercentOutput, 0);
    }
}
