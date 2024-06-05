package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.IntakeConstants;

public class ClimberSub extends SubsystemBase {
  private final WPI_VictorSPX motor = ClimberConstants.MOTOR_1_ID.get();
  private final Servo servo = new Servo(ClimberConstants.SERVO_PORT);
  private final DigitalInput climberStop = new DigitalInput(ClimberConstants.CLIMBER_STOP_PORT);

  private double targetSpeed = 0;

  public ClimberSub() {
    motor.configFactoryDefault();
    addChild("Motor", motor);
    addChild("Servo", servo);
  }

  @Override
  public void periodic() {
    //var servoUnlocked = servo.getPosition() == ClimberConstants.SERVO_LOCKED_POS;
    var motorIsCommanded = targetSpeed != 0;

    if (!motorIsCommanded) {
      motor.set(0);
      servo.setPosition(ClimberConstants.SERVO_LOCKED_POS);
    } else if (motorIsCommanded) {
      motor.set(targetSpeed);
      servo.setPosition(ClimberConstants.SERVO_UNLOCKED_POS);
    }
  }

  /**
   * Set the speed of the intake and takes necessary servo actions.
   * 
   * @param speed Speed from -1 to 1
   */
  public void set(double speed) {
    targetSpeed = MathUtil.clamp(speed, -1, 1);
  }

  /** Stops the intake motor, taking necessary servo actions. */
  public void stop() {
    set(0);
  }
}