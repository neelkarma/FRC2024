package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.IntakeConstants;

public class ClimberSub extends SubsystemBase {
  private final WPI_VictorSPX motor = ClimberConstants.MOTOR_1_ID.get();
  private final Servo servo = new Servo(ClimberConstants.SERVO_PORT);
  private final DigitalInput climberStop = new DigitalInput(ClimberConstants.CLIMBER_STOP_PORT);

  private double targetSpeed = 0;
  private Timer timer = new Timer();

  public ClimberSub() {
    timer.start();
    motor.configFactoryDefault();
    addChild("Motor", motor);
    addChild("Servo", servo);
  }

  public void initialize(){
    servo.setPosition(ClimberConstants.SERVO_UNLOCKED_POS);
    timer.reset();
  }

  @Override
  public void periodic() {
    if(targetSpeed != 0 && timer.get()>0.5){
      motor.set(targetSpeed);
    } else {
      motor.stopMotor();
    }
  }

  /**
   * Set the speed of the intake and takes necessary servo actions.
   * 
   * @param speed Speed from -1 to 1
   */
  public void set(double speed) {
    if(targetSpeed == 0)
      initialize();

      targetSpeed = MathUtil.clamp(speed, -1, 1);
  }

  /** Stops the intake motor, taking necessary servo actions. */
  public void stop() {
    targetSpeed = 0;
    motor.stopMotor();

    servo.setPosition(ClimberConstants.SERVO_LOCKED_POS);
  }
}