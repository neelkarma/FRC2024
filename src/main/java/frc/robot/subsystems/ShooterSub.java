package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.shufflecontrol.ShuffleControl;

public class ShooterSub extends SubsystemBase {
  private final WPI_TalonSRX upperMotor = ShooterConstants.UPPER_MOTOR_ID.get();
  private final WPI_TalonSRX lowerMotor = ShooterConstants.LOWER_MOTOR_ID.get();

  public double ampSpd = 0.6;
  public double ampDiff = -0.22;
  public double SpeakerSpd = 1;
  public double speakerDiff = 0.05;

  public ShooterSub() {
    addChild("Master Motor", upperMotor);
    addChild("Slave Motor", lowerMotor);
  }

  /**
   * Sets the speed of the shooter wheel.
   * 
   * @param speed Speed, from -1 to 1.
   */
  public void setSpeed(double speed, double diff) {
    double speedUp = MathUtil.clamp(speed * (1 + diff), -1, 1);
    double speedDown = MathUtil.clamp(speed * (1 - diff), -1, 1);
    upperMotor.set(speedUp);
    lowerMotor.set(speedDown);
  }

  public void setAmp(){
    ShuffleControl.driveTab.updateShooterSpd();
    setSpeed(ampSpd, ampDiff);
  }
  public void setSpeaker(){
    ShuffleControl.driveTab.updateShooterSpd();
    setSpeed(SpeakerSpd, speakerDiff);
  }

  /** Stops the shooter. Call `setSpeed` to start again. */
  public void stop() {
    upperMotor.stopMotor();
    lowerMotor.stopMotor();
  }
}
