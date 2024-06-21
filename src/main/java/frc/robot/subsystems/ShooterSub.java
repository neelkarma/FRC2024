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
  public double SpeakerSpd = 0.75;
  public double speakerDiff = 0.25;

  public ShooterSub() {
    addChild("Master Motor", upperMotor);
    addChild("Slave Motor", lowerMotor);
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

  public void setSpeed(double speed, double spin) {
    var upperSpeed = speed * (1 + spin);
    var lowerSpeed = speed * (1 - spin);

    final var upperAbs = Math.abs(upperSpeed);
    final var lowerAbs = Math.abs(lowerSpeed);

    upperSpeed -= Math.copySign(Math.max(upperAbs - 1, 0), upperSpeed);
    lowerSpeed -= Math.copySign(Math.max(lowerAbs - 1, 0), lowerSpeed);

    upperMotor.set(MathUtil.clamp(upperSpeed, -1, 1));
    lowerMotor.set(MathUtil.clamp(lowerSpeed, -1, 1));
  }
}
