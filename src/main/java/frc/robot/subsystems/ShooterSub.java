package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;
import frc.robot.shufflecontrol.ShuffleControl;

public class ShooterSub extends SubsystemBase {

  private final WPI_TalonFX upperMotor = new WPI_TalonFX(5); //ShooterConstants.UPPER_MOTOR_ID.get();
  private final WPI_TalonFX lowerMotor = new WPI_TalonFX(6);//ShooterConstants.LOWER_MOTOR_ID.get();
  
  private boolean enableShooter = false;
  private final PIDController upperPid = new PIDController(0.1, 0, 0);
  private final PIDController lowerPid = new PIDController(0.1, 0, 0);
  
  public double ampSpd = 0.6;
  public double ampDiff = -0.22;
  public double speakerSpd = 0.6;
  public double speakerDiff = 0.25;

  public ShooterSub() {
    addChild("Master Motor", upperMotor);
    addChild("Slave Motor", lowerMotor);
    upperPid.setTolerance(0.1);    
    lowerPid.setTolerance(0.1);

  }

  private long counter = 0;
  
  @Override
  public void periodic() {

    if (enableShooter) {
      double upperVelocity = -upperMotor.getSelectedSensorVelocity();
      double lowerVelocity = -lowerMotor.getSelectedSensorVelocity();

      double upperPidVelocity = upperPid.calculate(upperVelocity);
      double lowerPidVelocity = lowerPid.calculate(lowerVelocity);
      
      if (counter % 10 == 0) {
        System.out.println("upper " + upperVelocity + " -> " + upperPidVelocity/100);
        System.out.println("lower " + lowerVelocity + " -> " + lowerPidVelocity/100);
        System.out.println("------------------------");
      }

      upperMotor.set(upperPidVelocity*10);
      lowerMotor.set(lowerPidVelocity*10);

      counter++;
    }

  }
  
  /** Stops the shooter. Call `setSpeed` to start again. */
  public void stop() {
    upperPid.setSetpoint(0);
    lowerPid.setSetpoint(0);
    enableShooter = false;
    
    upperMotor.stopMotor();
    lowerMotor.stopMotor();
  }

  public void setSpeed(double speed, double spin) {

    var upperSpeed = speed * (1 + spin);
    var lowerSpeed = speed * (1 - spin);

    //final var upperAbs = Math.abs(upperSpeed);
    //final var lowerAbs = Math.abs(lowerSpeed);

    //upperSpeed -= Math.copySign(Math.max(upperAbs - 1, 0), upperSpeed);
    //lowerSpeed -= Math.copySign(Math.max(lowerAbs - 1, 0), lowerSpeed);

    //upperPid.setSetpoint(upperSpeed);
    //lowerPid.setSetpoint(lowerSpeed);

    upperMotor.set(upperSpeed);
    lowerMotor.set(lowerSpeed);

    enableShooter = false;

  }
  
  public void setAmp(){
    ShuffleControl.driveTab.updateShooterSpd();
    setSpeed(ampSpd, ampDiff);
  }
  public void setSpeaker(){
    ShuffleControl.driveTab.updateShooterSpd();
    setSpeed(speakerSpd, speakerDiff);
  }
}
