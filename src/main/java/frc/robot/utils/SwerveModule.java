// Originally from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/MAXSwerveModule.java

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.DriveConstants;

public class SwerveModule {
  private final TalonFX driveMotor;
  private final CANSparkMax turnMotor;

  private final AbsoluteEncoder turnEncoder;

  private final VelocityVoltage driveController;
  private final SparkPIDController turnController;

  private double angularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SwerveModule(int drivingCANId, int turningCANId, double angularOffset) {
    driveMotor = new TalonFX(drivingCANId);
    turnMotor = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Configure Drive Motor
    final var driveMotorConfig = new TalonFXConfiguration();
    /* Motor Inverts and Neutral Mode */
    driveMotorConfig.MotorOutput.Inverted = DriveConstants.DRIVE_MOTOR_INVERTED;
    driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    /* Gear Ratio Config */
    driveMotorConfig.Feedback.SensorToMechanismRatio = DriveConstants.DRIVE_GEAR_RATIO;

    /* Current Limiting */
    driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;

    /* PID Config */
    driveMotorConfig.Slot0.kP = DriveConstants.DRIVE_P;
    driveMotorConfig.Slot0.kI = DriveConstants.DRIVE_I;
    driveMotorConfig.Slot0.kD = DriveConstants.DRIVE_D;

    driveMotor.getConfigurator().apply(driveMotorConfig);
    driveController = new VelocityVoltage(0).withSlot(0);

    // Configure Turn Motor
    turnMotor.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    turnEncoder = turnMotor.getAbsoluteEncoder(Type.kDutyCycle);
    turnController = turnMotor.getPIDController();
    turnController.setFeedbackDevice(turnEncoder);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turnEncoder.setPositionConversionFactor(DriveConstants.TURNING_ENCODER_POSITION_FACTOR);
    turnEncoder.setVelocityConversionFactor(DriveConstants.TURNING_ENCODER_VELOCITY_FACTOR);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    turnEncoder.setInverted(DriveConstants.TURNING_ENCODER_INVERTED);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turnController.setPositionPIDWrappingEnabled(true);
    turnController.setPositionPIDWrappingMinInput(DriveConstants.TURNING_ENCODER_POSITION_PID_MIN_INPUT);
    turnController.setPositionPIDWrappingMaxInput(DriveConstants.TURNING_ENCODER_POSITION_PID_MAX_INPUT);

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    turnController.setP(DriveConstants.TURNING_P);
    turnController.setI(DriveConstants.TURNING_I);
    turnController.setD(DriveConstants.TURNING_D);
    turnController.setFF(DriveConstants.TURNING_FF);
    turnController.setOutputRange(-1, 1);

    turnMotor.setIdleMode(IdleMode.kBrake);
    turnMotor.setSmartCurrentLimit(30);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    turnMotor.burnFlash();

    this.angularOffset = angularOffset;
    desiredState.angle = new Rotation2d(turnEncoder.getPosition());
    driveMotor.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(driveMotor.getVelocity().getValue(),
        new Rotation2d(turnEncoder.getPosition() - angularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        driveMotor.getPosition().getValue(),
        new Rotation2d(turnEncoder.getPosition() - angularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(angularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(
        correctedDesiredState,
        new Rotation2d(turnEncoder.getPosition()));

    // Command driving and turning motors towards their respective setpoints.
    driveMotor.setControl(
        driveController
            .withVelocity(
                // withVelocity accepts rps, not mps
                optimizedDesiredState.speedMetersPerSecond / DriveConstants.WHEEL_CIRCUMFERENCE_METERS)
            .withFeedForward(0.5));
    turnController.setReference(
        optimizedDesiredState.angle.getRadians(),
        CANSparkMax.ControlType.kPosition);

    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    driveMotor.setPosition(0);
  }
}