// Originally from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/subsystems/DriveSubsystem.java

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.DriveConstants;
import frc.robot.shufflecontrol.ShuffleControl;
import frc.robot.utils.SwerveModule;
import frc.robot.utils.PhotonBridge;
import frc.robot.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSub extends SubsystemBase {
  // Swerve Modules
  public final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
      DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
      DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule frontRight = new SwerveModule(
      DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
      DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
      DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule backLeft = new SwerveModule(
      DriveConstants.BACK_LEFT_DRIVING_CAN_ID,
      DriveConstants.BACK_LEFT_TURNING_CAN_ID,
      DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final SwerveModule backRight = new SwerveModule(
      DriveConstants.BACK_RIGHT_DRIVING_CAN_ID,
      DriveConstants.BACK_RIGHT_TURNING_CAN_ID,
      DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);

  public SwerveModuleState[] lastStates = {
      frontLeft.getState(),
      frontRight.getState(),
      backLeft.getState(),
      backRight.getState()
  };
  private int updateShuffleCounter = 0;

  // The gyro sensor
  private final ADIS16470_IMU imu = new ADIS16470_IMU();

  // Photon Bridge
  // public final PhotonBridge photon = new PhotonBridge();

  // Field for robot viz
  private final Field2d field = new Field2d();

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.ROTATIONAL_SLEW_RATE);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  // Pose estimation class for tracking robot pose
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.DRIVE_KINEMATICS,
      Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kZ)),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
      }, new Pose2d());

  // Simulation Variables
  ADIS16470_IMUSim imuSim = new ADIS16470_IMUSim(imu);
  private Pose2d poseSim = new Pose2d();

  /** Creates a new DriveSubsystem. */
  public DriveSub() {
    // Configure PathPlanner auto
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::resetOdometry,
        this::getRelativeChassisSpeeds,
        this::driveRelative,
        new HolonomicPathFollowerConfig(
            DriveConstants.AUTO_TRANSLATION_PID,
            DriveConstants.AUTO_ROTATION_PID,
            DriveConstants.MAX_MODULE_SPEED,
            DriveConstants.DRIVEBASE_RADIUS,
            new ReplanningConfig()),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        },
        this);
    SmartDashboard.putData(field);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    if (updateShuffleCounter > DriveConstants.updateShuffleInterval) {
      ShuffleControl.driveTab.setWheelAxes(lastStates[0], lastStates[1], lastStates[2], lastStates[3]);
      updateShuffleCounter = 0;
    } else {
      updateShuffleCounter++;
    }
    updateOdometry();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return RobotBase.isSimulation() ? poseSim : odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    if (RobotBase.isSimulation()) {
      imuSim.setGyroAngleZ(pose.getRotation().getDegrees());
      poseSim = pose;
      return;
    }

    odometry.resetPosition(
        getHeading(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        pose);
  }

  // For reference, the below code is the actual drive function if the last
  // argument is false
  //
  // public void drive(double xSpeed, double ySpeed, double rot) {
  // var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
  // ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading()));
  //
  // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates,
  // DriveConstants.MAX_SPEED);
  //
  // frontLeft.setDesiredState(states[0]);
  // frontRight.setDesiredState(states[1]);
  // backLeft.setDesiredState(states[2]);
  // backRight.setDesiredState(states[3]);
  // }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    if (xSpeed == 0 && ySpeed == 0) {
      resetIntegral();
    }
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.DIRECTION_SLEW_RATE / currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        if(inputTranslationMag != 0)
            currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        else
            currentTranslationMag = 0;
      } else if (angleDif > 0.85 * Math.PI) {
        if (currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        } else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          
          if(inputTranslationMag != 0)
            currentTranslationMag = magLimiter.calculate(inputTranslationMag);
          else
            currentTranslationMag = 0;
        }
      } else {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;

      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.MAX_SPEED;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.MAX_SPEED;
    double rotDelivered = currentRotation * DriveConstants.MAX_ANGULAR_SPEED;

    // System.out.println(xSpeedDelivered + " " + ySpeedDelivered + " " +
    // rotDelivered +" "+ getHeading());

    var swerveModuleStates = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                getHeading())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED);

    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);

    lastStates = new SwerveModuleState[] {
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState()
    };
  }

  /**
   * Drives with robot-relative ChassisSpeeds.
   * 
   * Mainly used by the auto.
   */
  public void driveRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false, false);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  /** reset turn motor pid I accumulation to 0 */
  public void resetIntegral() {
    frontLeft.resetIntegral();
    backLeft.resetIntegral();
    frontRight.resetIntegral();
    backRight.resetIntegral();

  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    imu.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading as a Rotation2d
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kZ));
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return imu.getRate(IMUAxis.kZ) * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  public double estimateDist() {
    double d = (frontLeft.getPosition().distanceMeters) / 3.5151856018;
    return d;
  }

  /**
   * Updates the robot's odometry.
   * 
   * This should be called every robot tick, in the periodic function.
   */
  public void updateOdometry() {
    odometry.update(
        getHeading(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
        //System.out.println(frontLeft.getPosition() + " " +
        //    frontRight.getPosition() + " " +
        //    backLeft.getPosition() + " " +
        //    backRight.getPosition());

    // photon.getEstimatedGlobalPose()
    // .ifPresent((visionResult) -> {
    // var visionPose = visionResult.estimatedPose.toPose2d();

    // // Reject any egregiously incorrect vision pose estimates
    // if (visionPose.getTranslation().getDistance(getPose().getTranslation()) > 1)
    // return;

    // odometry.addVisionMeasurement(visionPose, 0.02);
    // });

    field.setRobotPose(getPose());
  }

  public ChassisSpeeds getRelativeChassisSpeeds() {
    return DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  /** not really intended for more accuracy than logging requires */
  public double getSpeedMS(){
    ChassisSpeeds speed = getRelativeChassisSpeeds();
    return Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);
  }

  @Override
  public void simulationPeriodic() {
    final var speeds = DriveConstants.DRIVE_KINEMATICS.toChassisSpeeds(
        frontLeft.getDesiredState(),
        frontRight.getDesiredState(),
        backLeft.getDesiredState(),
        backRight.getDesiredState());

    imuSim.setGyroRateZ(speeds.omegaRadiansPerSecond * (180 / Math.PI));
    imuSim.setGyroAngleZ(getHeading().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * 0.02)).getDegrees());

    poseSim = poseSim.exp(
        new Twist2d(
            speeds.vxMetersPerSecond * 0.02,
            speeds.vyMetersPerSecond * 0.02,
            speeds.omegaRadiansPerSecond * 0.02));

    // photon.simulationPeriodic(getPose());
  }
}