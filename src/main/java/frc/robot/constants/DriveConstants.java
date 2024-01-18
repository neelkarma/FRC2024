// Originally from https://github.com/REVrobotics/MAXSwerve-Java-Template/blob/main/src/main/java/frc/robot/Constants.java

package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class DriveConstants {
  // NEO Motor Constants
  public static final double FREE_SPEED_RPM = 5676;

  // Drive Constants

  // Driving Parameters - Note that these are not the maximum capable speeds of
  // the robot, rather the allowed maximum speeds
  public static final double MAX_SPEED = 4.8;
  public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second
  public static final double MAX_ANGULAR_ACCELERATION = MAX_ANGULAR_SPEED / 60; // radians per second squared

  public static final double DIRECTION_SLEW_RATE = 1.2; // radians per second
  public static final double MAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
  public static final double ROTATIONAL_SLEW_RATE = 2.0; // percent per second (1 = 100%)

  public static final double DRIVE_GEAR_RATIO = 4.71;

  // Chassis configuration
  public static final double TRACK_WIDTH = Units.inchesToMeters(26.5);
  // Distance between centers of right and left wheels on robot
  public static final double WHEEL_BASE = Units.inchesToMeters(26.5);
  // Distance between front and back wheels on robot
  public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
      new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
      new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

  // Angular offsets of the modules relative to the chassis in radians
  public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
  public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
  public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
  public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

  // SPARK MAX CAN IDs
  public static final int FRONT_LEFT_DRIVING_CAN_ID = 11;
  public static final int REAR_LEFT_DRIVING_CAN_ID = 13;
  public static final int FRONT_RIGHT_DRIVING_CAN_ID = 15;
  public static final int REAR_RIGHT_DRIVING_CAN_ID = 17;

  public static final int FRONT_LEFT_TURNING_CAN_ID = 10;
  public static final int REAR_LEFT_TURNING_CAN_ID = 12;
  public static final int FRONT_RIGHT_TURNING_CAN_ID = 14;
  public static final int REAR_RIGHT_TURNING_CAN_ID = 16;

  public static final boolean GYRO_REVERSED = false;

  // Module Constants

  // The MAXSwerve module can be configured with one of three pinion gears: 12T,
  // 13T, or 14T.
  // This changes the drive speed of the module (a pinion gear with more teeth
  // will result in a
  // robot that drives faster).
  public static final int DRIVING_MOTOR_PINION_TEETH = 14;

  // Invert the turning encoder, since the output shaft rotates in the opposite
  // direction of
  // the steering motor in the MAXSwerve Module.
  public static final InvertedValue DRIVE_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
  public static final boolean TURNING_ENCODER_INVERTED = true;

  // Calculations required for driving motor conversion factors and feed forward
  public static final double DRIVING_MOTOR_FREE_SPEED_RPS = FREE_SPEED_RPM / 60;
  public static final double WHEEL_DIAMETER_METERS = 0.0762;
  public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
  // teeth on the bevel pinion
  public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
  public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS
      * WHEEL_CIRCUMFERENCE_METERS)
      / DRIVING_MOTOR_REDUCTION;

  public static final double DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * Math.PI)
      / DRIVING_MOTOR_REDUCTION; // meters
  public static final double DRIVING_ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMETER_METERS * Math.PI)
      / DRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

  public static final double TURNING_ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
  public static final double TURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

  public static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
  public static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

  public static final double DRIVE_P = 0.04;
  public static final double DRIVE_I = 0;
  public static final double DRIVE_D = 0;
  public static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS;

  public static final double TURNING_P = 1;
  public static final double TURNING_I = 0;
  public static final double TURNING_D = 0;
  public static final double TURNING_FF = 0;

  // Auto Constants
  public static final double AUTO_X_P = 0;
  public static final double AUTO_Y_P = 0;
  public static final double AUTO_THETA_P = 3;

  public static final TrapezoidProfile.Constraints AUTO_THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      MAX_ANGULAR_SPEED,
      MAX_ANGULAR_ACCELERATION);

}
