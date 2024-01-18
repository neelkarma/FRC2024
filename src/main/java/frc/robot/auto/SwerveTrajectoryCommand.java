package frc.robot.auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;
import frc.robot.utils.logger.Logger;

public class SwerveTrajectoryCommand extends SequentialCommandGroup {
  public SwerveTrajectoryCommand(Trajectory trajectory) {
    // 1. Define PID controllers for tracking trajectory
    var xController = new PIDController(DriveConstants.AUTO_X_P, 0, 0);
    var yController = new PIDController(DriveConstants.AUTO_Y_P, 0, 0);
    var thetaController = new ProfiledPIDController(
        DriveConstants.AUTO_THETA_P, 0, 0, DriveConstants.AUTO_THETA_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 2. Construct command to follow trajectory
    var swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        Subsystems.drive::getPose,
        DriveConstants.kDriveKinematics,
        xController,
        yController,
        thetaController,
        Subsystems.drive::setModuleStates,
        Subsystems.drive);

    // 3. Add some init and wrap-up, and add all commands
    addCommands(
        new InstantCommand(() -> Subsystems.drive.resetOdometry(trajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> Subsystems.drive.setX()));
  }

  public static SwerveTrajectoryCommand fromPathWeaver(String pathweaverPath) {
    Trajectory trajectory = new Trajectory();

    // Reads trajectory from the specified filepath
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathweaverPath);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      Logger.info("SwerveTrajectoryCommand.fromPathWeaver : Loaded path " + trajectoryPath);
    } catch (IOException e) {
      Logger.error("SwerveTrajectoryCommand.fromPathWeaver : Unable to open trajectory: " + pathweaverPath + e);
    }

    return new SwerveTrajectoryCommand(trajectory);
  }
}