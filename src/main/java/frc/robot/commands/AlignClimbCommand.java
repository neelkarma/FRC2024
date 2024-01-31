package frc.robot.commands;

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems;
import frc.robot.auto.SwerveTrajectoryCommand;
import frc.robot.utils.logger.Logger;

public class AlignClimbCommand extends InstantCommand {
  private static final Pose2d[] RED_CLIMB_POSES = new Pose2d[] {
      new Pose2d(10, 4, Rotation2d.fromDegrees(0)),
      new Pose2d(12.5, 5.2, Rotation2d.fromDegrees(-120)),
      new Pose2d(12.5, 3, Rotation2d.fromDegrees(120)) };
  private static final Pose2d[] BLUE_CLIMB_POSES = new Pose2d[] {
      new Pose2d(6, 4, Rotation2d.fromDegrees(180)),
      new Pose2d(3.5, 5.2, Rotation2d.fromDegrees(-60)),
      new Pose2d(3.5, 3, Rotation2d.fromDegrees(60))
  };

  public AlignClimbCommand(BooleanSupplier continueCondition) {
    super(() -> {
      var targetPose = DriverStation.getAlliance()
          // If Driver.getAlliance() is present, use only the climb poses for our alliance
          .map((alliance) -> {
            if (alliance == Alliance.Red) {
              return Arrays.stream(RED_CLIMB_POSES);
            } else {
              return Arrays.stream(BLUE_CLIMB_POSES);
            }
          })
          // Otherwise, just use both red and blue climb poses
          .orElse(Stream.concat(Arrays.stream(RED_CLIMB_POSES), Arrays.stream(BLUE_CLIMB_POSES)))
          // Find the climb pose that's closest to us
          .min(Comparator
              .comparing((pose) -> Subsystems.drive.getPose().getTranslation().getDistance(pose.getTranslation())))
          .get();

      try {
        var trajectory = TrajectoryGenerator.generateTrajectory(
            List.of(
                Subsystems.drive.getPose(),
                targetPose),
            new TrajectoryConfig(2, 1));

        // I can't find a way to have dynamically instantiated commands in
        // SequentialCommandGroup, so we have to resort to this terribleness instead
        new SwerveTrajectoryCommand(trajectory).onlyWhile(continueCondition).schedule();
      } catch (TrajectoryGenerationException e) {
        Logger.error("AlignClimbCommand : Trajectory Generation Failed!");
      }
    });
  }
}
