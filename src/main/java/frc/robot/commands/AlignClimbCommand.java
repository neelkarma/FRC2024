package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.utils.Range;

public class AlignClimbCommand extends Command {
  private static final Translation2d TARGET_TRANSLATION = new Translation2d(0, 0);

  public final PIDController transXController = new PIDController(0.1, 0, 0);
  public final PIDController transYController = new PIDController(0.1, 0, 0);
  public final PIDController rotController = new PIDController(0.5, 0, 0);

  private boolean shouldFinish = false;

  private int targetId = -1;

  public AlignClimbCommand() {
    transXController.setSetpoint(TARGET_TRANSLATION.getX());
    transYController.setSetpoint(TARGET_TRANSLATION.getY());
    rotController.setSetpoint(0);

    addRequirements(Subsystems.drive);
  }

  @Override
  public void initialize() {
    var result = Subsystems.drive.photon.getLatestPipelineResult();

    if (!result.hasTargets()) {
      shouldFinish = true;
      return;
    }

    var target = result.getBestTarget();

    // Ignore any non-climb apriltags
    var range = DriverStation.getAlliance().map((alliance) -> {
      if (alliance == Alliance.Red) {
        return new Range(11, 13);
      } else {
        return new Range(14, 16);
      }
    }).orElse(new Range(11, 16));
    System.out.println(range);

    if (!range.contains(target.getFiducialId())) {
      shouldFinish = true;
      return;
    }

    targetId = target.getFiducialId();
  }

  @Override
  public void execute() {
    // TrajectoryGenerator.generateTrajectory(
    // List.of(Subsystems.drive.getPose(), new Pose2d(10, 4,
    // Rotation2d.fromDegrees(0))), new TrajectoryConfig(1, 1));

    var result = Subsystems.drive.photon.getLatestPipelineResult();

    if (!result.hasTargets()) {
      shouldFinish = true;
      return;
    }

    var target = result.getBestTarget();

    if (target.getFiducialId() != targetId) {
      shouldFinish = true;
      return;
    }

    System.out.println(target.getBestCameraToTarget());
    var errorTransform = target.getBestCameraToTarget();
    var errorRotation = errorTransform.getRotation();
    var errorTranslation = errorTransform.getTranslation();

    var inputX = -transXController.calculate(errorTranslation.getX());
    var inputY = -transYController.calculate(errorTranslation.getZ());
    var inputRot = rotController.calculate(new Rotation2d(errorRotation.getX(), errorRotation.getZ()).getRotations());

    Subsystems.drive.drive(inputX, inputY, inputRot, false, true);
  }

  public boolean isFinished() {
    return shouldFinish;
  }

  @Override
  public void end(boolean interrupted) {
    transXController.reset();
    transYController.reset();
    rotController.reset();
    shouldFinish = false;
    targetId = -1;
  }

}
