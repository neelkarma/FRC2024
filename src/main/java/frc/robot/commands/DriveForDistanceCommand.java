package frc.robot.commands;

import java.util.Arrays;
import java.util.Comparator;
import java.util.function.BooleanSupplier;
import java.util.stream.Stream;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.units.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.DriveSub;
import frc.robot.utils.logger.Logger;

public class DriveForDistanceCommand extends InstantCommand {
  Translation2d startP;
  double xSpeed = 0;
  double ySpeed = 0;
  double dist = 0;

  public DriveForDistanceCommand(double xspeed, double ySpeed, double dist) {
    this.xSpeed = xspeed;
    this.ySpeed = ySpeed;
    this.dist = dist;
  }
  
  @Override
  public void initialize(){
    startP = Subsystems.drive.getPose().getTranslation();
    Subsystems.drive.drive(xSpeed,ySpeed,0,false,true);
  }

  @Override
  public boolean isFinished() {
    return startP.getDistance(Subsystems.drive.getPose().getTranslation()) > dist;
  }
}
