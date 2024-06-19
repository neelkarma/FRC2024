// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoProvider;
import frc.robot.teleop.TeleopProvider;
import frc.robot.commands.AlignClimbCommand;
import frc.robot.commands.FlashLEDCommand;
import frc.robot.commands.SolidLEDCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including
 * Subsystemsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final AutoProvider autoProvider = AutoProvider.getInstance();
  private final TeleopProvider teleopProvider = TeleopProvider.getInstance();

  /**
   * The container for the robot. Contains Subsystemsystems, OI devices, and
   * commands.
   */
  public RobotContainer() {
    // configure auto named commands
    NamedCommands.registerCommand(
        "shoot",
        new SequentialCommandGroup(
            Subsystems.shooter.runOnce(() -> Subsystems.shooter.setSpeed(1, 0)),
            new WaitCommand(0.5),
            Subsystems.intake.runOnce(() -> Subsystems.intake.set(1)),
            new WaitCommand(0.5),
            Subsystems.intake.runOnce(Subsystems.intake::stop),
            Subsystems.shooter.runOnce(Subsystems.shooter::stop)));

    NamedCommands.registerCommand(
        "startIntake",
        Subsystems.intake.runOnce(() -> Subsystems.intake.set(0.3)));

    NamedCommands.registerCommand(
        "stopIntake",
        Subsystems.intake.runOnce(Subsystems.intake::stop));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Robot Automations
    // make leds green if note is present in robot
    new Trigger(Subsystems.intake::noteIsPresent).whileTrue(new SolidLEDCommand(Color.kGreen));
    // flash leds yellow during endgame
    new Trigger(() -> DriverStation.isTeleop() && DriverStation.getMatchTime() <= 30)
        .whileTrue(new FlashLEDCommand(Color.kYellow, 1));

    // +----------------+
    // | PILOT CONTROLS |
    // +----------------+

    // --- Manual Controls ---

    // Invert Drive
    //OI.pilot.start().onTrue(new InstantCommand(() -> Variables.invertDriveDirection = !Variables.invertDriveDirection));

    // intake reverse
    OI.pilot.leftBumper()
        .whileTrue(
            new StartEndCommand(
                () -> Subsystems.intake.set(-0.2),
                Subsystems.intake::stop,
                Subsystems.intake));
    // intake
    OI.pilot.leftTrigger()
        .whileTrue(
            new StartEndCommand(
                () -> Subsystems.intake.set(0.2),
                Subsystems.intake::stop,
                Subsystems.intake));
    // intake reverse
    OI.copilot.leftBumper()
        .whileTrue(
            new StartEndCommand(
                () -> Subsystems.intake.set(-0.2),
                Subsystems.intake::stop,
                Subsystems.intake));
    // intake
    OI.copilot.leftTrigger()
        .whileTrue(
            new StartEndCommand(
                () -> Subsystems.intake.set(0.2),
                Subsystems.intake::stop,
                Subsystems.intake));
    // intake rapid
    OI.copilot.y()
        .whileTrue(
            new StartEndCommand(
                () -> Subsystems.intake.set(1),
                Subsystems.intake::stop,
                Subsystems.intake));
    // shoot
    OI.copilot.rightTrigger()
        .whileTrue(
            new StartEndCommand(
                () -> Subsystems.shooter.setAmp(),
                Subsystems.shooter::stop,
                Subsystems.shooter));
    // shoot
    OI.copilot.rightBumper()
        .whileTrue(
            new StartEndCommand(
                () -> Subsystems.shooter.setSpeed(.98, 0),
                Subsystems.shooter::stop,
                Subsystems.shooter));

    OI.pilot.start()
        .onTrue(
            new InstantCommand(() -> Subsystems.drive.zeroHeading(), Subsystems.drive));

    // pivot up
    // OI.pilot.povRight().whileTrue(Subsystems.pivot.run(Subsystems.pivot::up));

    // pivot down
    // OI.pilot.povLeft().whileTrue(Subsystems.pivot.run(Subsystems.pivot::down));

    // climber up
    OI.pilot.povDown().whileTrue(
        new StartEndCommand(
            () -> Subsystems.climber.set(1),
            Subsystems.climber::stop,
            Subsystems.climber));

    // climber down
    OI.pilot.povUp().whileTrue(
        new StartEndCommand(
            () -> Subsystems.climber.set(-0.5),
            Subsystems.climber::stop,
            Subsystems.climber));
    // climber up
    OI.copilot.povDown().whileTrue(
        new StartEndCommand(
            () -> Subsystems.climber.set(1),
            Subsystems.climber::stop,
            Subsystems.climber));

    // climber down
    OI.copilot.povUp().whileTrue(
        new StartEndCommand(
            () -> Subsystems.climber.set(-0.5),
            Subsystems.climber::stop,
            Subsystems.climber));

    // --- Recipes ---

    // auto climb align
    // OI.pilot.back().whileTrue(new AlignClimbCommand(() ->
    // OI.pilot.getHID().getBackButton()));

    // amp shooting sequence
    /*
     * OI.pilot.x()
     * .whileTrue(
     * new SequentialCommandGroup(
     * // At the same time:
     * // - Stop the intake
     * // - Start pivoting up
     * // - Start speeding up the shooter
     * new ParallelCommandGroup(
     * Subsystems.intake.runOnce(Subsystems.intake::stop),
     * Subsystems.pivot.runOnce(Subsystems.pivot::up),
     * Subsystems.shooter.runOnce(() -> Subsystems.shooter.setSpeed(1))),
     * 
     * // Wait until the pivot is finished and the shooter is at the desired speed
     * new WaitCommand(0.5),
     * 
     * // Shoot
     * Subsystems.intake.runOnce(() -> Subsystems.intake.set(1, true)),
     * 
     * // Wait until the note is no longer in the robot, and some more after that
     * // for completeness
     * new WaitUntilCommand(() -> !Subsystems.intake.noteIsPresent()),
     * new WaitCommand(0.5),
     * 
     * // Pivot back down and stop shooting
     * new ParallelCommandGroup(
     * Subsystems.intake.runOnce(Subsystems.intake::stop),
     * Subsystems.shooter.runOnce(Subsystems.shooter::stop),
     * Subsystems.pivot.runOnce(Subsystems.pivot::down))));
     */
    // speaker shooting sequence
    /*
     * OI.pilot.a()
     * .whileTrue(
     * new SequentialCommandGroup(
     * // At the same time:
     * // - Stop the intake
     * // - Start pivoting down
     * // - Start speeding up the shooter
     * new ParallelCommandGroup(
     * Subsystems.intake.runOnce(Subsystems.intake::stop),
     * Subsystems.pivot.runOnce(Subsystems.pivot::down),
     * Subsystems.shooter.runOnce(() -> Subsystems.shooter.setSpeed(5))),
     * 
     * // Wait until the pivot is completed and the shooter is at the desired speed
     * new WaitCommand(0.5),
     * 
     * // Shoot
     * Subsystems.intake.runOnce(() -> Subsystems.intake.set(1, true)),
     * 
     * // Wait until the note is no longer in the robot, and some more after that
     * // for completeness
     * new WaitUntilCommand(() -> !Subsystems.intake.noteIsPresent()),
     * new WaitCommand(0.5),
     * 
     * // Stop shooting
     * new ParallelCommandGroup(
     * Subsystems.intake.runOnce(Subsystems.intake::stop),
     * Subsystems.shooter.runOnce(Subsystems.shooter::stop))));
     */
    // Drive bindings handled in teleop command
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getTeleopCommand() {
    return teleopProvider.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;//utoutoProvider.getSelected();
  }
}
