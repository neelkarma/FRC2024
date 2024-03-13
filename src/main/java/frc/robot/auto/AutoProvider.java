package frc.robot.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Provides the default command for autonomous.
 */
public class AutoProvider {
  private static Optional<AutoProvider> inst = Optional.empty();

  private final SendableChooser<Command> chooser;

  private AutoProvider() {
    chooser = AutoBuilder.buildAutoChooser();
    chooser.addOption("Mobility", new NaiveAuto());
    SmartDashboard.putData("Auto Chooser", chooser);
  }

  public static AutoProvider getInstance() {
    if (!inst.isPresent()) {
      inst = Optional.of(new AutoProvider());
    }
    return inst.get();
  }

  public Command getSelected() {
    return chooser.getSelected();
  }
}