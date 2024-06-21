package frc.robot.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems;

/**
 * Provides the default command for autonomous.
 */
public class AutoProvider {
  private static Optional<AutoProvider> inst = Optional.empty();

  private final SendableChooser<Command> chooser;

  private AutoProvider() {

    chooser = new SendableChooser<>(); // pub for shuffle board
    //chooser = AutoBuilder.buildAutoChooser();
    chooser.setDefaultOption("disabled", new InstantCommand(() -> {}, Subsystems.drive));
    chooser.addOption("Mobility", new ModilityAuto());
    chooser.addOption("Mobility (delay 10s)", new ModilityAutoDelayed());
    chooser.addOption("Amp Blue", new AmpAutoBlue());
    chooser.addOption("Amp Red", new AmpAutoRed());
    chooser.addOption("Speaker", new SpeakerAuto());
    chooser.addOption("test", new TestAuto());
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