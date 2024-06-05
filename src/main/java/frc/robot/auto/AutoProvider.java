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

    chooser = new SendableChooser<>(); // pub for shuffle board
    //chooser = AutoBuilder.buildAutoChooser();
    chooser.setDefaultOption("Mobility (delay 10s)", new tmpAuto());
    chooser.addOption("Amp Blue", new SpeakerAuto());
    chooser.addOption("Amp Red", new AmpAutoRed());
    chooser.addOption("Speaker", new SpeakerAuto());
    chooser.addOption("tmp", new tmpAuto());
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