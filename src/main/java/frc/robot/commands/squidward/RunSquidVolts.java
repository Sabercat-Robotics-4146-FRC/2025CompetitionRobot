package frc.robot.commands.squidward;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.squidward.Squidward;

public class RunSquidVolts extends Command {
  private Squidward squid;
  private double volts;

  public RunSquidVolts(Squidward squid, double volts) {
    this.squid = squid;
    this.volts = volts;
    addRequirements(squid);
  }

  @Override
  public void execute() {
    squid.runVolts(volts);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
