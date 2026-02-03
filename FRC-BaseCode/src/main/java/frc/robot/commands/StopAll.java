package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class StopAll extends InstantCommand {
  private final List<Runnable> stopFunctions;

  public StopAll(List<Runnable> stopFunctions) {
    this.stopFunctions = stopFunctions;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void initialize() {
    for (Runnable r : stopFunctions) {
      r.run();
    }
  }
}