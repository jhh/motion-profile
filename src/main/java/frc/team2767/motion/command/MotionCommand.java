package frc.team2767.motion.command;

import edu.wpi.first.wpilibj.command.Command;
import frc.team2767.motion.Robot;
import frc.team2767.motion.subsystem.DriveSubsystem;

public class MotionCommand extends Command {

  private static final DriveSubsystem drive = Robot.DRIVE;
  private final double direction;
  private final int distance;

  public MotionCommand(double direction, int distance) {
    this.direction = direction;
    this.distance = distance;
    requires(drive);
    setTimeout(5);
  }

  @Override
  protected void initialize() {
    drive.motionTo(direction, distance, 0d);
  }

  @Override
  protected boolean isFinished() {
    return drive.isMotionFinished() || isTimedOut();
  }

  @Override
  protected void end() {
    drive.endMotion();
  }
}
