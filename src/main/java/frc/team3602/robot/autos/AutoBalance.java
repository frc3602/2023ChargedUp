package frc.team3602.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.team3602.robot.subsystems.Swerve;

public class AutoBalance extends PIDCommand {
  private final static double minOutput = -0.075;
  private final static double maxOutput = +0.075;
  private final static double tolerance = 2.0;

  public AutoBalance(Swerve swerveSubsys) {
    super(
        new PIDController(0, 0, 0),
        () -> swerveSubsys.getRoll().getDegrees(),
        () -> 0.0,
        output -> {
          swerveSubsys.drive(
              new Translation2d(Math.abs(output) <= tolerance ? 0.0 : (output < 0.0 ? minOutput : maxOutput), 0.0),
              output, false, false);
        });
    addRequirements(swerveSubsys);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
