package frc.team3602.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3602.lib.math.MathBruh;
import frc.team3602.robot.subsystems.Swerve;

public class AutonCommands {
  public static CommandBase driveOutCommunity(Swerve driveSubsys) {
    return Commands.run(() -> {
      driveSubsys.drive(new Translation2d(-0.75, 0.0), 0.0, false, false);
    }, driveSubsys).withTimeout(6.5);
  }

  public static CommandBase driveToBalance(Swerve driveSubsys) {
    return Commands.run(() -> {
      driveSubsys.drive(new Translation2d(0.75, 0.0), 0.0, false, false);
    }, driveSubsys).until(() -> MathBruh.between(driveSubsys.getPitch().getDegrees(), 0, 10));
  }

  public static CommandBase driveBalance(Swerve driveSubsys) {
    PIDController pidController = new PIDController(0.3, 0, 0.0);
    return Commands.run(() -> {
      driveSubsys.drive(
          new Translation2d(pidController.calculate(driveSubsys.getPitch().getDegrees() * 0.01),
              0.0),
          0.0, false, false);
    }, driveSubsys);
  }
}
