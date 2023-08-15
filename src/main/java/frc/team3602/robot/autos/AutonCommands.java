package frc.team3602.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3602.robot.subsystems.Swerve;

public class AutonCommands {
  public static CommandBase driveOutCommunity(Swerve driveSubsys) {
    return Commands.run(() -> {
      driveSubsys.drive(new Translation2d(-1.0, 0.0), 0.0, false, false); // 0.75 7.5
    }, driveSubsys).withTimeout(6.0);
  }

  public static CommandBase driveToBalance(Swerve driveSubsys) {
    return Commands.run(() -> {
      driveSubsys.drive(new Translation2d(1.0, 0.0), 0.0, false, false); // 0.75 4.0
    }, driveSubsys).withTimeout(4.10).andThen(Commands.run(() -> {
      driveSubsys.drive(new Translation2d(0.0, 0.5), 0.0, false, false);
    }, driveSubsys).withTimeout(0.15));
  }
}
