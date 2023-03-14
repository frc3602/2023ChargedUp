package frc.team3602.robot.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.team3602.robot.subsystems.Swerve;

public class AutonCommands {
  public static CommandBase driveOutCommunity(Swerve driveSubsys) {
    return Commands.run(() -> {
      driveSubsys.drive(new Translation2d(-0.75, 0.0), 0.0, false, false);
    }, driveSubsys).withTimeout(6.5);
  }
}
