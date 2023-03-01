package frc.team3602.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.team3602.robot.subsystems.ArmSubsystem;
import frc.team3602.robot.subsystems.Swerve;

public class AutonCommands {
  public AutonCommands() {}

  public static CommandBase singleGamePiece(Swerve swerveSubsys, ArmSubsystem armSubsys) {
    return new SequentialCommandGroup(
      armSubsys.moveArmAngle(() -> 15.0).until(() -> !armSubsys.armAngleTopLimit.get()),
      armSubsys.checkArmAngleLimit().until(() -> !armSubsys.armAngleTopLimit.get()),
      armSubsys.moveArmAngle(() -> -25.0), // TODO: Set angle later
      armSubsys.moveArmExtend(() -> 15.0), // TODO: Set inches later
      armSubsys.openGripper()
    );
  }

  public static CommandBase autoBalance(Swerve swerveSubsys) {
    PIDController autoBalanceController = new PIDController(0.5, 0, 0);

    var minOutput = -0.075;
    var maxOutput = +0.075;
    var tolerance = 2.0;

    autoBalanceController.setTolerance(tolerance);

    return new PIDCommand(
        autoBalanceController,
        () -> swerveSubsys.getRoll().getDegrees(),
        () -> 0.0,
        output -> {
          swerveSubsys.drive(
              new Translation2d(Math.abs(output) <= tolerance ? 0.0 : (output < 0.0 ? minOutput : maxOutput), 0.0),
              output, false, false);
        },
        swerveSubsys);
  }
}
