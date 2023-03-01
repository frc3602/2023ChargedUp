package frc.team3602.robot;

import frc.team3602.lib.math.MathBruh;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team3602.robot.autos.AutonCommands;
import frc.team3602.robot.commands.*;
import frc.team3602.robot.subsystems.*;

public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final CommandXboxController armJoystick = new CommandXboxController(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // private final JoystickButton zeroGyro =
  // new JoystickButton(driver, XboxController.Button.kY.value);
  // private final JoystickButton robotCentric =
  // new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final ArmSubsystem armSubsys = new ArmSubsystem();

  /* Autonomous */
  SendableChooser<Command> sendableChooser = new SendableChooser<>();

  public RobotContainer() {
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis), () -> false));

    configureButtonBindings();
    configAuton();
  }

  private void configureButtonBindings() {
    // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    // armJoystick.x()
    // .onTrue(armSubsys.moveArmAngle(() -> 25.0)
    // .raceWith(armSubsys.moveArmExtend(() -> 25.0), armSubsys.moveWristAngle(() -> 45))
    // .until(() -> Math.between(armSubsys.getArmAngleEncoder(), 23, 27)));

    armJoystick.y().whileTrue(armSubsys.moveArmAngle(() -> 8.0))
        .whileFalse(armSubsys.stopArmAngle());
    armJoystick.x().whileTrue(armSubsys.moveToMid(armSubsys)).whileFalse(armSubsys.stopArm());

    armJoystick.a().whileTrue(armSubsys.moveWristAngle(() -> 10.0))
        .whileFalse(armSubsys.stopArmWrist());

    // armJoystick.x().whileTrue(armSubsys.moveArmAngle(() -> -25.0))
    // .whileFalse(armSubsys.stopArmAngle());
    // armJoystick.x().whileTrue(armSubsys.moveArmExtend(() -> 25.0))
    // .whileFalse(armSubsys.stopArmExtend());

    // new JoystickButton(driver, XboxController.Button.kX.value)
    // .whileTrue(armSubsys.moveArmAngle(() -> 4.0)).whileFalse(armSubsys.stopArmAngle());

    // new JoystickButton(driver, XboxController.Button.kY.value)
    // .whileTrue(armSubsys.moveArmAngle(() -> -35.0)).whileFalse(armSubsys.stopArmAngle());

    // Close Gripper
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
        .whileTrue(armSubsys.openGripper());

    // Open Gripper
    new JoystickButton(driver, XboxController.Button.kRightBumper.value)
        .whileTrue(armSubsys.closeGripper());

    // // Arm Up
    // new JoystickButton(driver, XboxController.Button.kX.value)
    // .whileTrue(armSubsys.moveArmAngle(() -> 0.50)).whileFalse(armSubsys.stopArmAngle());

    // // Arm Down
    // new JoystickButton(driver, XboxController.Button.kY.value)
    // .whileTrue(armSubsys.moveArmAngle(() -> -0.50)).whileFalse(armSubsys.stopArmAngle());

    // // Arm Extend
    // new JoystickButton(driver, XboxController.Button.kA.value)
    // .whileTrue(armSubsys.moveArmExtend(() -> 0.25)).whileFalse(armSubsys.stopArmExtend());

    // // Arm Extend
    // new JoystickButton(driver, XboxController.Button.kB.value)
    // .whileTrue(armSubsys.moveArmExtend(() -> -0.25)).whileFalse(armSubsys.stopArmExtend());
  }

  private void configAuton() {
    SmartDashboard.putData(sendableChooser);

    sendableChooser.addOption("Single Game Piece Command",
        AutonCommands.singleGamePiece(s_Swerve, armSubsys));
  }

  public Command getAutonomousCommand() {
    return sendableChooser.getSelected();
  }
}
