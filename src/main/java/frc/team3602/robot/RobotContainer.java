package frc.team3602.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.team3602.robot.autos.AutonCommands;
import frc.team3602.robot.commands.*;
import frc.team3602.robot.subsystems.*;

public class RobotContainer {
  /* Controllers */
  private final Joystick driver = new Joystick(0);

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
        () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis), () -> true));

    configureButtonBindings();
    configAuton();
  }

  private void configureButtonBindings() {
    // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    new JoystickButton(driver, XboxController.Button.kA.value)
        .whileTrue(armSubsys.moveWristAngle(() -> 15.0)).whileFalse(armSubsys.stopArmWrist());

    new JoystickButton(driver, XboxController.Button.kX.value)
        .whileTrue(armSubsys.moveArmAngle(() -> 6.0)).whileFalse(armSubsys.stopArmAngle());

    new JoystickButton(driver, XboxController.Button.kY.value)
        .whileTrue(armSubsys.moveArmExtend(() -> 35.0)).whileFalse(armSubsys.stopArmExtend());

    // Open Gripper
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
        .whileTrue(armSubsys.openGripper());

    // Close Gripper
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

    sendableChooser.addOption("Balance Command", AutonCommands.autoBalance(s_Swerve));
  }

  public Command getAutonomousCommand() {
    return sendableChooser.getSelected();
  }
}
