package frc.team3602.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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

  private boolean robotCentric = true;

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
        () -> -driver.getRawAxis(strafeAxis), () -> -driver.getRawAxis(rotationAxis), () -> false)); // false = field
                                                                                                     // true = robot
    configureButtonBindings();
    configAuton();
  }

  private void configureButtonBindings() {
    // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    // Move to floor
    armJoystick.a().whileTrue(armSubsys.moveToLow(armSubsys)).whileFalse(armSubsys.stopArm());

    // Move to mid
    armJoystick.x().whileTrue(armSubsys.moveToMid(armSubsys)).whileFalse(armSubsys.stopArm());

    // Move to high
    armJoystick.y().whileTrue(armSubsys.moveToHigh(armSubsys)).whileFalse(armSubsys.stopArm());

    // Open gripper
    armJoystick.leftBumper().whileTrue(armSubsys.openGripper());

    // Close gripper
    armJoystick.rightBumper().whileTrue(armSubsys.closeGripper());

    // Raise Intake
    new JoystickButton(driver, XboxController.Button.kLeftBumper.value).whileTrue(armSubsys.raiseIntake());

    // Lower Intake
    new JoystickButton(driver, XboxController.Button.kRightBumper.value).whileTrue(armSubsys.lowerInake());
  }

  private void configAuton() {
    SmartDashboard.putData(sendableChooser);

    // sendableChooser.addOption("Single Piece Mid",
    armSubsys.moveToMidAuton(armSubsys);
  }

  public Command getAutonomousCommand() {
    // return sendableChooser.getSelected();
    return armSubsys.moveToMidAuton(armSubsys);
  }
}
