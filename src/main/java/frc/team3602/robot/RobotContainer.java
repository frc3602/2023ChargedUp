package frc.team3602.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

  SendableChooser<Double> controllerPolarity = new SendableChooser<>();
  private final JoystickButton robotOriented =
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private double slowDown = 1.0;

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final ArmSubsystem armSubsys = new ArmSubsystem();

  /* Autonomous */
  SendableChooser<Command> sendableChooser = new SendableChooser<>();

  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(s_Swerve,
        () -> controllerPolarity.getSelected() * driver.getRawAxis(translationAxis) * slowDown,
        () -> controllerPolarity.getSelected() * driver.getRawAxis(strafeAxis) * slowDown,
        () -> -driver.getRawAxis(rotationAxis) * slowDown,
        () -> robotOriented.getAsBoolean())); // false = field
                                              // true = robot
    configureButtonBindings();
    configAuton();

    SmartDashboard.putData(controllerPolarity);
    controllerPolarity.addOption("Positive", 1.0);
    controllerPolarity.addOption("Negative", -1.0);
  }

  private void configureButtonBindings() {
    // Slow down button
    new JoystickButton(driver, XboxController.Button.kRightBumper.value)
        .whileTrue(new InstantCommand(() -> slowDown = 0.5))
        .whileFalse(new InstantCommand(() -> slowDown = 1.0));

    // Move down a bit
    armJoystick.povUp()
        .whileTrue(armSubsys.run(() -> armSubsys.moveArmAngle(() -> -25))
            .until(() -> armSubsys.checkArmAngle(armSubsys, -25)).andThen(armSubsys.stopArmAngle()))
        .whileFalse(armSubsys.stopArmAngle());

    // Move in frame
    armJoystick.b().whileTrue(armSubsys.moveInFrame(armSubsys)).whileFalse(armSubsys.stopArm());

    // Move to double substation
    armJoystick.a().whileTrue(armSubsys.moveToSub(armSubsys)).whileFalse(armSubsys.stopArm());

    // Move to mid
    armJoystick.x().whileTrue(armSubsys.moveToMid(armSubsys)).whileFalse(armSubsys.stopArm());

    // Move to high
    armJoystick.y().whileTrue(armSubsys.moveToHigh(armSubsys)).whileFalse(armSubsys.stopArm());

    // Open gripper
    armJoystick.leftBumper().whileTrue(armSubsys.openGripper());

    // Close gripper
    armJoystick.rightBumper().whileTrue(armSubsys.closeGripper());
  }

  private void configAuton() {
    SmartDashboard.putData(sendableChooser);

    sendableChooser.addOption("Single Piece Mid", armSubsys.moveToMidAuton(armSubsys));
    sendableChooser.addOption("Single Piece High", armSubsys.moveToHighAuton(armSubsys));
    sendableChooser.addOption("Single Mid Drive", armSubsys.moveToMidDriveAuton(armSubsys, s_Swerve));
    sendableChooser.addOption("Single High Drive", armSubsys.moveToHighDriveAuton(armSubsys, s_Swerve));
    sendableChooser.addOption("Single Mid Drive Balance", armSubsys.moveToMidBalanceAuton(armSubsys, s_Swerve));
    sendableChooser.addOption("Single High Drive Balance", armSubsys.moveToHighBalanceAuton(armSubsys, s_Swerve));
    
  }

  public Command getAutonomousCommand() {
    return sendableChooser.getSelected();
  }
}
