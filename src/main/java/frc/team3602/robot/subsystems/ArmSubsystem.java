package frc.team3602.robot.subsystems;

import frc.team3602.lib.math.MathBruh;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  /* Motor Controllers */
  private final CANSparkMax armMotor = new CANSparkMax(ArmConstants.armMotorCANID, MotorType.kBrushless);
  private final CANSparkMax armExtendMotor = new CANSparkMax(ArmConstants.armExtendCANID, MotorType.kBrushless);
  private final CANSparkMax armWristMotor = new CANSparkMax(ArmConstants.armWristCANID, MotorType.kBrushless);

  /* Encoders */
  private final RelativeEncoder armAngleEncoder = armMotor.getEncoder();
  private final RelativeEncoder armExtendEncoder = armExtendMotor.getEncoder();
  private final RelativeEncoder armWristEncoder = armWristMotor.getEncoder();

  /* Pneumatics */
  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid gripperSolenoid = new DoubleSolenoid(50, PneumaticsModuleType.CTREPCM, 0, 1);

  public final DigitalInput armAngleTopLimit = new DigitalInput(0);

  private final PIDController armAnglePIDController = new PIDController(ArmConstants.armAngleP, ArmConstants.armAngleI,
      ArmConstants.armAngleD);
  private final ArmFeedforward armAngleFeedforward = new ArmFeedforward(ArmConstants.armKS,
      ArmConstants.armKG, ArmConstants.armKV, ArmConstants.armKA);
  private final PIDController armExtendPIDController = new PIDController(ArmConstants.armExtendP,
      ArmConstants.armExtendI, ArmConstants.armExtendD);
  private final PIDController armWristPIDController = new PIDController(0.60, 0.0, 0.10);
  private final ArmFeedforward armWristFeedforward = new ArmFeedforward(2.50, 0.18, 3.91, 0.01); // 0.68, 5.37, 2.35

  public ArmSubsystem() {
    resetArmAngleEncoder();
    resetArmExtendEncoder();
    resetArmWristEncoder();

    configArmSubsys();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Arm Angle Encoder", getArmAngleEncoder());
    // SmartDashboard.putNumber("Arm Wrist Encoder", getArmWristEncoder());
    // SmartDashboard.putNumber("Arm Extend Encoder", getArmExtendEncoder());
    // SmartDashboard.putBoolean("Arm Limit Switch", armAngleTopLimit.get());
  }

  public double getArmAngleEncoder() {
    return armAngleEncoder.getPosition();
  }

  public double getArmExtendEncoder() {
    return armExtendEncoder.getPosition();
  }

  public double getArmWristEncoder() {
    return armWristEncoder.getPosition();
  }

  public void resetArmAngleEncoder() {
    armAngleEncoder.setPosition(0.0);
  }

  public void resetArmExtendEncoder() {
    armExtendEncoder.setPosition(0.0);
  }

  public void resetArmWristEncoder() {
    armWristEncoder.setPosition(0.0);
  }

  public boolean checkAllArm(ArmSubsystem armSubsys, double armAngle, double armExtendInches,
      double armWristAngle) {
    if (MathBruh.between(armSubsys.getArmAngleEncoder(), armAngle - 5.0, armAngle + 5.0)
        && MathBruh.between(armSubsys.getArmExtendEncoder(), armExtendInches - 5.0, armExtendInches + 5.0)
        && MathBruh.between(armSubsys.getArmWristEncoder(), armWristAngle - 3.0, armWristAngle + 10.0)) {
      return true;
    } else {
      return false;
    }
  }

  public void moveArmAngle(DoubleSupplier angleSup) {
    if (angleSup.getAsDouble() > 0.0) {
      if (!armAngleTopLimit.get() || MathBruh.between(getArmAngleEncoder(), 5.0, 1.0)) {
        armMotor.set(0.0);
      } else {
        armMotor
            .setVoltage(armAnglePIDController.calculate(getArmAngleEncoder(), angleSup.getAsDouble())
                + armAngleFeedforward.calculate(Math.toRadians(angleSup.getAsDouble()), 0.0));
      }
    } else {
      if (angleSup.getAsDouble() < -63) {
        armMotor.set(0.0);
      } else {
        armMotor
            .setVoltage(armAnglePIDController.calculate(getArmAngleEncoder(), angleSup.getAsDouble())
                + armAngleFeedforward.calculate(Math.toRadians(angleSup.getAsDouble()), 0.0));
      }
    }
  }

  public void moveArmExtend(DoubleSupplier inchSup) {
    armExtendMotor
        .set(armExtendPIDController.calculate(getArmExtendEncoder(), inchSup.getAsDouble()));
  }

  public void moveWristAngle(DoubleSupplier angleSup) {
    armWristMotor
        .setVoltage(armWristPIDController.calculate(getArmWristEncoder(), angleSup.getAsDouble())
            + armWristFeedforward.calculate(Math.toRadians(angleSup.getAsDouble()), 0.0));
  }

  public void moveArm(ArmSubsystem armSubsys, DoubleSupplier armAngleSup, DoubleSupplier armExtendSup,
      DoubleSupplier armWristAngleSup) {
    armSubsys.moveWristAngle(armWristAngleSup);
    armSubsys.moveArmAngle(armAngleSup);
    armSubsys.moveArmExtend(armExtendSup);
  }

  public CommandBase moveInFrame(ArmSubsystem armSubsys) {
    var armAngle = -63.0;
    var extendInches = 0.0;
    var wristAngle = 115.0;
    return run(() -> armSubsys.moveArm(armSubsys, () -> armAngle, () -> extendInches, () -> wristAngle));
  }

  public CommandBase moveToLow(ArmSubsystem armSubsys) {
    var armAngle = -63.0;
    var extendInches = 0.0;
    var wristAngle = 115.0;
    return run(() -> armSubsys.moveArm(armSubsys, () -> armAngle, () -> extendInches, () -> wristAngle))
        .until(() -> armSubsys.checkAllArm(armSubsys, armAngle, extendInches, wristAngle)).andThen(armSubsys.stopArm());
  }

  public CommandBase moveToMid(ArmSubsystem armSubsys) {
    var armAngle = -23.0;
    var extendInches = 10.0;
    var wristAngle = 115.0;
    return run(() -> armSubsys.moveArm(armSubsys, () -> armAngle, () -> extendInches, () -> wristAngle))
        .until(() -> armSubsys.checkAllArm(armSubsys, armAngle, extendInches, wristAngle)).andThen(armSubsys.stopArm());
  }

  public CommandBase moveToHigh(ArmSubsystem armSubsys) {
    var armAngle = -15.0;
    var extendInches = 28.0;
    var wristAngle = 115.0;
    return run(() -> armSubsys.moveArm(armSubsys, () -> armAngle, () -> extendInches, () -> wristAngle))
        .until(() -> armSubsys.checkAllArm(armSubsys, armAngle, extendInches, wristAngle)).andThen(armSubsys.stopArm());
  }

  public CommandBase moveToMidAuton(ArmSubsystem armSubsys) {
    return new SequentialCommandGroup(
        armSubsys.moveToMid(armSubsys),
        openGripper().until(() -> gripperSolenoid.get() == Value.kForward));
  }

  public CommandBase moveToHighAuton(ArmSubsystem armSubsys) {
    return new SequentialCommandGroup(
        armSubsys.moveToHigh(armSubsys),
        openGripper().until(() -> gripperSolenoid.get() == Value.kForward));
  }

  public CommandBase stopArm() {
    return runOnce(() -> {
      armMotor.set(0.0);
      armExtendMotor.set(0.0);
      armWristMotor.set(0.0);
    });
  }

  public CommandBase stopArmAngle() {
    return runOnce(() -> armMotor.set(0.0));
  }

  public CommandBase stopArmExtend() {
    return runOnce(() -> armExtendMotor.set(0.0));
  }

  public CommandBase stopArmWrist() {
    return runOnce(() -> armWristMotor.set(0.0));
  }

  public CommandBase openGripper() {
    return runOnce(() -> gripperSolenoid.set(Value.kForward));
  }

  public CommandBase closeGripper() {
    return runOnce(() -> gripperSolenoid.set(Value.kReverse));
  }

  private void configArmSubsys() {
    armMotor.setIdleMode(IdleMode.kBrake);
    armExtendMotor.setIdleMode(IdleMode.kBrake);
    armWristMotor.setIdleMode(IdleMode.kBrake);

    armMotor.setSmartCurrentLimit(30);
    armExtendMotor.setSmartCurrentLimit(30);
    armWristMotor.setSmartCurrentLimit(5);

    armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    armMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 1);

    armMotor.setInverted(true);

    compressor.enableDigital();
    gripperSolenoid.set(Value.kOff);

    armAngleEncoder.setPositionConversionFactor(360.0 / ArmConstants.armAngleGearRatio);
    armExtendEncoder.setPositionConversionFactor((Math.PI * 2.0) / ArmConstants.armExtendGearRatio);
    armWristEncoder.setPositionConversionFactor(360.0 / ArmConstants.armWristGearRatio);
  }
}
