package frc.team3602.robot.subsystems;

import frc.team3602.lib.math.MathBruh;
import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax armMotor =
      new CANSparkMax(ArmConstants.armMotorCANID, MotorType.kBrushless);
  private final CANSparkMax armExtendMotor =
      new CANSparkMax(ArmConstants.armExtendCANID, MotorType.kBrushless);
  private final CANSparkMax armWristMotor =
      new CANSparkMax(ArmConstants.armWristCANID, MotorType.kBrushless);

  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid gripperSolenoid =
      new DoubleSolenoid(50, PneumaticsModuleType.CTREPCM, 0, 1);

  private final RelativeEncoder armAngleEncoder = armMotor.getEncoder();
  private final RelativeEncoder armExtendEncoder = armExtendMotor.getEncoder();
  private final RelativeEncoder armWristEncoder = armWristMotor.getEncoder();

  public final DigitalInput armAngleTopLimit = new DigitalInput(0);

  private final PIDController armAnglePIDController =
      new PIDController(ArmConstants.armAngleP, ArmConstants.armAngleI, ArmConstants.armAngleD);
  private final ArmFeedforward armAngleFeedforward = new ArmFeedforward(ArmConstants.armKS,
      ArmConstants.armKG, ArmConstants.armKV, ArmConstants.armKA);
  private final PIDController armExtendPIDController =
      new PIDController(ArmConstants.armExtendP, ArmConstants.armExtendI, ArmConstants.armExtendD);
  private final ArmFeedforward armWristFeedforward = new ArmFeedforward(6.50, 0.48, 1.22, 0.02);

  public ArmSubsystem() {
    resetArmAngleEncoder();
    resetArmExtendEncoder();
    resetArmWristEncoder();

    configArmSubsys();
  }

  @Override
  public void periodic() {
    // if (!armAngleTopLimit.get()) {
    // resetArmAngleEncoder();
    // }

    SmartDashboard.putNumber("Arm Angle Encoder", getArmAngleEncoder());
    SmartDashboard.putNumber("Arm Wrist Encoder", getArmWristEncoder());
    SmartDashboard.putNumber("Arm Extend Encoder", getArmExtendEncoder());
    SmartDashboard.putBoolean("Stuff", MathBruh.between((int)getArmAngleEncoder(), -21, -30));
    // SmartDashboard.putBoolean("Arm Angle Limit", armAngleTopLimit.get());
    // SmartDashboard.putData(CommandScheduler.getInstance());
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

  public CommandBase checkArmAngleLimit() {
    return run(() -> {
      if (!armAngleTopLimit.get()) {
        resetArmAngleEncoder();
      }
    });
  }

  // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/limit-switch.html
  public CommandBase moveArmAngle(DoubleSupplier angle) {
    return run(() -> {
      if (angle.getAsDouble() > 0.0) {
        if (!armAngleTopLimit.get()) {
          armMotor.set(0.0);
        } else {
          armMotor
              .setVoltage(armAnglePIDController.calculate(getArmAngleEncoder(), angle.getAsDouble())
                  + armAngleFeedforward.calculate(Math.toRadians(angle.getAsDouble()), 0.0));
        }
      } else {
        armMotor
            .setVoltage(armAnglePIDController.calculate(getArmAngleEncoder(), angle.getAsDouble())
                + armAngleFeedforward.calculate(Math.toRadians(angle.getAsDouble()), 0.0));
      }
    });
  }

  public CommandBase moveArmExtend(DoubleSupplier inches) {
    return run(() -> {
      armExtendMotor
          .set(armExtendPIDController.calculate(getArmExtendEncoder(), inches.getAsDouble()));
    });
  }

  public CommandBase moveWristAngle(DoubleSupplier angle) {
    return run(() -> {
      armWristMotor
          .setVoltage(armWristFeedforward.calculate(Math.toRadians(angle.getAsDouble() + 3.0), 0.0));
    });
  }

  public CommandBase moveToMid(ArmSubsystem armSubsys) {
    return new SequentialCommandGroup(
      armSubsys.moveArmAngle(() -> -25.0).until(() -> MathBruh.between(armSubsys.getArmAngleEncoder(), -21.0, -30.0)),
      armSubsys.moveArmExtend(() -> 25.0).until(() -> MathBruh.between(armSubsys.getArmExtendEncoder(), 21.0, 30.0))
    );
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
    return runOnce(() -> gripperSolenoid.set(Value.kReverse));
  }

  public CommandBase closeGripper() {
    return runOnce(() -> gripperSolenoid.set(Value.kForward));
  }

  private void configArmSubsys() {
    armMotor.setIdleMode(IdleMode.kBrake);
    armExtendMotor.setIdleMode(IdleMode.kBrake);
    armWristMotor.setIdleMode(IdleMode.kBrake);

    armMotor.setInverted(true);

    compressor.enableDigital();

    gripperSolenoid.set(Value.kReverse);

    armAngleEncoder.setPositionConversionFactor(360.0 / ArmConstants.armAngleGearRatio);
    armExtendEncoder.setPositionConversionFactor((Math.PI * 2.0) / ArmConstants.armExtendGearRatio);
    armWristEncoder.setPositionConversionFactor(360.0 / ArmConstants.armWristGearRatio);
  }
}
