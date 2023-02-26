package frc.team3602.robot.subsystems;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax armMotor =
      new CANSparkMax(ArmConstants.armMotorCANID, MotorType.kBrushless);
  private final CANSparkMax armExtendMotor =
      new CANSparkMax(ArmConstants.armExtendCANID, MotorType.kBrushless);

  private final Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private final DoubleSolenoid gripperSolenoid =
      new DoubleSolenoid(50, PneumaticsModuleType.CTREPCM, 0, 1);

  private final RelativeEncoder armAngleEncoder = armMotor.getEncoder();
  private final RelativeEncoder armExtendEncoder = armExtendMotor.getEncoder();

  public final DigitalInput armAngleTopLimit = new DigitalInput(0);

  private final PIDController armAnglePIDController =
      new PIDController(ArmConstants.armAngleP, ArmConstants.armAngleI, ArmConstants.armAngleD);
  private final ArmFeedforward armAngleFeedforward = new ArmFeedforward(ArmConstants.armKS,
      ArmConstants.armKG, ArmConstants.armKV, ArmConstants.armKA);
  private final PIDController armExtendPIDController =
      new PIDController(ArmConstants.armExtendP, ArmConstants.armExtendI, ArmConstants.armExtendD);

  public ArmSubsystem() {
    configArmSubsys();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Arm Angle Top", armAngleTopLimit.get());
  }

  public double getArmAngle() {
    return armAngleEncoder.getPosition() - ArmConstants.armZeroAngle;
  }

  public double getArmExtend() {
    return armExtendEncoder.getPosition();
  }

  // public CommandBase moveArmAngle(DoubleSupplier angle) {
  // return run(() -> armMotor
  // .setVoltage(armAnglePIDController.calculate(getArmAngle(), angle.getAsDouble())
  // + armAngleFeedforward.calculate(Math.toRadians(angle.getAsDouble()), 0.0)));
  // }

  public CommandBase moveArmAngle(DoubleSupplier angle) {
    return run(() -> {
      if (angle.getAsDouble() > 0) {
        if (!armAngleTopLimit.get()) {
          armMotor.set(0.0);
        } else {
          armMotor.setVoltage(armAnglePIDController.calculate(getArmAngle(), angle.getAsDouble())
              + armAngleFeedforward.calculate(Math.toRadians(angle.getAsDouble()), 0.0));
        }
      } else {
        armMotor.setVoltage(armAnglePIDController.calculate(getArmAngle(), angle.getAsDouble())
            + armAngleFeedforward.calculate(Math.toRadians(angle.getAsDouble()), 0.0));
      }
    });
  }

  public CommandBase moveArmExtendPID(DoubleSupplier inches) {
    return run(() -> armExtendMotor
        .set(armExtendPIDController.calculate(getArmExtend(), inches.getAsDouble())));
  }

  public CommandBase moveArmExtend(DoubleSupplier speed) {
    return run(() -> armExtendMotor.set(speed.getAsDouble()));
  }

  public CommandBase stopArmAngle() {
    return run(() -> armMotor.set(0.0));
  }

  public CommandBase stopArmExtend() {
    return run(() -> armExtendMotor.set(0.0));
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

    armMotor.setInverted(true);

    compressor.enableDigital();
    gripperSolenoid.set(Value.kForward);

    armAngleEncoder.setPosition(0.0);
    armExtendEncoder.setPosition(0.0);

    armAngleEncoder.setPositionConversionFactor(360.0 / ArmConstants.armAngleGearRatio);
    armExtendEncoder.setPositionConversionFactor((Math.PI * 2.0) / ArmConstants.armExtendGearRatio);
  }
}
