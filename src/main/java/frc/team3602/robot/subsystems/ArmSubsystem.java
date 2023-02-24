package frc.team3602.robot.subsystems;

import java.util.function.DoubleSupplier;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3602.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final CANSparkMax armMotor = new CANSparkMax(ArmConstants.armMotorCANID, MotorType.kBrushless);
  private final CANSparkMax armExtendMotor = new CANSparkMax(ArmConstants.armExtendCANID, MotorType.kBrushless);
  private final CANSparkMax gripperMotor = new CANSparkMax(ArmConstants.gripperCANID, MotorType.kBrushless);

  private final RelativeEncoder armAngleEncoder = armMotor.getEncoder();
  private final RelativeEncoder armExtendEncoder = armExtendMotor.getEncoder();
  private final RelativeEncoder gripperEncoder = gripperMotor.getEncoder();

  private final PIDController armPIDController = new PIDController(ArmConstants.armP, ArmConstants.armI,
      ArmConstants.armD);
  private final ArmFeedforward armFeedforward = new ArmFeedforward(ArmConstants.armKS,
      ArmConstants.armKG, ArmConstants.armKV, ArmConstants.armKA);

  public ArmSubsystem() {
    configArmSubsys();
  }

  @Override
  public void periodic() {

  }

  public double getArmAngle() {
    return armAngleEncoder.getPosition() - ArmConstants.armZeroAngle;
  }

  // public CommandBase moveArmFF(DoubleSupplier velocity, DoubleSupplier
  // setpoint) {
  // return run(() -> {
  // armMotor.setVoltage(armFeedforward.calculate(getArmAngle(),
  // velocity.getAsDouble())
  // + armPIDController.calculate(armAngleEncoder.getPosition(),
  // setpoint.getAsDouble()));
  // });
  // }

  public CommandBase moveArmAngle(DoubleSupplier speed) {
    return run(() -> {
      armMotor.set(speed.getAsDouble());
    });
  }

  public CommandBase moveArmExtend(DoubleSupplier speed) {
    return run(() -> {
      armExtendMotor.set(speed.getAsDouble());
    });
  }

  public CommandBase moveGripper(DoubleSupplier speed) {
    return run(() -> {
      gripperMotor.set(speed.getAsDouble());
    });
  }

  public CommandBase stopArmAngle() {
    return run(() -> {
      armMotor.set(0.0);
    });
  }

  public CommandBase stopArmExtend() {
    return run(() -> {
      armExtendMotor.set(0.0);
    });
  }

  public CommandBase stopGripper() {
    return run(() -> {
      gripperMotor.set(0.0);
    });
  }

  private void configArmSubsys() {
    armMotor.setIdleMode(IdleMode.kBrake);
    armExtendMotor.setIdleMode(IdleMode.kBrake);
    gripperMotor.setIdleMode(IdleMode.kBrake);

    armAngleEncoder.setPositionConversionFactor(360.0 / ArmConstants.armGearRatio);
  }
}
