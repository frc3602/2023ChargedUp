package frc.team3602.lib.util;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.sensors.BasePigeon;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;

public class CANOptimization {
  public static void optimizeCTREMotors(BaseTalon baseTalon) {
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 43);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 61);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 71);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 79);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 89);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, 107);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, 113);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 131);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 139);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 163);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 173);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 181);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 193);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 223);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus, 229);
    baseTalon.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 251);
  }

  public static void optimizeCTREGyro(BasePigeon basePigeon) {
    basePigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 37);
    basePigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 43);
    basePigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, 53);
    basePigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 61);
    basePigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, 71);
    basePigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, 79);
    basePigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 89);
    basePigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 97);
    basePigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, 112);
    basePigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 118);
    basePigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, 124);
  }
}
