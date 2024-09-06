package frc.robot.subsystems.hood

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.Constants
import frc.robot.subsystems.intake.IntakeConstants
import frc.robot.subsystems.intake.IntakeConstants.GEAR_RATIO

object HoodConstants {
    var MOTOR_CONFIGURATION = TalonFXConfiguration()
    var restAngle = 35.0
    const val GEAR_RATIO: Double = 3.0 * (36.0 / 18.0) * (158.0 / 18.0)
    var ANGLE_KP = 0.0
    var ANGLE_KD = 0.0
    var ANGLE_KI = 0.0
    const val TOLERANCE: Double = 0.75 / 360

    init {
        when(Constants.CURRENT_MODE){
            Constants.Mode.REAL->{

            }
            Constants.Mode.SIM , Constants.Mode.REPLAY->{

            }
        }
        MOTOR_CONFIGURATION.withMotorOutput(MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive)
        ).withFeedback(
            FeedbackConfigs()
                .withRotorToSensorRatio(1.0)
                .withSensorToMechanismRatio(GEAR_RATIO)
        ).withSlot0(
            Slot0Configs()
                .withKP(IntakeConstants.ANGLE_KP)
                .withKI(IntakeConstants.ANGLE_KI)
                .withKD(IntakeConstants.ANGLE_KD)
        ).CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80.0)
            .withSupplyCurrentLimit(40.0)
    }
}