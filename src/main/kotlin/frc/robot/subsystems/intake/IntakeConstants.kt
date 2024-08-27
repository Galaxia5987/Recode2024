package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.Constants
import frc.robot.lib.webconstants.LoggedTunableNumber

object IntakeConstants {
    val MOTOR_CONFIGURATION = TalonFXConfiguration()
    val GEAR_RATIO = 55.56
    var ANGLE_KP = 0.0
    var ANGLE_KD = 0.0
    var ANGLE_KI = 0.0

    init{
        when(Constants.CURRENT_MODE){
            Constants.Mode.REAL ->{
                ANGLE_KP = 20.0
                ANGLE_KD = 0.0
                ANGLE_KI = 0.03

            }
            Constants.Mode.SIM , Constants.Mode.REPLAY->{
                ANGLE_KP = 10.1 / 360.0
                ANGLE_KD = 0.0
                ANGLE_KI = 0.0
            }
        }
        MOTOR_CONFIGURATION.withMotorOutput(
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
        ).withFeedback(
            FeedbackConfigs()
                .withRotorToSensorRatio(1.0)
                .withSensorToMechanismRatio(GEAR_RATIO)
        )
            .withSlot0(
                Slot0Configs()
                    .withKP(ANGLE_KP)
                    .withKI(ANGLE_KI)
                    .withKD(ANGLE_KD)
            ).CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80.0)
            .withSupplyCurrentLimit(40.0)
    }
}