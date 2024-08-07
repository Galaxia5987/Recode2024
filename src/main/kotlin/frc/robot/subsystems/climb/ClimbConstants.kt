package frc.robot.subsystems.climb

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

object ClimbConstants {
    const val GEAR_RATIO = 12.0
    const val STOPPER_MOTOR_CURRENT_LIMIT = 20
    const val STOPPER_MOTOR_CURRENT_THRESHOLD = 10 //The current threshold for the stopper to stop at(when it hits something)
    const val STOPPER_MOTOR_VOLTAGE_COMPENSATION_SATURATION = 12.0
    const val STOPPER_MOTOR_POWER = 0.5
    val MOTOR_CONFIG = TalonFXConfiguration()

    init {
        MOTOR_CONFIG.withMotorOutput(
            MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)
        ).CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(40.0)
            .withSupplyCurrentLimit(40.0)
    }
}