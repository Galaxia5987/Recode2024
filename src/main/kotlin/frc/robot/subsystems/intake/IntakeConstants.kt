package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object IntakeConstants {
    const val INTAKE_SPIN_POWER = 0.4
    const val INTAKE_CENTER_POWER = 0.4
    const val GEAR_RATIO = 45.62
    val MOTOR_CONFIG = TalonFXConfiguration()

    val INTAKE_ANGLE: Measure<Angle> = Units.Degree.zero()
    val REST_ANGLE: Measure<Angle> = Units.Degree.of(114.0)

    init {
        MOTOR_CONFIG.withMotorOutput(
            MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive)
        ).CurrentLimits
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimit(80.0)
            .withSupplyCurrentLimit(40.0)
    }
}
