package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.CurrentUnit
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Dimensionless
import edu.wpi.first.units.measure.MomentOfInertia

object ConveyorConstants {
    const val GEAR_RATIO = 1.0
    val TOLERANCE: Dimensionless = Units.Percent.of(0.1)
    private val CURRENT_LIMIT: Measure<CurrentUnit> = Units.Amps.of(40.0)
    val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.000_05)
    const val RUN_POWER: Double = 0.7

    val CONFIG = TalonFXConfiguration().apply {
        MotorOutput = MotorOutputConfigs().apply {
            InvertedValue.Clockwise_Positive
        }
        CurrentLimits = CurrentLimitsConfigs().apply {
            val currentLimit = CURRENT_LIMIT.`in`(Units.Amps)
            SupplyCurrentLimit = currentLimit
            StatorCurrentLimit = currentLimit * 2
            StatorCurrentLimitEnable = true
            SupplyCurrentLimitEnable = true
        }
    }
}