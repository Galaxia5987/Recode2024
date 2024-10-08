package frc.robot.subsystems.conveyor

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.Current
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object ConveyorConstants {
  const val TOLERANCE = 0.5
  private val CURRENT_LIMIT: Measure<Current> = Units.Amps.of(40.0)
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