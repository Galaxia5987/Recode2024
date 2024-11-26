package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units

object ShooterConstants {
    val CURRENT_LIMIT: Double = 40.0
    val SHOOT_VELOCITY = Units.RotationsPerSecond.of(3.0)
    val KP: Double = 0.0
    val KD: Double = 0.0
    val KI: Double = 0.0
    val KV: Double = 0.0

    val CONNFIG_TOP: TalonFXConfiguration = TalonFXConfiguration().apply {
        Slot0.apply {
            kP = KP
            kD = KD
            kI = KI
            kV = KV

        }
        CurrentLimits.apply {
            StatorCurrentLimitEnable = true
            SupplyCurrentLimit = CURRENT_LIMIT
            SupplyCurrentLimitEnable = true
            StatorCurrentLimit = CURRENT_LIMIT * 2
        }
        MotorOutput.apply {
            Inverted = InvertedValue.Clockwise_Positive
        }
    }


    var CONNFIG_BOTTOM: TalonFXConfiguration = TalonFXConfiguration().apply {
        Slot0.apply {
            kP = KP
            kD = KD
            kI = KI
            kV = KV

        }
        CurrentLimits.apply {
            StatorCurrentLimitEnable = true
            SupplyCurrentLimit = CURRENT_LIMIT
            SupplyCurrentLimitEnable = true
            StatorCurrentLimit = CURRENT_LIMIT * 2
        }
        MotorOutput.apply {
            Inverted = InvertedValue.Clockwise_Positive
        }
    }
}