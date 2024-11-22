package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units

object ShooterConstants {
    var CURRENT_LIMIT: Double = 40.0
    var SHOOT_VELOCITY = MutableMeasure.ofBaseUnits(3.0, Units.RotationsPerSecond)
    var KP: Double = 0.0
    var KD: Double = 0.0
    var KI: Double = 0.0
    var KV: Double = 0.0
    var CONNFIG_TOP: TalonFXConfiguration = TalonFXConfiguration().apply {
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