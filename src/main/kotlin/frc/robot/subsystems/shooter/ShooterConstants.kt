package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Dimensionless
import edu.wpi.first.units.measure.MomentOfInertia

object ShooterConstants {
    val CURRENT_LIMIT: Double = 40.0
    val SHOOT_VELOCITY = Units.RotationsPerSecond.of(3.0)
    const val GEAR_RATIO_TOP: Double = 1.0
    const val GEAR_RATIO_BOTTOM: Double = 1.0

//    val TOP_ROLLER_TOLERANCE: Dimensionless = Units.Percent.of(0.03)
//    val BOTTOM_ROLLER_TOLERANCE: Dimensionless = Units.Percent.of(0.03)
    val MOMENT_OF_INERTIA_TOP: MomentOfInertia = Units.KilogramSquareMeters.of(.0008)
    val MOMENT_OF_INERTIA_BOTTOM: MomentOfInertia = Units.KilogramSquareMeters.of(.0008)

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


    val CONNFIG_BOTTOM: TalonFXConfiguration = TalonFXConfiguration().apply {
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