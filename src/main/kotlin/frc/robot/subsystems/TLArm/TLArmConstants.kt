package frc.robot.subsystems.TLArm

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Current
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object TLArmConstants {
    const val KP = 0.0
    const val KD = 0.0
    const val KI = 0.0
    const val KV = 0.0
    val CURRENT_LIMIT: Measure<Current> = Units.Amps.of(40.0)
    val dramRadius: Measure<Distance> = Units.Centimeter.of(3.0)

    val MOTOR_CONFIGURATION = TalonFXConfiguration().apply {
        MotorOutput = MotorOutputConfigs().apply {
            Inverted = InvertedValue.Clockwise_Positive
            NeutralMode = NeutralModeValue.Brake
        }
        CurrentLimits = CurrentLimitsConfigs().apply {
            val currentLimit = CURRENT_LIMIT.`in`(Units.Amps)
            StatorCurrentLimit = currentLimit * 2
            SupplyCurrentLimit = currentLimit
            SupplyCurrentLimitEnable = true
            StatorCurrentLimitEnable = true
        }
        Slot0 = Slot0Configs().apply {
            kP = KP
            kD = KD
            kI = KI
            kV = KV
        }
        Feedback = FeedbackConfigs().apply {
            SensorToMechanismRatio = 2 * Math.PI
        }
    }

}