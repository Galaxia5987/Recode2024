package frc.robot.subsystems.TLArm

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Current
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object TLArmConstants {
    var KP = 0.0
    var KD = 0.0
    var KI = 0.0
    var KV = 0.0
    var CURRENT_LIMIT: Measure<Current> = Units.Amps.of(40.0)
    var DramRatio:Double = 3.0 //the Circumference of the drams compare to cm
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
    }

}