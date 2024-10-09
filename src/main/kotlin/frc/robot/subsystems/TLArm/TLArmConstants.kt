package frc.robot.subsystems.TLArm

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Current
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object TLArmConstants {
    val MOTOR_CONFIGURATION = TalonFXConfiguration()
    var KP = 0.0
    var KD = 0.0
    var KI = 0.0
    var KV = 0.0

    var CURRENT_LIMIT: Measure<Current> = Units.Amps.of(40.0)

    fun a() {
        val i = 2
    }

    fun b() {
        val i = 3
    }

}