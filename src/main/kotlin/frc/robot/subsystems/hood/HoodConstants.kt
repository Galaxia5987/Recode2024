package frc.robot.subsystems.hood

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units

object HoodConstants {
    val MOTOR_CONFIGURATION = TalonFXConfiguration()
    val restAngle: Measure<Angle> = MutableMeasure.ofBaseUnits(100.0, Units.Degree)
    const val GEAR_RATIO: Double = 3.0 * (36.0 / 18.0) * (158.0 / 18.0)
    val ANGLE_KP = 0.0
    val ANGLE_KD = 0.0
    val ANGLE_KI = 0.0
    const val TOLERANCE: Double = 0.75 / 360

    init {

        MOTOR_CONFIGURATION.apply {
            MotorOutput.apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
            Feedback.apply {
                RotorToSensorRatio = 1.0
                SensorToMechanismRatio = GEAR_RATIO
            }
            Slot0.apply {
                kP = ANGLE_KP
                kI = ANGLE_KI
                kD = ANGLE_KD
            }
            CurrentLimits.apply {
                StatorCurrentLimitEnable = true
                StatorCurrentLimit = 80.0
                SupplyCurrentLimitEnable = true
                SupplyCurrentLimit = 40.0
            }
        }
    }
}