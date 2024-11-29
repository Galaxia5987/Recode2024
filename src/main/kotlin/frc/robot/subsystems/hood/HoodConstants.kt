package frc.robot.subsystems.hood

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.MomentOfInertia

object HoodConstants {

    val REST_ANGLE: Angle = Units.Degree.of(100.0)
    const val GEAR_RATIO: Double = 3.0 * (36.0 / 18.0) * (158.0 / 18.0)
    val ANGLE_KP = 0.0
    val ANGLE_KD = 0.0
    val ANGLE_KI = 0.0
    const val TOLERANCE: Double = 0.75 / 360
    val MOMENT_OF_INERTIA: MomentOfInertia = Units.KilogramSquareMeters.of(0.0003)

    val MOTOR_CONFIGURATION = TalonFXConfiguration().apply {
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