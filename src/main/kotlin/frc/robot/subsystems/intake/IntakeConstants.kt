package frc.robot.subsystems.intake

import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object IntakeConstants {
    val GEAR_RATIO=55.56
    val kP=55.0
    val kI=3.0
    val kD=0.1
    val SPIN_POWER=0.4
    val Centar_POWER=0.4
    val ANGLE_UP: Measure<Angle> = Units.Degree.of(126.2)
    val ANGLE_DOWN: Measure<Angle> = Units.Degree.of(6.5)
}