package frc.robot.subsystems.intake

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object IntakeConstants {
    const val gearRatio:Double=1.0
    val kp=0.0
    val ki=0.0
    val kd=0.0
    val INTAKE_ANGLE:Measure<Angle> = Units.Degree.of(21.5)
    val REST_ANGLE:Measure<Angle> = Units.Degree.of(127.0)
    const val INTAKE_ROLLER_POWER = -0.4
    const val INTAKE_CENTER_POWER = -0.4

}