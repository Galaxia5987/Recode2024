package frc.robot.subsystems.telescopicArm

import edu.wpi.first.units.Current
import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object TelescopicArmConstants {
    const val KP = 0.0
    const val KD = 0.0
    const val KI = 0.0
    const val KV = 0.0
    const val GEAR_RATIO = 3.0
    const val MOMENT_OF_INERTIA = 3.0
    const val CONVERSION_FACTOR = 2 * Math.PI
    val CURRENT_LIMIT: Measure<Current> = Units.Amps.of(40.0)
    val dramRadius: Measure<Distance> = Units.Centimeter.of(3.0)
}