package frc.robot.commandGroups

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Velocity
import edu.wpi.first.math.geometry.Translation2d
import frc.robot.lib.getTranslationByColor

val HOOD_ANGLE_SUPER_POOP: Measure<Angle> = Units.Degrees.of(87.0)
val SHOOTER_VELOCITY_SUPER_POOP: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(77.07)
val CONVEYOR_VELOCITY_SUPER_POOP: Measure<Velocity<Angle>> = Units.RotationsPerSecond.of(60.0)

private val SUPER_POOP_TRANSLATION_BLUE = Translation2d(0.8, 7.1)

val SUPER_POOP_TRANSLATION: Translation2d
    get() = getTranslationByColor(SUPER_POOP_TRANSLATION_BLUE)

const val SUPER_POOP_TURN_TOLERANCE = 0.06
