package frc.robot.subsystems.gripper

import edu.wpi.first.units.Current
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object GripperConstants {
    const val ROLLER_INVERTED_VALUE = false
    const val INTAKE_POWER = 0.5

    val currentLimit : Measure<Current> = Units.Amp.of(40.0)
}