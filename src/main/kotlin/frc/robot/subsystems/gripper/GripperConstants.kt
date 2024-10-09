package frc.robot.subsystems.gripper

import edu.wpi.first.units.Current
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object GripperConstants {
    const val GRIPPER_POWER = 0.7
    const val IS_ROLL_INVERTED_VALUE = false
    val currentLimiter: Measure<Current> = Units.Amps.of(40.0)
}