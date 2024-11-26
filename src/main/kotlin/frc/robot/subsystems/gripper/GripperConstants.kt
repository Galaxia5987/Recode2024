package frc.robot.subsystems.gripper


import edu.wpi.first.units.CurrentUnit
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object GripperConstants {
    const val GRIPPER_POWER = 0.7
    const val IS_ROLL_INVERTED_VALUE = false
    val currentLimit: Measure<CurrentUnit> = Units.Amps.of(40.0)
}