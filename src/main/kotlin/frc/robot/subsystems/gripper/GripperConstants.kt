package frc.robot.subsystems.gripper

import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.Current
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object GripperConstants {
    const val ROLLER_INVERTED_VALUE = true
    val currentLimit : Measure<Current> = Units.Amp.of(40.0)
}