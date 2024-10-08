package frc.robot.subsystems.conveyor

import edu.wpi.first.units.Current
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object ConveyorConstants {
    val currentLimiter: Measure<Current> = Units.Amps.of(40.0)
}