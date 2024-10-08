package frc.robot.subsystems.conveyor

import edu.wpi.first.units.Current
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units

object ConveyorConstants {
  const val TOLERANCE = 0.5
  private val CURRENT_LIMIT: Measure<Current> = Units.Amps.of(40.0)
}