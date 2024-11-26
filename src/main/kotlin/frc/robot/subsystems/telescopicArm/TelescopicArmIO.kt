package frc.robot.subsystems.telescopicArm


import edu.wpi.first.units.DistanceUnit
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage

import org.team9432.annotation.Logged

interface TelescopicArmIO {           //TL for telescopic
    var inputs: LoggedTLArmInputs
    fun updateInputs() {}
    fun setHeight(distance: Distance) {}

    @Logged
    open class TLArmInputs {
        var currentPose: Distance = Units.Meters.zero()
        var voltage: Voltage = Units.Volt.zero()
    }
}