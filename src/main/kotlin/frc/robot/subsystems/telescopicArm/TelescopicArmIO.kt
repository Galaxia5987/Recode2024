package frc.robot.subsystems.telescopicArm

import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.Voltage
import org.team9432.annotation.Logged

interface TelescopicArmIO {           //TL for telescopic
    var inputs: LoggedTLArmInputs
    fun updateInput()
    fun setDistance(distance: Measure<Distance>)

    @Logged
    open class TLArmInputs {
        var currentPose: Measure<Distance> = Units.Meters.zero()
        var voltage:Measure<Voltage> = Units.Volt.zero()
    }
}