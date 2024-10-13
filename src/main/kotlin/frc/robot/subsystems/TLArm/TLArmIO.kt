package frc.robot.subsystems.TLArm

import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.Units
import org.team9432.annotation.Logged

interface TLArmIO {           //TL for telescopic
    var inputs: LoggedTLArmInputs
    fun updateInput()
    fun setPosition(setPoint: Measure<Distance>)

    @Logged
    open class TLArmInputs {
        var currentPose:Measure<Distance> = Units.Meters.zero()
    }
}