package frc.robot.subsystems.TLArm

import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

class TLArm private constructor(private var io: TLArmIO) {
    private var inputs = io.inputs

    companion object {
        @Volatile
        private var instance: TLArm? = null

        fun initialize(io: TLArmIO) {
            synchronized(true) {
                if (instance == null) {
                    instance = TLArm(io)
                }
            }
        }

        fun getInstance(): TLArm = instance ?: throw IllegalStateException(
            "telescopic arm has not been initialized. Call initialize(io:TLArmIO) first"
        )
    }
    fun setPosition(setPoint:Measure<Distance>):Command = Commands.runOnce({io.setPosition(setPoint)})
}