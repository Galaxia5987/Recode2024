package frc.robot.subsystems.telescopicArm

import edu.wpi.first.units.Distance
import edu.wpi.first.units.Measure
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

class TelescopicArm private constructor(private var io: TelescopicArmIO) : SubsystemBase() {
    private val inputs = io.inputs

    companion object {
        @Volatile
        private var instance: TelescopicArm? = null

        fun initialize(io: TelescopicArmIO) {
            synchronized(this) {
                if (instance == null) {
                    instance = TelescopicArm(io)
                }
            }
        }

        fun getInstance(): TelescopicArm = instance ?: throw IllegalStateException(
            "telescopic arm has not been initialized. Call initialize(io:TLArmIO) first"
        )
    }

    fun setHeight(setPoint: Measure<Distance>): Command = Commands.runOnce({ io.setHeight(setPoint) })

    override fun periodic() {
        io.updateInputs()
    }
}