package frc.robot.subsystems.gripper

import edu.wpi.first.units.Power
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.subsystems.hood.Hood
import org.littletonrobotics.junction.Logger

class Gripper private constructor(private val io: GripperIO) : SubsystemBase() {
    private val inputs = io.inputs

    companion object {
        @Volatile
        private var instance: Gripper? = null

        fun initialize(io: GripperIO) {
            synchronized(this) {
                if (instance == null) {
                    instance = Gripper(io)
                }
            }
        }

        fun getInstance(): Gripper {
            return instance ?: throw IllegalStateException(
                "Gripper has not been initialize. call initialize(io:GripperIO) first"
            )
        }
    }

    fun setPower(power: Double): Command = Commands.runOnce({ io.setPower(power) }).withName("setPower")
    fun gripperIn(): Command = Commands.runOnce({ io.setPower(GripperConstants.GRIPPER_POWER) }).withName("gripperIN")
    fun gripperOut(): Command =
        Commands.runOnce({ io.setPower(-GripperConstants.GRIPPER_POWER) }).withName("gripperOut")

    fun stopGripper(): Command = Commands.runOnce({ io.setPower(0.0) }).withName("stopGripper")
    fun hasNote(): Boolean = inputs.hasNote

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs("Gripper", inputs)
    }


}