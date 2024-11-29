package frc.robot.subsystems.conveyor

import edu.wpi.first.units.AngularVelocityUnit
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Conveyor private constructor(private val io: ConveyorIO) : SubsystemBase() {
    private var inputs = io.inputs

    @AutoLogOutput
    private var setpointSpeed: Measure<AngularVelocityUnit> = Units.RotationsPerSecond.zero()

    companion object {
        @Volatile
        private var instance: Conveyor? = null

        fun initialize(io: ConveyorIO) {
            synchronized(this) {
                if (instance == null) {
                    instance = Conveyor(io)
                }
            }
        }

        fun getInstance(): Conveyor {
            return instance ?: throw IllegalStateException(
                "Conveyor has not been initialize. call initialize(io:ConveyorIO) first"
            )
        }
    }

    fun setPower(vel: AngularVelocity): Command =
        Commands.runOnce({ io.setSpinVelocity(vel) }).withName("setPower")

    fun stopConveyor(): Command =
        Commands.runOnce({ io.setSpinVelocity(Units.RotationsPerSecond.zero()) }).withName("stopGripper")

    fun atSetSpeed(): Boolean =
        inputs.spinMotorVelocity.isNear(setpointSpeed, ConveyorConstants.TOLERANCE.`in`(Units.Percent))
    override fun periodic() {
        io.updateInput()
        Logger.processInputs(this::class.simpleName, inputs)
    }
}