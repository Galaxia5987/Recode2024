package frc.robot.subsystems.gripper

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger


class Gripper private constructor(private val io: GripperIO): SubsystemBase() {
    private val timer = Timer()
    @AutoLogOutput
    private var rollerPowerSetPoint = 0.0

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

        fun getInstance() : Gripper {
            return instance ?: throw IllegalArgumentException(
                "Gripper has not been initialized. Call initialize(io: GripperIO) first."
            )
        }
    }

    init {
        timer.start()
        timer.reset()
    }

    fun setRollerPower(power: Double): Command {
        return run {
            rollerPowerSetPoint = power
            io.setRollerMotorPower(power)
        }.withName("Set Roller Power")
    }

    fun feed(): Command {
        return setRollerPower(GripperConstants.INTAKE_POWER).withTimeout(0.4)
    }

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, io.inputs)
    }
}