package frc.robot.subsystems.climb

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import java.util.function.DoubleSupplier
import java.util.function.Supplier
import kotlin.math.absoluteValue

class Climb private constructor(private val io:ClimbIO) :SubsystemBase() {
    private var input: LoggedClimbInputs = io.inputs

    private var isStopperStuck: () -> Boolean = {false}

    companion object{
        @Volatile
        private var instance: Climb? = null

        fun initialize(io: ClimbIO){
            synchronized(this){
                if(instance == null){
                    instance = Climb(io)
                }
            }
        }
        fun getInstance(): Climb {
            return instance ?: throw  IllegalStateException(
                "Climb has not been initialized. Call initialize(io: IntakeIO) first."
            )
        }
    }
    fun setPower(power:DoubleSupplier):Command{
        return Commands.run({io.setPower(power.asDouble)})
    }

    fun lock():Command{
        return Commands.run({io.lockClimb()}).until(isStopperStuck).andThen({io.disableLockMotor()})
    }


    fun unlock():Command{
        return Commands.run({io.unlockClimb()}).until(isStopperStuck).andThen({io.disableLockMotor()})
    }

    override fun periodic() {
        super.periodic()
    }
}