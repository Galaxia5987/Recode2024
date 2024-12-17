package frc.robot.subsystems.elevator

import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import org.littletonrobotics.junction.Logger

class Elevator private constructor(private val io: ElevatorIO) : SubsystemBase() {

    private var inCharacterizationMode = false
    companion object {
        @Volatile
        private var instance: Elevator? = null

        fun initialize(io: ElevatorIO) {
            synchronized(this) {
                if (Elevator.instance == null) {
                    instance = Elevator(io)
                }
            }
        }

        fun getInstance(): Elevator {
            return Elevator.instance ?: throw IllegalStateException(
                "Elevator has not been initialized. Call initialize(io: ElevatorIO) first."
            )
        }
    }

    fun setPosition(position: Double) {
        io.setHeight(position)
    }

    fun setPower(percentOutput: Double) {
        io.setPower(percentOutput)
    }

    fun characterize(): Command {
        var routine = SysIdRoutine(
            SysIdRoutine.Config(),
            SysIdRoutine.Mechanism(
                { voltage: Voltage -> io.setVoltage(voltage) },
                { sysIdRoutineLog: SysIdRoutineLog -> io.updateRoutineLog(sysIdRoutineLog) },
                this
            )
        )
        return Commands.sequence(
            Commands.runOnce({inCharacterizationMode = true}),
            routine.dynamic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(1.0),
            routine.dynamic(SysIdRoutine.Direction.kReverse),
            Commands.waitSeconds(1.0),
            routine.quasistatic(SysIdRoutine.Direction.kForward),
            Commands.waitSeconds(1.0),
            routine.quasistatic(SysIdRoutine.Direction.kReverse)
        ).finallyDo(Runnable{inCharacterizationMode=false})
    }


    fun reset() {
        io.reset()
    }

    override fun periodic() {
        io.updateInputs()
        Logger.processInputs("elevator", io.inputs)
    }
}
