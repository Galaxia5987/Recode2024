package frc.robot.subsystems.conveyor

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Velocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Conveyor private constructor(private val io:ConveyorIO):SubsystemBase() {
    private var inputs = io.inputs
    companion object{
        @Volatile
        private var instance:Conveyor? = null

        fun initialize(io: ConveyorIO){
            synchronized(this){
                if(instance == null){
                    instance = Conveyor(io)
                }
            }
        }
        fun getInstance():Conveyor{
            return instance?:throw IllegalStateException(
                "Conveyor has not been initialize. call initialize(io:ConveyorIO) first"
            )
        }
    }
    fun setPower(vel:Measure<Velocity<Angle>>):Command=Commands.runOnce({io.setSpinPower(vel)}).withName("setPower")
    fun stopConveyor():Command = Commands.runOnce({io.setSpinPower(0.0)}).withName("stopGripper")

    override fun periodic() {
        io.updateInput()
        Logger.processInputs("Conveyor",inputs)
    }
}