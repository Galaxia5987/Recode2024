package frc.robot.subsystems.intake

import edu.wpi.first.units.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.StartEndCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Intake private constructor(private val io: IntakeIO):SubsystemBase(){
    private var input:LoggedIntakeInput = io.inputs

    companion object{
        @Volatile
        private var instance:Intake? = null

        fun initialize(io:IntakeIO){
            synchronized(this){
                if(instance == null){
                    instance = Intake(io)
                }
            }
        }
        fun getInstance():Intake{
            return instance ?: throw  IllegalStateException(
                "Intake has not been initialized. C all initialize(io: IntakeIO) first."
            )
        }
    }
    fun setAngle(angle: Double):Command = run { io.setAngle(angle) }.withName("Set Angle Intake")

    fun resetAngle():Command {
       return Commands.run({ io.resetAngle() }).withName("resetAngle")
    }
    fun setAnglePower(power:Double):Command {
        return Commands.run({ io.setAnglePower(power) }).withName("setAnglePower")
    }

    override fun periodic() {
        io.updateInput()
        Logger.processInputs("Intake",input)
        setAngle(input.angle)
    }
}