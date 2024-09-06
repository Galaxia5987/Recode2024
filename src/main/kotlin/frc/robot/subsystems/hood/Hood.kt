package frc.robot.subsystems.hood

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants

class Hood private constructor(private var io: HoodIO):SubsystemBase(){
    private val inputs = io.inputs

    companion object{
        @Volatile
        private var instance:Hood? = null

        fun initialize(io:HoodIO){
            synchronized(this){
                if(instance == null){
                    instance = Hood(io)
                }
            }
        }
        fun getInstance():Hood{
            return instance?:throw IllegalStateException(
                "Hood has not been initialized. Call initialize(io: HoodIO) first."
            )
        }
    }
    fun setAngle(angle:Double):Command = Commands.run({io.setAngle(angle)}).withName("set Angle Hood")

    fun setRestAngle():Command = Commands.run({io.setAngle(HoodConstants.restAngle)})

    fun atSetPoint():Command = Commands.run({
        inputs.angle.isNear(inputs.angleSetPoint,HoodConstants.TOLERANCE)
    })

}