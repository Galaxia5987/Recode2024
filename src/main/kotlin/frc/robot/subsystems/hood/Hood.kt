package frc.robot.subsystems.hood

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.MutableMeasure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Hood private constructor(private var io: HoodIO):SubsystemBase(){
    private val inputs = io.inputs

        @AutoLogOutput
        var angleSetPoint:MutableMeasure<Angle> = MutableMeasure.zero(Units.Rotations)

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
    fun setAngle(angle:Measure<Angle>):Command = Commands.run({
        io.setAngle(angle)
        angleSetPoint = angle.mutableCopy()
    }).withName("set Angle Hood")

    fun setRestAngle():Command = Commands.run({io.setAngle(HoodConstants.restAngle)})

    fun atSetPoint():Boolean = inputs.angle.isNear(angleSetPoint,HoodConstants.TOLERANCE)


    override fun periodic() {
        io.updateInputs()
        Logger.processInputs("Hood",inputs)
    }
}