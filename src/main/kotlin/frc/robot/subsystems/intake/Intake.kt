package frc.robot.subsystems.intake

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Intake private constructor (private val io: IntakeIO):SubsystemBase(){

    private var angleSetPoint: Measure<Angle> = Units.Degree.zero()
    val inputs=io.inputs

    companion object{// Custom Singleton Implementation
        @Volatile
        private var instance: Intake? = null
        fun initialize(io: IntakeIO){
            synchronized(this) {
                if (instance == null)
                    instance = Intake(io)
            }

        }

        fun getInstance():Intake{
            return instance ?:throw IllegalStateException("instanceIsNull")

        }

    }
    fun setAnglePower(power:Double):Command{
        return Commands.runOnce({
            io.setAngleMotorPow(power)
        })

    }

    fun setCenterPower(power: Double):Command{
        return Commands.runOnce({
            io.setCenterPower(power)
        })
    }

    fun setAnglePosition(angle: Measure<Angle>):Command{
        return Commands.runOnce({
            angleSetPoint=angle
            io.setAngleMotorAngle(angle)
        })
    }

    fun setRollerPower(power: Double):Command{
        return Commands.runOnce({
            io.setRollerPower(power)
        })
    }

    fun intake(): Command{
        return Commands.parallel(
            setAnglePosition(IntakeConstants.INTAKE_ANGLE),
            setRollerPower(IntakeConstants.INTAKE_ROLLER_POWER),
            setCenterPower(IntakeConstants.INTAKE_CENTER_POWER)
        )
    }

    fun outtake():Command{
        return Commands.parallel(
            setAnglePosition(IntakeConstants.REST_ANGLE),
            setRollerPower(-IntakeConstants.INTAKE_ROLLER_POWER),
            setCenterPower(-IntakeConstants.INTAKE_CENTER_POWER)
        )
    }

    fun stopSpin():Command{
        return Commands.parallel(
            setRollerPower(0.0),setCenterPower(0.0)
        )
    }

    fun stop():Command{
        return setAnglePosition(IntakeConstants.REST_ANGLE).alongWith(stopSpin())
    }

    fun reset():Command{
        return Commands.run({io.setAngleMotorPow(-0.3)}).finallyDo(Runnable {
            io.reset()
            io.setAngleMotorPow(0.0)
        })
    }

    override fun periodic() {
        io.updateinputs()
        Logger.processInputs("intake",inputs)
    }

}