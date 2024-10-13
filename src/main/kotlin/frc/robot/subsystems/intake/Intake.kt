package frc.robot.subsystems.intake

import edu.wpi.first.units.Angle
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Intake private constructor(val io:IntakeIO):SubsystemBase() {
    fun intake():Command{
        Commands.parallel(io.setSpinPower(), io.setCenterPower(), io.setAngle())

    companion object{
        @Volatile
        private var instance: Intake?=null
        fun initialize(io: IntakeIO){
            synchronized(this)
            {if (instance==null){instance= Intake(io) } }
        }
       fun getInstance(): Intake {
         return instance?:throw IllegalStateException(" Intake Instance null")
       }
    }
    fun intake():Command {
       return Commands.parallel(
            Commands.runOnce({io.setSpinPower(IntakeConstants.SPIN_POWER)}),
            Commands.runOnce({io.setCenterPower(IntakeConstants.Centar_POWER)}),
            Commands.runOnce({io.setAngle(IntakeConstants.ANGLE_DOWN)})
       )
    }
    fun outtake():Command {
        return Commands.parallel(
            Commands.runOnce({io.setSpinPower(-IntakeConstants.SPIN_POWER)}),
            Commands.runOnce({io.setCenterPower(-IntakeConstants.Centar_POWER)}),
            Commands.runOnce({io.setAngle(IntakeConstants.ANGLE_UP)})
        )
    }
    fun reset():Command{
        return Commands.runOnce(
            {io.setAnglePower(-0.3)}).finallyDo(
            Runnable{ io.reset()
            io.setAnglePower(0.0)})
    }
    fun stop(): Command {
        return Commands.runOnce({
        io.setAngle(IntakeConstants.ANGLE_UP)
        io.setCenterPower(0.0)
        io.setCenterPower(0.0)})
        }

    fun SetAngle(angle: Measure<Angle>): Command{
        return Commands.run({
            angleSetpoint=angle
            io.setAngle(angle)

        })
    }

    fun SetCenterPower(power: Double): Command{
        return Commands.runOnce({
            io.setCenterPower(power)
        })
    }

    fun setSpinPower(power: Double): Command{
        return Commands.runOnce({
            io.setSpinPower(power)
        })
    }

  override fun periodic() {
        io.updateInputs()
        Logger.processInputs(this::class.simpleName, io.inputs)
    }
}

