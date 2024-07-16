package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Intake(private val io: IntakeIO) : SubsystemBase() {
    private val inputs = io.inputs

    fun setSpinPower(power: Double): Command {
        return Commands.run({
                io.setSpinPower(power)
            }
        )
    }

}