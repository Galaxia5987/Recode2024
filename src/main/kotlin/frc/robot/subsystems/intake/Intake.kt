package frc.robot.subsystems.intake

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase

class Intake private constructor(val io:IntakeIO):SubsystemBase() {
    fun intake():Command{
        Commands.parallel(io.setSpinPower(), io.setCenterPower(), io.setAngle())
        }

}