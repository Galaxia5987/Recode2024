package frc.robot.subsystems.elevator

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.AutoLogOutput

class Elevator private constructor(private val io: ElevatorIO) : SubsystemBase() {



    fun setPosition(position: Double) {
        io.setPosition(position)
    }

    fun setPower(percentOutput: Double) {
        io.setPower(percentOutput)
    }

    fun resat() {
        io.setPosition()=0.0
    }
}
