package frc.robot

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController

object ControllerInputs {
    private val driverController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)

    fun driverController(): CommandXboxController {
        return driverController
    }

    fun operatorController(): CommandXboxController {
        return operatorController
    }

    fun startRumble(): Command {
        return Commands.runOnce(
            {
                driverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 1.0)
            })
    }

    fun stopRumble(): Command {
        return Commands.runOnce(
            {
                driverController.hid.setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
            })
    }
}