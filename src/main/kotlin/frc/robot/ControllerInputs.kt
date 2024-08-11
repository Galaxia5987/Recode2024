package frc.robot

import edu.wpi.first.wpilibj2.command.button.CommandXboxController

object ControllerInputs {
    private val driverController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)

    fun getDriverController(): CommandXboxController {
        return driverController
    }

    fun getOperatorController(): CommandXboxController {
        return operatorController
    }
}