
package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.MathUtil
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.climb.ClimbIOTalonFX
import java.util.function.DoubleSupplier

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
    private val driverController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)
    private val testController = CommandXboxController(2)

    private val climb: Climb

    init {
        Climb.initialize(ClimbIOTalonFX())
        climb = Climb.getInstance()
        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()
    }

    private fun configureDefaultCommands() {
        climb.setDefaultCommand(
            climb.setPower {
                MathUtil.applyDeadband(
                    -(driverController.leftTriggerAxis + 1) / 2
                            + (driverController.rightTriggerAxis + 1) / 2,
                    0.15
                )
            }
        )
    }

    private fun configureButtonBindings() {
        driverController.start().onTrue(climb.lock().withTimeout(2.0))
        driverController.back().onTrue(climb.open().withTimeout(2.0))
    }

    fun getAutonomousCommand(): Command = Commands.none()

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) = NamedCommands.registerCommand(name, command)
    }
}
