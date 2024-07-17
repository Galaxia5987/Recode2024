package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.subsystems.swerve.Swerve
import frc.robot.subsystems.swerve.SwerveConstants
import swervelib.SwerveDrive

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
    private val swerveDrive = Swerve(
        SwerveConstants.SWERVE_CONFIG, SwerveConstants.SWERVE_CONTROLLER_CONFIG, SwerveConstants.MAX_SPEED
    )

    private val driverController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)
    private val testController = CommandXboxController(2)
    private val autoChooser = AutoBuilder.buildAutoChooser()

    init {
        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()
    }

    private fun configureDefaultCommands() {
        swerveDrive.driveCommand(
            { -driverController.leftY },
            { -driverController.leftX },
            { 0.6 * -driverController.rightX })
    }

    private fun configureButtonBindings() {
        driverController.y().onTrue(Commands.runOnce({ swerveDrive.zeroGyro() }))
    }

    fun getAutonomousCommand(): Command = autoChooser.selected

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) = NamedCommands.registerCommand(name, command)
    }
}
