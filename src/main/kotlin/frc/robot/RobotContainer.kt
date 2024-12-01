package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers
import frc.robot.ControllerInputs.driverController
import frc.robot.lib.enableAutoLogOutputFor
import frc.robot.subsystems.swerve.SwerveDrive
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
    private val swerveDrive = SwerveDrive.getInstance()

    private val testController = CommandXboxController(2)

    private val autoChooser: LoggedDashboardChooser<Command>

    init {
        enableAutoLogOutputFor(this)

        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()

        swerveDrive.configAutoBuilder()

        autoChooser = LoggedDashboardChooser("AutoChooser", AutoBuilder.buildAutoChooser())
        SmartDashboard.putData("autoChooser", autoChooser.sendableChooser)
    }

    private fun configureDefaultCommands() {
        swerveDrive.defaultCommand = swerveDrive.driveCommand(
            { -driverController().leftY },
            { -driverController().leftX },
            { 0.5 * -driverController().rightX }
        )
    }

    private fun configureButtonBindings() {
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).onTrue(swerveDrive.setBrakeMode())
        RobotModeTriggers.disabled().debounce(7.0).onTrue(swerveDrive.setCoastMode())

        driverController().y().onTrue(Commands.runOnce(swerveDrive::resetGyro))

    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) = NamedCommands.registerCommand(name, command)
    }

}
