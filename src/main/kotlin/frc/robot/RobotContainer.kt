package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.scoreState.AmpState
import frc.robot.scoreState.ClimbState
import frc.robot.scoreState.ScoreState
import frc.robot.scoreState.ShootState
import frc.robot.subsystems.swerve.SwerveDrive

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
    private val swerveDrive: SwerveDrive = SwerveDrive.getInstance()

    private val testController = CommandXboxController(2)

    private val autoChooser: SendableChooser<Command> = AutoBuilder.buildAutoChooser()
    private val shootState: ShootState by lazy { ShootState() }
    private val ampState: AmpState by lazy { AmpState() }
    private val climbState: ClimbState by lazy { ClimbState() }

    private lateinit var currentState: ScoreState

    init {
        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()

        currentState = shootState
    }

    private fun configureDefaultCommands() {

        swerveDrive.defaultCommand = swerveDrive.driveCommand(
            { -ControllerInputs.driverController().leftY },
            { -ControllerInputs.driverController().leftX },
            { 0.6 * -ControllerInputs.driverController().rightX })
    }

    private fun configureButtonBindings() {
        ControllerInputs.operatorController().a().whileTrue(currentState.execute())
        ControllerInputs.operatorController().b().onTrue(Commands.runOnce({ currentState = shootState }))
        ControllerInputs.operatorController().x().onTrue(Commands.runOnce({ currentState = ampState }))
        ControllerInputs.operatorController().y().onTrue(Commands.runOnce({ currentState = climbState }))
    }

    fun getAutonomousCommand(): Command = Commands.none()

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) = NamedCommands.registerCommand(name, command)
    }
}
