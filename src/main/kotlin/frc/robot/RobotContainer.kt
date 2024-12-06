package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.ControllerInputs.driverController
import frc.robot.generated.TunerConstants
import frc.robot.subsystems.drive.Drive
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.drive.GyroIO
import frc.robot.subsystems.drive.GyroIONavX
import frc.robot.subsystems.drive.ModuleIO
import frc.robot.subsystems.drive.ModuleIOSim
import frc.robot.subsystems.drive.ModuleIOTalonFX

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
    private val swerveDrive: Drive
    private val testController = CommandXboxController(2)

    init {
        swerveDrive = when(Constants.CURRENT_MODE){
            Constants.Mode.REAL -> {
                Drive(
                    GyroIONavX(),
                    ModuleIOTalonFX(TunerConstants.FrontLeft),
                    ModuleIOTalonFX(TunerConstants.FrontRight),
                    ModuleIOTalonFX(TunerConstants.BackLeft),
                    ModuleIOTalonFX(TunerConstants.BackRight)
                )
            }
            Constants.Mode.SIM -> {
                Drive(
                    object: GyroIO {},
                    ModuleIOSim(TunerConstants.FrontLeft),
                    ModuleIOSim(TunerConstants.FrontRight),
                    ModuleIOSim(TunerConstants.BackLeft),
                    ModuleIOSim(TunerConstants.BackRight)
                )
            }
            else -> {
                Drive(
                    object: GyroIO {},
                    object: ModuleIO {},
                    object: ModuleIO {},
                    object: ModuleIO {},
                    object: ModuleIO {}
                )
            }
        }
        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()
    }

    private fun configureDefaultCommands() {
        swerveDrive.defaultCommand = DriveCommands.joystickDrive(
            swerveDrive,
            { -driverController().leftX },
            { -driverController().leftY },
            { 0.5 * -driverController().rightX }
        )
    }

    private fun configureButtonBindings() {
    }

    fun getAutonomousCommand(): Command = Commands.none()

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) = NamedCommands.registerCommand(name, command)
    }
}
