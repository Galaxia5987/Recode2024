package frc.robot

import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
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
        swerveDrive = when (Constants.CURRENT_MODE) {
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
                    object : GyroIO {},
                    ModuleIOSim(TunerConstants.FrontLeft),
                    ModuleIOSim(TunerConstants.FrontRight),
                    ModuleIOSim(TunerConstants.BackLeft),
                    ModuleIOSim(TunerConstants.BackRight)
                )
            }
            else -> {
                Drive(
                    object : GyroIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    object : ModuleIO {},
                    object : ModuleIO {}
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
            { MathUtil.applyDeadband(-driverController().leftX, 0.15) },
            { MathUtil.applyDeadband(-driverController().leftY, 0.15) },
            { 0.5 * MathUtil.applyDeadband(-driverController().rightX, 0.15) }
        )
    }

    private fun configureButtonBindings() {
        driverController().y().onTrue(
            Commands.runOnce({
                swerveDrive.pose = Pose2d(swerveDrive.pose.translation, Rotation2d())
            },swerveDrive)
        .ignoringDisable(true));
    }

    fun getAutonomousCommand(): Command = Commands.none()

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) = NamedCommands.registerCommand(name, command)
    }
}
