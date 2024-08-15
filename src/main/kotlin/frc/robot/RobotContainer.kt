package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.robot.commandGroups.CommandGroups
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

    init {
        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()
    }

    private fun configureDefaultCommands() {

        swerveDrive.defaultCommand = swerveDrive.driveCommand(
            { ControllerInputs.getDriverController().leftY },
            { ControllerInputs.getDriverController().leftX },
            { 0.6 * ControllerInputs.getDriverController().rightX })

//        climb.setDefaultCommand(
//            climb.setPower {
//                MathUtil.applyDeadband(
//                    -(ControllerInputs.getDriverController().leftTriggerAxis + 1) / 2
//                            + (ControllerInputs.getDriverController().rightTriggerAxis + 1) / 2,
//                    0.15
//                )
//            }
//        )
    }

    private fun configureButtonBindings() {
        ControllerInputs.getDriverController().y().onTrue(Commands.runOnce({ swerveDrive.resetGyro() }))

//        ControllerInputs.getDriverController().start().whileTrue(intake.reset())
//        ControllerInputs.getDriverController().rightBumper()
//            .whileTrue(gripper.feed())
//            .onFalse(gripper.setRollerPower(0.0))

//        ControllerInputs.getDriverController().leftTrigger()
//            .whileTrue(CommandGroups.intake(Commands.none()))
//            .onFalse(intake.stop().alongWith(gripper.setRollerPower(0.0)))
//        ControllerInputs.getDriverController().leftBumper()
//            .whileTrue(CommandGroups.outtake())
//            .onFalse(intake.stop().alongWith(gripper.setRollerPower(0.0)))

        ControllerInputs.getDriverController().x().onTrue(Constants.State.SHOOT.setState())
        ControllerInputs.getDriverController().b()
            .whileTrue(CommandGroups.setpointShoot())
            .onFalse(CommandGroups.stopWarmup())
//        ControllerInputs.getDriverController().a().onTrue(Constants.State.CLIMB.setState())
//        ControllerInputs.getDriverController().b().onTrue(Constants.State.AMP.setState())

//        ControllerInputs.getDriverController().rightTrigger().whileTrue(
//            Constants.CURRENT_STATE?.execute() ?: Commands.none())

//        ControllerInputs.getDriverController().start().onTrue(climb.lock().withTimeout(2.0))
//        ControllerInputs.getDriverController().back().onTrue(climb.open().withTimeout(2.0))
    }

    fun getAutonomousCommand(): Command = Commands.none()

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) = NamedCommands.registerCommand(name, command)
    }
}
