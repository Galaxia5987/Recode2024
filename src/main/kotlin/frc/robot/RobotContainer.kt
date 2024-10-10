package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.commandGroups.IntakeCommands
import frc.robot.commandGroups.ShootingCommands
import frc.robot.commandGroups.WarmupCommands
import frc.robot.scoreState.AmpState
import frc.robot.scoreState.ClimbState
import frc.robot.scoreState.ScoreState
import frc.robot.scoreState.ShootState
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.swerve.SwerveDrive
import frc.robot.ControllerInputs.driverController as driverController
import frc.robot.ControllerInputs.operatorController as operatorController

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

    private var currentState: ScoreState

    init {
        currentState = shootState

        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()
    }

    private fun configureDefaultCommands() {
        swerveDrive.defaultCommand = swerveDrive.driveCommand(
            { -driverController().leftY },
            { -driverController().leftX },
            { 0.6 * -driverController().rightX })
    }

    private fun configureButtonBindings() {
        driverController().y().onTrue(Commands.runOnce(swerveDrive::resetGyro))

        driverController().rightTrigger().whileTrue(Commands.defer({currentState.execute()}, currentState.execute().requirements))
        driverController().a().onTrue(Commands.runOnce({ currentState = shootState }))
        driverController().b().onTrue(Commands.runOnce({ currentState = ampState }))
        driverController().x().onTrue(Commands.runOnce({ currentState = climbState }))

        driverController().rightBumper().whileTrue(ShootingCommands.shootOverStage())

        driverController().leftTrigger().whileTrue(IntakeCommands.intake())
        driverController().leftBumper().whileTrue(IntakeCommands.outtake())
        driverController().back()
            .whileTrue(Gripper.getInstance().setRollerPower(0.4))
            .onFalse(Gripper.getInstance().stop())
        driverController().start().whileTrue(Intake.getInstance().reset())

        driverController().povUp().whileTrue(Climb.getInstance().openClimb())
        driverController().povDown().whileTrue(Climb.getInstance().closeClimb())
    }

    fun getAutonomousCommand(): Command = Commands.none()

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) = NamedCommands.registerCommand(name, command)
        register("score", shootState.execute())
        register("closeShoot", ShootingCommands.closeShoot().withTimeout(4.0))
        register("warmup", WarmupCommands.warmup())
        register("ampScore", ampState.execute())
        register("intake", IntakeCommands.intake())
        register("outtake", IntakeCommands.outtake())
        register("stopIntake", IntakeCommands.stopIntake())
        register("rollShooter", Shooter.getInstance().rollNote())
    }
}
