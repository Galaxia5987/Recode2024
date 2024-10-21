package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import frc.robot.ControllerInputs.driverController
import frc.robot.ControllerInputs.operatorController
import frc.robot.commandGroups.*
import frc.robot.scoreState.AmpState
import frc.robot.scoreState.ClimbState
import frc.robot.scoreState.ScoreState
import frc.robot.scoreState.ShootState
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.leds.AMP_STATE_COLOR
import frc.robot.subsystems.leds.LEDs
import frc.robot.subsystems.leds.SHOOT_STATE_COLOR
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.swerve.SwerveDrive
import org.littletonrobotics.junction.AutoLogOutput


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
    private val swerveDrive = SwerveDrive.getInstance()
    private val gripper = Gripper.getInstance()
    private val climb = Climb.getInstance()
    private val intake = Intake.getInstance()
    private val leds = LEDs.getInstance()

    private val testController = CommandXboxController(2)

    private val autoChooser: SendableChooser<Command>
    private val shootState: ShootState by lazy { ShootState() }
    private val ampState: AmpState by lazy { AmpState() }
    private val climbState: ClimbState by lazy { ClimbState() }

    private var currentState: ScoreState

    init {
        currentState = shootState

        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()

        swerveDrive.configAutoBuilder()

        autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("autoChooser", autoChooser)
    }

    private fun configureDefaultCommands() {
        swerveDrive.defaultCommand = swerveDrive.driveCommand(
            { -driverController().leftY },
            { -driverController().leftX },
            { 0.5 * -driverController().rightX })
    }

    private fun configureButtonBindings() {
        driverController().y().onTrue(Commands.runOnce(swerveDrive::resetGyro))

        driverController().rightTrigger()
            .whileTrue(Commands.defer({ currentState.execute() }, currentState.execute().requirements))
        driverController().a().onTrue(
            Commands.runOnce({ currentState = shootState })
                .alongWith(leds.setSolidMode(SHOOT_STATE_COLOR))
        )
        driverController().b().onTrue(
            Commands.runOnce({ currentState = ampState })
                .alongWith(leds.setSolidMode(AMP_STATE_COLOR))
        )

        driverController().x().whileTrue(closeShoot())
            .onFalse(finishScore())
        driverController().povLeft().whileTrue(trussSetpoint())
            .onFalse(finishScore())

        driverController().rightBumper().whileTrue(shootOverStage())

        driverController().leftTrigger().whileTrue(intake())
            .onFalse(stopIntake())
        driverController().leftBumper().whileTrue(outtake())
            .onFalse(stopIntake())
        driverController().back()
            .whileTrue(gripper.setRollerPower(0.4))
            .onFalse(gripper.stop())
        driverController().start().whileTrue(intake.reset())

        driverController().povUp().whileTrue(climb.openClimb())
        driverController().povDown().whileTrue(climb.closeClimb())

        operatorController().R2().whileTrue(climb.openClimb())
        operatorController().L2().whileTrue(climb.closeClimb())

        operatorController().R1().whileTrue(gripper.setRollerPower(-0.4))
            .onFalse(gripper.stop())
        operatorController().L1().whileTrue(gripper.setRollerPower(0.4))
            .onFalse(gripper.stop())

        operatorController().cross().onTrue(gripper.enableSensor())
        operatorController().circle().onTrue(gripper.disableSensor())

        operatorController().options().whileTrue(intake.reset())
    }

    fun getAutonomousCommand(): Command = autoChooser.selected


    private fun registerAutoCommands() {
        fun register(name: String, command: Command) = NamedCommands.registerCommand(name, command)
        register("score", shootState.init().until { shooterConveyorHoodAtSetpoint() })
        register("finishScore", shootState.end())
        register("warmup", warmup())
        register("intake", intake())
        register("outtake", outtake())
        register("stopIntake", stopIntake())
        register("rollShooter", Shooter.getInstance().rollNote())
        register("setpointShoot", closeShoot())
        register("finishSetpointShoot", finishScore())
    }

    @AutoLogOutput
    fun getState(): String {
        return currentState.execute().name
    }
}
