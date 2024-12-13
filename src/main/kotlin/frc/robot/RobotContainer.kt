package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import java.util.Optional
import kotlin.math.absoluteValue
import edu.wpi.first.math.MathUtil
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.climb.ClimbIOReal
import java.util.function.DoubleSupplier
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import frc.robot.subsystems.conveyor.Conveyor
import frc.robot.subsystems.conveyor.ConveyorIO
import frc.robot.subsystems.conveyor.ConveyorIOReal
import frc.robot.subsystems.conveyor.ConveyorIOSim
import frc.robot.subsystems.gripper.Gripper
import frc.robot.subsystems.gripper.GripperIOReal
import frc.robot.subsystems.gripper.GripperIOSIm
import frc.robot.subsystems.hood.Hood
import frc.robot.subsystems.hood.HoodIOReal
import frc.robot.subsystems.hood.HoodIOSim
import frc.robot.subsystems.intake.Intake
import frc.robot.subsystems.intake.IntakeIOReal
import frc.robot.subsystems.intake.IntakeIOSim
import frc.robot.subsystems.shooter.Shooter
import frc.robot.subsystems.shooter.ShooterIOReal
import frc.robot.subsystems.shooter.ShooterIOSim
import frc.robot.subsystems.swerve.*
import frc.robot.subsystems.telescopicArm.TelescopicArm
import frc.robot.subsystems.telescopicArm.TelescopicArmIOReal
import frc.robot.subsystems.telescopicArm.TelescopicArmIOSim
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the [Robot]
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
object RobotContainer {
    //    private val swerveDrive: SwerveDrive
//    private val climb: Climb
    private val shooter: Shooter
    private val hood: Hood
    private val driverController = CommandXboxController(0)
    private val operatorController = CommandXboxController(1)
    private val testController = CommandXboxController(2)

//    private val autoChooser: SendableChooser<Command>

    init {
        when (Constants.CURRENT_MODE) {
            Constants.Mode.REAL -> {
//                SwerveDrive.initialize(GyroIOReal(), SwerveConstants.OFFSETS)
//                Climb.initialize(ClimbIOReal())
//                climb = Climb.getInstance()
                Conveyor.initialize(ConveyorIOReal())
                Gripper.initialize(GripperIOReal())
                Hood.initialize(HoodIOReal())
                Intake.initialize(IntakeIOReal())
                Shooter.initialize(ShooterIOReal())
                TelescopicArm.initialize(TelescopicArmIOReal())
            }

            Constants.Mode.SIM -> {
//              SwerveDrive.initialize(GyroIOSim(), SwerveConstants.OFFSETS, * moduleIOs )
                Conveyor.initialize(ConveyorIOSim())
                Gripper.initialize(GripperIOSIm())
                Hood.initialize(HoodIOSim())
                Intake.initialize(IntakeIOSim())
                Shooter.initialize(ShooterIOSim())
                TelescopicArm.initialize(TelescopicArmIOSim())

            }

            else -> {}
        }


//        swerveDrive = SwerveDrive.getInstance()

        shooter = Shooter.getInstance()
        hood = Hood.getInstance();

//        autoChooser = AutoBuilder.buildAutoChooser()

        registerAutoCommands()
//        configureButtonBindings()
        configureDefaultCommands()
    }

//    private fun configureButtonBindings() {
//        driverController.y().onTrue(Commands.runOnce({ swerveDrive.resetGyro() }))
//
//    }

    private fun configureDefaultCommands() {

//        swerveDrive.setDefaultCommand(
//            swerveDrive.driveCommand(
//                { -driverController.leftY },
//                { -driverController.leftX },
//                { 0.6 * -driverController.rightX })
//        )

//        climb.setDefaultCommand(
//            climb.setPower {
//                MathUtil.applyDeadband(
//                    -(driverController.leftTriggerAxis + 1) / 2
//                            + (driverController.rightTriggerAxis + 1) / 2,
//                    0.15
//                )
//            }
//        )
    }

    fun getAutonomousCommand(): Command = Commands.none()

    private fun registerAutoCommands() {
        fun register(name: String, command: Command) = NamedCommands.registerCommand(name, command)
    }
}
