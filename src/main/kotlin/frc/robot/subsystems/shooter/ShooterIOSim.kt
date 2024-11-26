package frc.robot.subsystems.shooter

import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.lib.motors.TalonFXSim

class ShooterIOSim : ShooterIO {
    override val inputs: LoggedShooterInputs = LoggedShooterInputs()
    var topMotor: TalonFXSim = TalonFXSim(
        1,
        ShooterConstants.GEAR_RATIO_TOP,
        ShooterConstants.MOMENT_OF_INERTIA_TOP.`in`(Units.KilogramSquareMeters),
        1.0
    )
    var bottomMotor: TalonFXSim = TalonFXSim(
        1,
        ShooterConstants.GEAR_RATIO_BOTTOM,
        ShooterConstants.MOMENT_OF_INERTIA_BOTTOM.`in`(Units.KilogramSquareMeters),
        1.0
    )
    private var bottomControlRequest: VelocityVoltage = VelocityVoltage(0.0)
    private var topControlRequest: VelocityVoltage = VelocityVoltage(0.0)

    init {
        topMotor.setController(
            PIDController(
                ShooterConstants.TOP_KP,
                ShooterConstants.TOP_KI,
                ShooterConstants.TOP_KD
            )
        )
        bottomMotor.setController(
            PIDController(
                ShooterConstants.BOTTOM_KP,
                ShooterConstants.BOTTOM_KI,
                ShooterConstants.BOTTOM_KD
            )
        )
    }

    override fun setTopVelocity(velocity: AngularVelocity) {
        topMotor.setControl(topControlRequest.withVelocity(velocity))
    }

    override fun setBottomVelocity(velocity: AngularVelocity) {
        bottomMotor.setControl(bottomControlRequest.withVelocity(velocity))

    }

    override fun updateInput() {
        inputs.topVelocity = Units.RotationsPerSecond.of(topMotor.velocity)
        inputs.topVoltage = Units.Volt.of(bottomMotor.appliedVoltage)
        inputs.bottomVelocity = Units.RotationsPerSecond.of(bottomMotor.velocity)
        inputs.bottomVoltage = Units.Volt.of(topMotor.appliedVoltage)
    }
}