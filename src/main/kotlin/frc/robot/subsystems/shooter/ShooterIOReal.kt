package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.hardware.TalonFX
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.*
import frc.robot.Ports

class ShooterIOReal : ShooterIO {
    override val topRollerInputs = LoggedRollerInputs()
    override val bottomRollerInputs = LoggedRollerInputs()
    private val topMotor = TalonFX(Ports.Shooter.TOP_MOTOR_ID)
    private val bottomMotor = TalonFX(Ports.Shooter.BOTTOM_MOTOR_ID)
    private val topControl = VelocityVoltage(0.0)
    private val bottomControl = VelocityVoltage(0.0)

    init {
        topMotor.setNeutralMode(NeutralModeValue.Coast)
        bottomMotor.setNeutralMode(NeutralModeValue.Coast)

        topMotor.configurator.apply(ShooterConstants.topMotorConfiguration)
        bottomMotor.configurator.apply(ShooterConstants.bottomMotorConfiguration)
    }

    override fun setTopVelocity(velocity: Measure<Velocity<Angle>>) {
        topMotor.setControl(topControl.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun setBottomVelocity(velocity: Measure<Velocity<Angle>>) {
        bottomMotor.setControl(bottomControl.withVelocity(velocity.`in`(Units.RotationsPerSecond)))
    }

    override fun stop() {
        topMotor.stopMotor()
        bottomMotor.stopMotor()
    }

    override fun updateInputs() {
        topRollerInputs.velocity.mut_replace(
            topMotor.velocity.value, Units.RotationsPerSecond
        )
        topRollerInputs.voltage.mut_replace(topMotor.motorVoltage.value, Units.Volts)

        bottomRollerInputs.velocity.mut_replace(
            bottomMotor.velocity.value, Units.RotationsPerSecond
        )
        bottomRollerInputs.voltage.mut_replace(bottomMotor.velocity.value, Units.Volts)

        if (hasPIDChanged(ShooterConstants.PID_VALUES)) updatePID()
    }

    override fun updatePID() {
        val topSlot0Configs =
            Slot0Configs()
                .withKP(ShooterConstants.TOP_kP.get())
                .withKI(ShooterConstants.TOP_kI.get())
                .withKD(ShooterConstants.TOP_kD.get())
                .withKS(ShooterConstants.TOP_kS.get())
                .withKV(ShooterConstants.TOP_kV.get())
                .withKA(ShooterConstants.TOP_kA.get())

        val bottomSlot0Configs =
            Slot0Configs()
                .withKP(ShooterConstants.BOTTOM_kP.get())
                .withKI(ShooterConstants.BOTTOM_kI.get())
                .withKD(ShooterConstants.BOTTOM_kD.get())
                .withKS(ShooterConstants.BOTTOM_kS.get())
                .withKV(ShooterConstants.BOTTOM_kV.get())
                .withKA(ShooterConstants.BOTTOM_kA.get())

        topMotor.configurator.apply(topSlot0Configs)
        bottomMotor.configurator.apply(bottomSlot0Configs)
    }
}