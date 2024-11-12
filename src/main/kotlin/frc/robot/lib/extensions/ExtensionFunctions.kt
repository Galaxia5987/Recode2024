package frc.robot.lib.extensions

import edu.wpi.first.wpilibj2.command.Command

fun Command.finallyDo(command: Command) = finallyDo(Runnable { command.asProxy().schedule() })