package frc.robot.lib

import edu.wpi.first.wpilibj.DataLogManager
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

fun markEvent(eventName: String): Command = Commands.runOnce({ DataLogManager.log(eventName) })