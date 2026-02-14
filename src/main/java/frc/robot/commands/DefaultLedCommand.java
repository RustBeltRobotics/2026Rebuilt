package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.LED;

public class DefaultLedCommand extends Command {

    private final LED ledSubsystem;

    public DefaultLedCommand(LED ledSubsystem) {
        this.ledSubsystem = ledSubsystem;
        this.setName("LED Default - Off");
        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        ledSubsystem.changeColor(Color.kBlack); // none/off
    }

}