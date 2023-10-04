package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PrintCommand extends CommandBase {
    private final String text;


    public PrintCommand(String text)  {
        this.text = text;
    }

    @Override
    public void execute() {
        System.out.println(text);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
