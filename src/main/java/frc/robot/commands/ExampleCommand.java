/*package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ExampleCommand extends Command {

    //the command needs the following objects to work properly:
    private SwerveSubsystem swerveSubsystem;
    
    //constructor for the command
    //the constructor takes the subsystems in as parameters
    public ExampleCommand (Object subsystem, Object othersubsystem){
        this.subsystem = subsystem;
       
        //addRequirements says that whatever subsytems you put in as parameters must be used
        //by this command. Prevents two commands that act on the same 
        //subsystem from being called at the same time 
        
        addRequirements(your subsystem name);
    }

    @Override
    // called once when command is first scheduled
    public void initialize() {

    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute(){
        swerveSubsystem.zeroHeading();
        other stuff the method should do
    }
    
    //called once when the command ends
    @Override
    public void end(){

    }
     // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }    
}
*/
