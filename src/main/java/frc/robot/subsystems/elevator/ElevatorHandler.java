package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

public class ElevatorHandler extends SubsystemBase{

    private ElevatorStates currentState = ElevatorStates.STOP;
    private Elevator s_elevator = new Elevator();

    private ElevatorStates[] enumList = {
        ElevatorStates.STOP,
        ElevatorStates.L1,
        ElevatorStates.L2,
        ElevatorStates.L3,
        ElevatorStates.L4,
        ElevatorStates.MAX,
    };

    public ElevatorHandler() {

    }

    public void SetStateUp(){
        int currentIndex = 0;
        for(int i = 0; i <= enumList.length; i++){
            if (enumList[i] == currentState) {
                currentIndex = i;
            }
        }
        if(currentIndex != enumList.length){
            currentState = enumList[currentIndex+1];
        }

        CurrentStateGoal();
    }
    public void SetStateDown(){
        int currentIndex = 0;
        for(int i = 0; i <= enumList.length; i++){
            if (enumList[i] == currentState) {
                currentIndex = i;
            }
        }
        if(currentIndex != 0){
            currentState = enumList[currentIndex-1];
        }

        CurrentStateGoal();
    }

    public void CurrentStateGoal(){
        s_elevator.setState(currentState);
    }
}
