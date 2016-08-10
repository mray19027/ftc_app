package com.qualcomm.ftcrobotcontroller.controllers;

import com.qualcomm.ftcrobotcontroller.containers.Location;
import com.qualcomm.ftcrobotcontroller.opmodes.ARMAuto;
import com.qualcomm.robotcore.robocol.Telemetry;

/**
 * Controller for managing the current state of the robot
 * @author micray01
 * @since 6/16/2016 7:03pm
 * @version 1
 */
public class StateControl {

    //Constants
    private static double FWD_ERR = 0.04;
    private static double TURN_ERR = 0.09;

    public enum State { FWD, TURN, STOP, REVERSE}

    //Lookup table to identify the state-to-state interactions for the robot
    //  This lookup table will drive in a square
    private static State[] state_array = {
            State.FWD,  //1
            State.TURN, //2
            State.FWD,  //3
            State.TURN, //4
            State.TURN, //5
            State.FWD,  //6 - Done with Bin 1
            State.TURN, //7
            State.FWD,  //8
            State.TURN, //9
            State.FWD,  //10
            State.TURN, //11
            State.TURN, //12
            State.FWD,  //13 - Done with Bin 2
            State.TURN, //14
            State.FWD,  //15
            State.STOP};//16

    //Lookup table that provides the details for the state lookup table
    //  This table should be the same size as the state lookup table
    private static Location[] condition_array = {
            new Location(80.25*0.0254,0,0), //1
            new Location(0,0,-Math.PI/2),   //2
            new Location(22.25*0.0254,0,0), //3
            new Location(0,0,-Math.PI/2),   //4
            new Location(0,0,-Math.PI/2),   //5
            new Location(22.25*0.0254,0,0), //6
            new Location(0,0,-Math.PI/2),   //7
            new Location(81.125*0.0254,0,0),//8
            new Location(0,0,-Math.PI/2),   //9
            new Location(22.25*0.0254,0,0), //10
            new Location(0,0,-Math.PI/2),   //11
            new Location(0,0,-Math.PI/2),   //12
            new Location(22.25*0.0254,0,0), //13
            new Location(0,0,-Math.PI/2),   //14
            new Location(50.75*0.0254,0,0), //13
            new Location(0,0,0)};           //15

    private int current_state;

    /**
     * Default constructor
     */
    public StateControl() {
        current_state = 0;
    }

    /**
     * Update the state if necessary based on the current location
     * @param currLocation Current location of the robot
     */
    public void update_state(Location currLocation) {
        //Base check to prevent array size exceptions
        if (current_state>=state_array.length) return;

        //FWD case
        if (state_array[current_state]==State.FWD) {
            ARMAuto.debug.addData("State", "FWD");
            double err = currLocation.distanceTo(condition_array[current_state]);
            ARMAuto.debug.addData("FWD ERR", err);
            //@TODO convert err to a FWD PD control loop for better accuracy
            if (err<=FWD_ERR) {
                currLocation.setX(0);
                currLocation.setY(0);
                currLocation.setTheta(0);
                current_state++;
            }

        //TURN case
        } else if (state_array[current_state]==State.TURN) {
            ARMAuto.debug.addData("State", "TURN");
            double err = currLocation.angleTo(condition_array[current_state]);
            //@TODO convert err to a TURN PD control loop for better accuracy
            if (err <= TURN_ERR) {
                currLocation.setX(0);
                currLocation.setY(0);
                currLocation.setTheta(0);
                current_state++;
            }

        //STOP case
        } else if (state_array[current_state]==State.STOP) {
            ARMAuto.debug.addData("State", "STOP");
            current_state++;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    //                                  Getters and Setters                                       //
    ////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * Get the current state
     * @return State representing the current state
     */
    public State getCurrent_state() {
        if (current_state>=state_array.length) return State.STOP;
        return state_array[current_state];
    }

    /**
     * Get the end condition
     * @return boolean representing if the end condition has been met
     */
    public boolean isEndCondition() {
        return current_state>=state_array.length;
    }
}
