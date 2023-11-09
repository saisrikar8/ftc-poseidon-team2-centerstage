package org.firstinspires.ftc.teamcode.autonomous;

public enum BlueTapeMark {
    LEFT(1), CENTER(2), RIGHT(3);

    private int num;
    BlueTapeMark(int i){
        num = i;
    }
    int getValue(){
        return num;
    }
}