package org.firstinspires.ftc.teamcode.autonomous;

public enum RedTapeMark {
    LEFT(4), CENTER(5), RIGHT(6)
    ;

    private int num;
    RedTapeMark(int i) {
        num = i;
    }
    int getValue(){
        return num;
    }
}
