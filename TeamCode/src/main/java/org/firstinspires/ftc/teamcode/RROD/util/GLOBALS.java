package org.firstinspires.ftc.teamcode.RROD.util;

public class GLOBALS {
    public enum ALLIANCE{
        BLUE,
        RED
    }
    ALLIANCE alliance = ALLIANCE.BLUE;

    public ALLIANCE getAllianceColor(){
            return alliance;
    }
    public void setAllianceColor(ALLIANCE color){
        alliance = color;
    }

}
