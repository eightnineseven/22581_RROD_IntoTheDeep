package org.firstinspires.ftc.teamcode.RROD.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class PoseHolder {
     double robotWidth = 7;
     double robotLength = 8;
     Vector2d startPos = new Vector2d(0-robotWidth,72-robotLength );
     Vector2d preloadPos = new Vector2d(-7,-36 );
     Vector2d beforePlowPos = new Vector2d(-15,-42 );
     Vector2d beforeSample1Pos = new Vector2d(-48,-7 );
     Vector2d plowSample1Pos = new Vector2d(-48,-60 );
     Vector2d beforeSample2Pos = new Vector2d(-60+robotWidth,-7 );
     Vector2d plowSample2Pos = new Vector2d(-60,-60 );
     Vector2d beforeSample3Pos = new Vector2d(-70+robotWidth,-7 );
     Vector2d plowSample3Pos = new Vector2d(-70,-60);
     Vector2d parkPos = preloadPos;

     public Vector2d startPos(){
          return startPos;
     }

     public Vector2d preloadPos(){
          return preloadPos;
     }
     public Vector2d beforePlowPos(){
          return beforePlowPos;
     }
     public Vector2d beforeSample1Pos(){
          return beforeSample1Pos;
     }
     public Vector2d plowSample1Pos(){
          return plowSample1Pos;
     }
     public Vector2d beforeSample2Pos(){
          return beforeSample2Pos;
     }
     public Vector2d plowSample2Pos(){
          return plowSample2Pos;
     }
     public Vector2d beforeSample3Pos(){
          return beforeSample3Pos;
     }
     public Vector2d plowSample3Pos(){
          return plowSample3Pos;
     }
     public Vector2d parkPos(){
          return parkPos;
     }

}
