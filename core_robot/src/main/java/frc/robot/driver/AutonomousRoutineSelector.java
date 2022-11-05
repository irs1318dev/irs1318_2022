package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.LoggingKey;
import frc.robot.common.LoggingManager;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.SmartDashboardSelectionManager.AutoRoutine;
import frc.robot.driver.common.*;
import frc.robot.driver.controltasks.*;
import frc.robot.*;

@Singleton
public class AutonomousRoutineSelector
{
    private final ILogger logger;

    private final PathManager pathManager;
    private final SmartDashboardSelectionManager selectionManager;
    private final IDriverStation driverStation;

    /**
     * Initializes a new AutonomousRoutineSelector
     */
    @Inject
    public AutonomousRoutineSelector(
        LoggingManager logger,
        PathManager pathManager,
        SmartDashboardSelectionManager selectionManager,
        IRobotProvider provider)
    {
        this.logger = logger;
        this.pathManager = pathManager;
        this.selectionManager = selectionManager;

        this.driverStation = provider.getDriverStation();

        RoadRunnerTrajectoryGenerator.generateTrajectories(this.pathManager);
    }

    /**
     * Check what routine we want to use and return it
     * @param mode that is starting
     * @return autonomous routine to execute during autonomous mode
     */
    public IControlTask selectRoutine(RobotMode mode)
    {
        String driverStationMessage = this.driverStation.getGameSpecificMessage();
        this.logger.logString(LoggingKey.AutonomousDSMessage, driverStationMessage);
        if (mode == RobotMode.Test)
        {
            return AutonomousRoutineSelector.GetFillerRoutine();
        }

        if (mode != RobotMode.Autonomous)
        {
            return null;
        }

        AutoRoutine routine = this.selectionManager.getSelectedAutoRoutine();

        String autoSelection = routine.toString(); //startPosition.toString() + "." + routine.toString();
        this.logger.logString(LoggingKey.AutonomousSelection, autoSelection);

        if (routine == AutoRoutine.AutoShootDriveBack)
        {
            return autoshootDriveBack();
        }
        else if (routine == AutoRoutine.ShootDriveBack)
        {
            return shootDriveBack();
        }
        else if (routine == AutoRoutine.ShootLowDriveBack)
        {
            return shootLowGoalDriveBack();
        }
        else if (routine == AutoRoutine.WillThreeBallAuto)
        {
            return willThreeBallAuto();
        }
        else if (routine == AutoRoutine.WillTwoBallAuto)
        {
            return willTwoBallAuto();
        }
        else if (routine == AutoRoutine.PravinThreeBallAuto)
        {
            return pravinThreeBallAuto();
        }
        else if (routine == AutoRoutine.PravinFourBallAuto)
        {
            return pravinFourBallAuto();
        }
        else if (routine == AutoRoutine.WinavTwoBallAuto)
        {
            return winavTwoBall();
        }
        else if (routine == AutoRoutine.PraTwoBallAll)
        {
            return praTwoBallAutoAll();
        }
        else if (routine == AutoRoutine.PraTwoBallMid)
        {
            return praTwoBallAutoMid();
        }
        else if (routine == AutoRoutine.PNWThreeBall) 
        {
            return PNWThreeBall();
        }
        else if (routine == AutoRoutine.tester)
        {
            return tester();
        }

        return new PositionStartingTask(0.0, true, true);
    }

    /**
     * Gets an autonomous routine that does nothing
     */
    private static IControlTask GetFillerRoutine()
    {
        return new WaitTask(0.0);
    }

    private static IControlTask autoshootDriveBack()
    {
        return SequentialTask.Sequence(
            new VisionCenteringTask(false),
            new VisionShootPositionTask(),
            ConcurrentTask.AnyTasks(
                new VisionShootSpinTask(10.0, true),
                new CargoShootTask()),
            new FollowPathTask("goBack6ft")
        );
    }

    private static IControlTask shootDriveBack()
    {
        return SequentialTask.Sequence(
            new CargoHoodTask(DigitalOperation.CargoHoodPointBlank),
            new VisionCenteringTask(false, true),
            ConcurrentTask.AnyTasks(
                new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
                new CargoShootTask()
            ),
            new FollowPathTask("goBack6ft")
        );
    }

    private static IControlTask shootLowGoalDriveBack()
    {
        return SequentialTask.Sequence(
            new CargoHoodTask(DigitalOperation.CargoHoodPointBlank),
            ConcurrentTask.AnyTasks(
                new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_LOW_SPINUP_SPEED),
                new CargoShootTask()
            ),
            new FollowPathTask("goBack6ft")
        );
    }

    private static IControlTask willThreeBallAuto()
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(69.0, false, true),
            new CargoHoodTask(DigitalOperation.CargoHoodPointBlank),
            ConcurrentTask.AnyTasks(
                new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
                new CargoShootTask(false)),

            // get second cargo
            ConcurrentTask.AllTasks(
                new FollowPathTask("w3ba-goToPickUpBall2", false, false),
                SequentialTask.Sequence(
                    new WaitTask(1.0),
                    new CargoIntakeTask(2.0, true))),

            // get third cargo
            ConcurrentTask.AllTasks(
                new FollowPathTask("w3ba-goToPickUpBall3", false, false),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new CargoIntakeTask(2.0, true))),

            // auto-align and shoot two cargo
            new FollowPathTask("w3ba-turnToShoot", false, false),
            new VisionCenteringTask(false),
            new VisionShootPositionTask(),
            ConcurrentTask.AnyTasks(
                new VisionShootSpinTask(5.0, true),
                new CargoShootTask()),
            ConcurrentTask.AllTasks(
                new VisionShootSpinTask(6.0, true),
                new CargoShootTask())
                
                );
    }

    private static IControlTask winavTwoBall()
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(69.0, false, true),
            new CargoHoodTask(DigitalOperation.CargoHoodPointBlank),
            ConcurrentTask.AnyTasks(
                new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
                new CargoShootTask(false)),

            // get second cargo
            ConcurrentTask.AllTasks(
                new FollowPathTask("winavGoToBall2Ball", false, false),
                SequentialTask.Sequence(
                    new WaitTask(1.0),
                    new CargoIntakeTask(3.0, true))),

            // auto-align and shoot cargo
            new FollowPathTask("turn180Path"),
            new VisionCenteringTask(false),
            new VisionShootPositionTask(),
            ConcurrentTask.AnyTasks(
                new VisionShootSpinTask(10.0, true),
                new CargoShootTask()));
    }

    private static IControlTask willTwoBallAuto()
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(-21.0, false, true),
            new CargoHoodTask(DigitalOperation.CargoHoodPointBlank),
            ConcurrentTask.AnyTasks(
                new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
                new CargoShootTask(false)),

            // get second cargo
            ConcurrentTask.AllTasks(
                new FollowPathTask("w2ba-goToPickUpBall2", false, false),
                SequentialTask.Sequence(
                    new WaitTask(1.0),
                    new CargoIntakeTask(2.0, true))),

            // auto-align and shoot two cargo
            new FollowPathTask("w2ba-turnToShoot", false, false),
            new VisionCenteringTask(false),
            new VisionShootPositionTask(),
            ConcurrentTask.AnyTasks(
                new VisionShootSpinTask(10.0, true),
                new CargoShootTask()));
    }

    private static IControlTask pravinThreeBallAuto()
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(-20.9, false, true),
            //0 Set hood position
            new CargoHoodTask(DigitalOperation.CargoHoodPointBlank),

            //1 shoot pre-loaded ball
            ConcurrentTask.AnyTasks(
                new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
                new CargoShootTask(false)
            ),
            //2 get first ball
            ConcurrentTask.AllTasks(
                new FollowPathTask("pravinGetFirstBall", false, false),
                SequentialTask.Sequence(
                    new WaitTask(1.0),
                    new CargoIntakeTask(2.0, true)
                )
            ),

            //3 get second ball
            ConcurrentTask.AllTasks(
                new FollowPathTask("pravinGetSecondBall", false, false),
                SequentialTask.Sequence(
                    new WaitTask(1.0),
                    new CargoIntakeTask(2.0, true)
                )
            ),

            //4 shoot two balls
            new FollowPathTask("pravinMoveToShoot", false, false),
            ConcurrentTask.AnyTasks(
                new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
                new CargoShootTask()
            )
        );
    }

    private static IControlTask pravinFourBallAuto()
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(-178.6, false, true),
            //0 Set hood position
            new CargoHoodTask(DigitalOperation.CargoHoodMedium),

            //1 get first ball
            ConcurrentTask.AnyTasks(
                new FollowPathTask("pravinGetFirstBall4"),
                SequentialTask.Sequence(
                    new WaitTask(3.0),
                    new CargoIntakeTask(2.0, true)
                )
            ),
            //2 shoot current balls
            new FollowPathTask("pravinMoveToShootFirstSetBalls", false, false),
            ConcurrentTask.AnyTasks(
                new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_AUTO_FOUR_BALL_SHOOT_SPEED),
                new CargoShootTask()
            ),

            //3 get second ball
            ConcurrentTask.AllTasks(
                new FollowPathTask("pravinGetSecondBall4", false, false),
                SequentialTask.Sequence(
                    new WaitTask(2.0),
                    new CargoIntakeTask(2.0, true)
                )
            ),

            //4 get third ball
            ConcurrentTask.AllTasks(
                new FollowPathTask("pravinGetThirdBall4", false, false),
                SequentialTask.Sequence(
                    new WaitTask(3.0),
                    new CargoIntakeTask(4.0, true)
                )
            ),

            //4 shoot current balls
            new FollowPathTask("pravinMoveToShootSecondSetBalls", false, false),
            ConcurrentTask.AnyTasks(
                new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_AUTO_FOUR_BALL_SHOOT_SPEED),
                new CargoShootTask()
            )
        );
    }

    private static IControlTask praTwoBallAutoMid()
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(-90.0, false, true),
            // get second ball
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    new FollowPathTask("goForwardBrodieMiddle"),
                    new WaitTask(2.0)),
                new CargoIntakeTask(4.0, true)
            ),

            // shoot current balls
            new FollowPathTask("goBackBrodie", true, false),
            new VisionCenteringTask(false),
            new VisionShootPositionTask(),
            ConcurrentTask.AnyTasks(
                new VisionShootSpinTask(10.0, true),
                new CargoShootTask()
            )
        );
    }

    private static IControlTask praTwoBallAutoAll()
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(-90.0, false, true),
            // get second ball
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    new FollowPathTask("goForwardBrodieAll"),
                    new WaitTask(2.0)),
                new CargoIntakeTask(4.0, true)
            ),

            // shoot current balls
            new FollowPathTask("goBackBrodie", true, false),
            new VisionCenteringTask(false),
            new VisionShootPositionTask(),
            ConcurrentTask.AnyTasks(
                new VisionShootSpinTask(7.0, true),
                new CargoShootTask()
            ),
            ConcurrentTask.AnyTasks(
                new VisionShootSpinTask(3.0, true),
                new CargoShootTask()
            )
        );
    }

    private static IControlTask tester()
    {
        return SequentialTask.Sequence(
            new VisionCenteringTask(false),
            new VisionShootPositionTask(),
            ConcurrentTask.AnyTasks(
                new VisionShootSpinTask(5.0, true),
                new CargoShootTask()
                ),
            ConcurrentTask.AnyTasks(
                new VisionShootSpinTask(5.0, true),
                new CargoShootTask()
                )
                );
    }

    private static IControlTask PNWThreeBall()
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(0.0, false, true),
            

            //1 shoot pre-loaded ball
            new CargoHoodTask(DigitalOperation.CargoHoodPointBlank),
            ConcurrentTask.AnyTasks(
                new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
                new CargoShootTask(false)
            ),
            new FollowPathTask("ThreeBallStep1"),
            new PositionStartingTask(0.0, false, true),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    new FollowPathTask("ThreeBallStep2"),
                    new WaitTask(2.5)),
                new CargoIntakeTask(5.0, true)
            ),
            new PositionStartingTask(0.0, false, true),
            new FollowPathTask("ThreeBallStep3"),
            new PositionStartingTask(0.0, false, true),
            new FollowPathTask("ThreeBallStep4"),
            new PositionStartingTask(0.0, false, true),
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    new FollowPathTask("ThreeBallStep5"),
                    new WaitTask(2.5)),
                new CargoIntakeTask(5.0, true)
            ),
            new PositionStartingTask(0.0, false, true),
            new FollowPathTask("ThreeBallStep6"),
            new VisionCenteringTask(false),
            new VisionShootPositionTask(),
            ConcurrentTask.AnyTasks(
                new VisionShootSpinTask(10.0, true),
                new CargoShootTask()
            )
        );
    }
} // yaaaaaAAAaaaAaaaAAAAaa








































































































































/*
                                      .                                                             
                                    .;+;+                                                           
                                    .+;;'   `,+'.                                                   
                                    ;';;+:..`` :+'+                                                 
                                    ,'+`    .+;;;;;+                                                
                                     ;,,, .+;;;;;'+++;                                              
                                     ;' `+;;;;;#+'+'+''#:.                                          
                                     '`+';;;'+;+;+++'''+'.                                          
                                     #';;;;#';+'+'''+''+'                                           
                                     ;;;;#;,+;;+;;;'''''':                                          
                                     ';'++'.`+;;'';;''+'',                                          
                                     :#'#+'``.'+++'#++'':`                                          
                                      `';++##```##+.''.##                                           
                                      +++#   #`#  `++++                                             
                                      +'#+ # :#: # ##'+                                             
                                      `#+#   +`+   #'#`                                             
                                       :,.+,+,`:+,+..,                                              
                                       `,:```,`,`.`;,                                               
                                        :+.;``.``;.#;                                               
                                        .'``'+'+'``'.                                               
                                         ,````````..                                                
                                          :```````:                                                 
                                          +``.:,``'                                                 
                                          :```````:                                                 
                                           +`````+                                                  
                                            ';+##                                                   
                                            '```'                                                   
                                           `'```'`                                                  
                                         .+''''''''                                                 
                                        +;;;;;;;;''#                                                
                                       :       `   `:                                               
                                      `,            '                                               
                                      +              '                                              
                                     ,;';,``.``.,,,:;#                                              
                                     +;;;;;;;;;;;;;;;'                                              
                                    ,';;;;;;;;;;;;;;;',                                             
                                    +:;;;;;;';;;;;;;;;+                                             
                                   `.   .:,;+;;:::;.``,                                             
                                   :`       #,       `.`                                            
                                   +       # ;        .;                                            
                                  .;;,`    ,         `,+                                            
                                  +;;;;;;''';;;;;;;';;';                                            
                                  +;;;;;;;';;;;;;;;;;'';;                                           
                                 `';;;;;;';;;;;;;;;;;';;+                                           
                                 + `:;;;;+;;;;;;;;';'''::                                           
                                 '     `:  ```````    ,  ,                                          
                                :       '             ;  +                                          
                                '`     ..             ,  ,                                          
                               ,;;;;;..+,`        ```.':;',                                         
                               +;;;;;;'+;;;;;;;;;;;;;;+;;;+                                         
                               ';;;;;;++;;;;;;;;;;;;;;';;;+                                         
                              `.:';;;;;#;;;;;;;;;;;;;;';;;;`                                        
                              ;    `,; ',:;;';;';;;;;:;``  +                                        
                              +      ; ;              ;    `                                        
                              ;      : +              '    `;                                       
                              ';:`` `` '              :`,:;;+                                       
                             `';;;;'+  +,..```````..:;#;;;;;;.                                      
                             `;;;;;;+  +;;;;;;;;;;;;;':';;;;;#                                      
                             .;;;;;;+  ';;;;;;;;;;;;;;,';;;;` .                                     
                             : `.;;'+  +;;;;;;;;;;;;;','.`    +                                     
                             '      ;  +.,,;:;:;;;,..`: ,     ``                                    
                             +      ,  '              : ;   .;'+                                    
                             +.`   ``  +              ;  ;:;;;;':                                   
                             ';;;';;`  +             .'  ;;;;;;;+                                   
                             ';;;;;'   :+++#++##+#+''',   +;;;;.`.                                  
                             +;;;;;'   +;;::;;;+:+;;'',   ,;;.   +                                  
                            ``:;;;;+   +;;:;;;:+;+;;++;    +     .`                                 
                             `   ``'   +;;;;;;;+;+;;'+;     ,   ;#,                                 
                            .      ;   ';;;;;;;;;;;;++'     + .+``.;                                
                            ``     ;   ';;;;;;+;';;;'+'      #`````:,                               
                             +++;,:.   ':;''++;:';:;'';      +``````,`                              
                             ,```,+    +;;';:;;+;;;;'';      +``````,+                              
                            .``````:   ;:;;++';;;;;;';,      ,``:#``+`.                             
                            ,``````'   `';;;;:;;;;;;+;`     '+``+:'`..'                             
                            ,``````'    +;;;;;;;;;;;''     ;:'``#;;.`++                             
                            ```````;    `;:;;;;;;;;;;#     ':'``++:+`+;                             
                            ```'`.`;     +;;;;;;;;;;;+    :::#``' +#`';                             
                            ,``'`:`#     `';;;;;;;;;;+    +:'.`,. ++`;;                             
                            +`.``+`'     :#;;;;;;;;;;;`   +:# ,`  +;`.'                             
                           ,.`+`.:.      ##;;;;;;;;;;;'   ,'`     ;:+#                              
                           '`;.`+`#      ##+;;;;;;;;;;+          ,::;                               
                           ,+,`:``,     :###;;;;;;;;;:'          +:;`                               
                            '`,,`+      ';##';;;;;;;;;;.         +:#                                
                             '+.+       +;;##;;;;;;;;;;'         ;:;                                
                               `       :;;;+#;;;;;;;;;;+        ;::`                                
                                       +;;;;#+;;;;;;;;;;        +:'                                 
                                       ';;;;+#;;;;;;;;;;.       ;:'                                 
                                      ,;;;;;;#;;;;;;;;;;+      +::.                                 
                                      +;;;;;;'';;;;;;;;;'      +:+                                  
                                     `;;;;;;;;#;;;;;;;;;;`    `;:+                                  
                                     ,;;;;;;;;+;;;;;;;;;;+    ':;,                                  
                                     +;;;;;;;;;+;;;;;;;;;'    +:+                                   
                                    .;;;;;;;;;+,;;;;;;;;;;`   ;;+                                   
                                    ';;;;;;;;;, ';;;;;;:;;,  +;:,                                   
                                    ';;;;;;;;'  +;;;;;;;;;'  +:+                                    
                                   ;;;;;;;;;;+  ,;;;;;;;;;+  ;:'                                    
                                   +;;;;;;;;;    ';;;;;;;;;`;:;`                                    
                                   ;;;;;;;;;+    +;;;;;;;;;+#:+                                     
                                  ';;;;;;;;;:    ;;;;;;;;;;';:'                                     
                                 `';;;;;;;:'      ';;;;;;;;;;:.                                     
                                 .;;;;;;;;;+      +;;;;;;;;;'+                                      
                                 +;;;;;;;;;       ';;;;;;;;;#+                                      
                                `;;;;;;;;;+       `;;;;;;;;;;`                                      
                                +;;;;;;;;;.        +;;;;;;;;;`                                      
                                ';;;;;;;:'         ;;;;;;;;;;;                                      
                               :;;;;;;;;;:         `;;;;;;;;;+                                      
                               +;;;;;;;;;           ';;;;;;;;;`                                     
                               ;;;;;;;;;+           ';;;;;;;;;:                                     
                              ';;;;;;;;;;           ,;;;;;;;;;+                                     
                              ':;;;;;;;'             +;;;;;;;;;                                     
                             .;:;;;;;;;'             +;;;;;;;;;:                                    
                             +;;;;;;;;;`             .;;;;;;;;;+                                    
                            `;;;;;;;;;+               ;:;;;;;;;;`                                   
                            ;;;;;;;;;;.               +;;;;;;;::.                                   
                            ';;;;;;;;'`               :;;;;;;;;:+                                   
                           :;;;;;;;;:'                ';;;;;;;;;'                                   
                           ';;;;;;;;'`                +#;;;;;;;;;`                                  
                          `;;;;;;;;;+                 '';;;;;;;;;+                                  
                          +;;;;;;;;;.                '::;;;;;;;;;+                                  
                          ;;;;;;;;;+                 #:'';;;;;;;;;`                                 
                         .#;;;;;;;;'                `;:+;;;;;;;;;;;                                 
                         ':'';;;;;;                 '::.,;;;;;;;;;+                                 
                        +::::+';;;+                 ':'  +:;;;;;;;;`                                
                       `;;;::::;#+:                `;:+  +;;;;;;;:;;      '#+,                      
                       +#::::::::;'`               +:;,  `;;;;:;;'#';;;;;::;:'`                     
                      ';:''::::::::#`              +:'    ';:;;+'::;;:;::::::''                     
                      #+::;+':::::::'.            .:;+    '''+;::;:;:::;:::;':'                     
                    `';+';;:;'';:::::':    '      +::.     +:::::::::::::;#;:#                      
                    :+;#'.''##;#;:;;:::'+  #     `+;'      ;:;::::::::;'+;:'+                       
                   '#;+". ` `+:;+:;::;::+'#+     +:;#     ';:::;:+#+';:::+.                        
                   ';#''      ,+::+#';::;+'#+    ';::      #:;;'+';'''++:`                          
                                '':::;'''#+     ,:;;`      #';:;;:+                                 
                                 `:'++;;':       :++       .;;:;;#,                                 
                                       `                    '':``                                   


*/
