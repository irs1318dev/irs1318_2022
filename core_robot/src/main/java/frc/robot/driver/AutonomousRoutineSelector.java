package frc.robot.driver;

import com.google.inject.Inject;
import com.google.inject.Singleton;

import frc.robot.LoggingKey;
import frc.robot.common.LoggingManager;
import frc.robot.common.robotprovider.*;
import frc.robot.driver.common.*;
import frc.robot.driver.controltasks.*;
import frc.robot.*;

@Singleton
public class AutonomousRoutineSelector
{
    private final ILogger logger;

    private final PathManager pathManager;
    private final IDriverStation driverStation;

    // private final ISendableChooser<StartPosition> positionChooser;
    private final ISendableChooser<AutoRoutine> routineChooser;

    // public enum StartPosition
    // {
    //     Center,
    //     Left,
    //     Right
    // }

    public enum AutoRoutine
    {
        None,
        AutoShootDriveBack,
        ShootDriveBack,
        ShootLowDriveBack,
        WillThreeBallAuto,
        WillTwoBallAuto,
        PravinThreeBallAuto,
        PravinFourBallAuto,
        WinavTwoBallAuto,
        PraTwoBallBrodie
    }

    /**
     * Initializes a new AutonomousRoutineSelector
     */
    @Inject
    public AutonomousRoutineSelector(
        LoggingManager logger,
        PathManager pathManager,
        IRobotProvider provider)
    {
        // initialize robot parts that are used to select autonomous routine (e.g. dipswitches) here...
        this.logger = logger;
        this.pathManager = pathManager;

        this.driverStation = provider.getDriverStation();

        INetworkTableProvider networkTableProvider = provider.getNetworkTableProvider();

        this.routineChooser = networkTableProvider.getSendableChooser();
        this.routineChooser.addDefault("None", AutoRoutine.None);
        this.routineChooser.addObject("1 Ball Autoshoot", AutoRoutine.ShootDriveBack);
        this.routineChooser.addObject("1 Ball Point-Blank", AutoRoutine.ShootDriveBack);
        this.routineChooser.addObject("1 Low Ball Point-Blank", AutoRoutine.ShootLowDriveBack);
        this.routineChooser.addObject("Will's 3-Ball Auto", AutoRoutine.WillThreeBallAuto);
        this.routineChooser.addObject("Will's 2-Ball Auto", AutoRoutine.WillTwoBallAuto);
        this.routineChooser.addObject("Pravin's 3-Ball Auto", AutoRoutine.PravinThreeBallAuto);
        this.routineChooser.addObject("Pravin's 4-Ball Auto", AutoRoutine.PravinFourBallAuto);
        this.routineChooser.addObject("Winav's 2-Ball Auto", AutoRoutine.WinavTwoBallAuto);
        this.routineChooser.addObject("Pra's 2-Ball Brodie Auto", AutoRoutine.PraTwoBallBrodie);
        networkTableProvider.addChooser("Auto Routine", this.routineChooser);

        // this.positionChooser = networkTableProvider.getSendableChooser();
        // this.positionChooser.addDefault("center", StartPosition.Center);
        // this.positionChooser.addObject("left", StartPosition.Left);
        // this.positionChooser.addObject("right", StartPosition.Right);
        // networkTableProvider.addChooser("Start Position", this.positionChooser);

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

        // StartPosition startPosition = this.positionChooser.getSelected();
        // if (startPosition == null)
        // {
        //     startPosition = StartPosition.Center;
        // }

        AutoRoutine routine = this.routineChooser.getSelected();
        if (routine == null)
        {
            routine = AutoRoutine.None;
        }

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
        else if (routine == AutoRoutine.PraTwoBallBrodie)
        {
            return praTwoBallBrodieAuto();
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
                    new WaitTask(1.0),
                    new CargoIntakeTask(2.0, true))),

            // auto-align and shoot two cargo
            new FollowPathTask("w3ba-turnToShoot", false, false),
            new VisionCenteringTask(false),
            new VisionShootPositionTask(),
            ConcurrentTask.AnyTasks(
                new VisionShootSpinTask(10.0, true),
                new CargoShootTask()));
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

    private static IControlTask praTwoBallBrodieAuto()
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(-90.0, false, true),
            // get second ball
            ConcurrentTask.AnyTasks(
                SequentialTask.Sequence(
                    new FollowPathTask("goForwardBrodie"),
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
