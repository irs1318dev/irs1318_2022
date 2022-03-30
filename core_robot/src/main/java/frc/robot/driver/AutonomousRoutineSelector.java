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
        ShootDriveBack,
        ShootLowDriveBack,
        // TwoBallAuto,
        // FiveBallAutoPog,
        // ThreeBallAuto,
        // ThreeBallCloseAuto,
        WillThreeBallAuto,
        WillTwoBallAuto,
        PravinThreeBallAuto
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
        this.routineChooser.addObject("1 Ball Auto", AutoRoutine.ShootDriveBack);
        this.routineChooser.addObject("1 Low Ball Auto", AutoRoutine.ShootLowDriveBack);
        // this.routineChooser.addObject("2 Ball Auto", AutoRoutine.TwoBallAuto);
        // this.routineChooser.addObject("3 Ball Auto", AutoRoutine.ThreeBallAuto);
        // this.routineChooser.addObject("5 Ball Auto", AutoRoutine.FiveBallAutoPog);
        // this.routineChooser.addObject("3 Ball Close Auto", AutoRoutine.ThreeBallCloseAuto);
        this.routineChooser.addObject("Will's 3 Ball Auto", AutoRoutine.WillThreeBallAuto);
        this.routineChooser.addObject("Will's 2 Ball Auto", AutoRoutine.WillTwoBallAuto);
        this.routineChooser.addObject("Pravin's 3 Ball Auto", AutoRoutine.PravinThreeBallAuto);
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

        if (routine == AutoRoutine.ShootDriveBack)
        {
            return shootDriveBack();
        }
        else if (routine == AutoRoutine.ShootLowDriveBack)
        {
            return shootLowGoalDriveBack();
        }
        // else if (routine == AutoRoutine.ThreeBallAuto)
        // {
        //     return threeBallAutoNotSoPog();
        // }
        // else if (routine == AutoRoutine.FiveBallAutoPog)
        // {
        //     return fiveBallAutoPog();
        // }
        // else if (routine == AutoRoutine.TwoBallAuto)
        // {
        //     return driveBackIntakeDriveForwardShoot();
        // }
        // else if (routine == AutoRoutine.ThreeBallCloseAuto)
        // {
        //     return threeBallAutoStartClose();
        // }
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

        return new PositionStartingTask(0.0, true, true);
    }

    /**
     * Gets an autonomous routine that does nothing
     */
    private static IControlTask GetFillerRoutine()
    {
        return new WaitTask(0.0);
    }

    // private static IControlTask driveBackIntakeDriveForwardShoot()
    // {
    //     return SequentialTask.Sequence(
    //         ConcurrentTask.AnyTasks(
    //             ConcurrentTask.AllTasks(
    //                 new FollowPathTask("goBack3ftRight1Turn4"),
    //                 new CargoExtendIntakeTask(true)
    //             ),
    //             new CargoIntakeTask(2.0, true)
    //         ),
    //         new FollowPathTask("goLeft1ftBack8ftTurn171"),
    //         new VisionCenteringTask(false, true),
    //         ConcurrentTask.AnyTasks(
    //             new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
    //             new CargoShootTask()
    //         )
    //     );
    // }
/*
    private static IControlTask shootDriveBackIntakeShoot()
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

        return SequentialTask.Sequence(
            ConcurrentTask.AnyTasks(
                ConcurrentTask.AllTasks(
                    new FollowPathTask("goBack3ftRight1Turn4"),
                    new CargoExtendIntakeTask(true)
                ),
                new CargoIntakeTask(2.0, true)
            ),
            new FollowPathTask("goLeft1ftBack8ftTurn171"),
            new VisionCenteringTask(false, true),
            ConcurrentTask.AnyTasks(
                new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
                new CargoShootTask()
            )
        );
    }
*/
    // private static IControlTask fiveBallAutoPog()
    // {
    //     return SequentialTask.Sequence(
    //         //0 set hood to long
    //         new CargoHoodTask(DigitalOperation.CargoHoodLong),
    //         //1 move to shooting position
    //         ConcurrentTask.AllTasks(
    //             new FollowPathTask("goForward5Feet"),
    //             new CargoExtendIntakeTask(true)
    //         ),
    //         //2 center with goal
    //         new VisionCenteringTask(false, true),
    //         //3 shoot pre-loaded ball
    //         ConcurrentTask.AnyTasks(
    //             new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_TARMAC_HIGH_SPINUP_SPEED),
    //             new CargoShootTask(false)
    //         ),
    //         //4 get first ball
    //         ConcurrentTask.AllTasks(
    //             new CargoIntakeTask(10.0, true),
    //             new FollowPathTask("goBack5ftLeft3ftTurn180GoBack3ft")
    //         ),
    //         //5 get second ball
    //         ConcurrentTask.AllTasks(
    //             new CargoIntakeTask(10.0, true),
    //             new FollowPathTask("goBack3ftRight5ftTurn122GoBack2ftRight3ft")
    //         ),
    //         //6 shoot the 2 balls
    //         new FollowPathTask("goBack6ftRight5ftTurn122"),
    //         new VisionCenteringTask(false, true),
    //         ConcurrentTask.AllTasks(
    //             new VisionCenteringTask(false, true),
    //             ConcurrentTask.AnyTasks(
    //                 new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_TARMAC_HIGH_SPINUP_SPEED),
    //                 new CargoShootTask())
    //         ),
    //         //7 move to terminal and start intake
    //         ConcurrentTask.AllTasks(
    //             new FollowPathTask("goBack6ftLeft16ftTurn154GoBack3ftLeft1ft"), //split into 2 tasks
    //             new CargoIntakeTask(10.0, true),
    //             SequentialTask.Sequence(
    //                 new WaitTask(2.0),
    //                 new VisionCenteringTask(true, true),
    //                 new WaitTask(1.0),
    //                 new VisionCenteringTask(true, true)
    //             )
    //         ),
    //         //8 move to shoot those balls but not during the month of november
    //         new FollowPathTask("goBack18ftLeft12ftTurn154"),
    //         new VisionCenteringTask(false, true),
    //         ConcurrentTask.AnyTasks(
    //             new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_TARMAC_HIGH_SPINUP_SPEED),
    //             new CargoShootTask()
    //         )
    //     );
    // }

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

    // private static IControlTask threeBallAutoNotSoPog()
    // {
    //     return SequentialTask.Sequence(
    //         //1 move to shooting position
    //         ConcurrentTask.AllTasks(
    //             new FollowPathTask("goForward5Feet"),
    //             new CargoExtendIntakeTask(true)
    //         ),
    //         //2 center with goal
    //         new VisionCenteringTask(false, true),
    //         //3 shoot pre-loaded ball
    //         ConcurrentTask.AnyTasks(
    //             new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_TARMAC_HIGH_SPINUP_SPEED),
    //             new CargoShootTask(false)
    //         ),
    //         //4 get first ball
    //         ConcurrentTask.AllTasks(
    //             new CargoIntakeTask(10.0, true),
    //             new FollowPathTask("goBack5ftLeft3ftTurn180GoBack3ft")
    //         ),
    //         //5 get second ball
    //         ConcurrentTask.AllTasks(
    //             new CargoIntakeTask(10.0, true),
    //             new FollowPathTask("goBack3ftRight5ftTurn122GoBack2ftRight3ft")
    //         ),
    //         //6 shoot the 2 balls
    //         new FollowPathTask("goBack6ftRight5ftTurn122"),
    //         ConcurrentTask.AllTasks(
    //             new VisionCenteringTask(false, true),
    //             ConcurrentTask.AnyTasks(
    //                 new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_TARMAC_HIGH_SPINUP_SPEED),
    //                 new CargoShootTask())
    //         )
    //     );
    // }

    // private static IControlTask threeBallAutoStartClose()
    // {
    //     return SequentialTask.Sequence(
    //         //0 Set hood position
    //         new CargoHoodTask(DigitalOperation.CargoHoodPointBlank),

    //         //1 shoot pre-loaded ball
    //         ConcurrentTask.AnyTasks(
    //             new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
    //             new CargoShootTask(false)
    //         ),
    //         //2 get first ball
    //         ConcurrentTask.AllTasks(
    //             new FollowPathTask("goBack9ftRight2ftTurn164"),
    //             SequentialTask.Sequence(
    //                 new WaitTask(1.0),
    //                 new CargoIntakeTask(4.0, true)
    //             )
    //         ),

    //         //3 get second ball
    //         ConcurrentTask.AllTasks(
    //             new FollowPathTask("goBack4ftRight9ftTurn113"),
    //             SequentialTask.Sequence(
    //                 new WaitTask(1.0),
    //                 new CargoIntakeTask(4.0, true)
    //             )
    //         ),

    //         //4 shoot two balls
    //         new FollowPathTask("goBack7ftRight1ftTurn8"),
    //         new FollowPathTask("goRight8ftTurn90"),
    //         ConcurrentTask.AnyTasks(
    //             new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
    //             new CargoShootTask()
    //         )
    //     );
    // }

    private static IControlTask willThreeBallAuto()
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(67.0, false, true),
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
            new DriveTrainFieldOrientationModeTask(true),
            ConcurrentTask.AnyTasks(
                new VisionShootSpinTask(10.0, true),
                new CargoShootTask()));
    }

    private static IControlTask willTwoBallAuto()
    {
        return SequentialTask.Sequence(
            new PositionStartingTask(-23.0, false, true),
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
            new DriveTrainFieldOrientationModeTask(true),
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
                new FollowPathTask("pravinGetFirstBall"),
                SequentialTask.Sequence(
                    new WaitTask(1.0),
                    new CargoIntakeTask(4.0, true)
                )
            ),

            //3 get second ball
            ConcurrentTask.AllTasks(
                new FollowPathTask("pravinGetSecondBall"),
                SequentialTask.Sequence(
                    new WaitTask(1.0),
                    new CargoIntakeTask(4.0, true)
                )
            ),

            //4 shoot two balls
            new FollowPathTask("pravinMoveToShoot"),
            ConcurrentTask.AnyTasks(
                new CargoSpinupTask(TuningConstants.CARGO_FLYWHEEL_POINT_BLANK_HIGH_SPINUP_SPEED),
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
