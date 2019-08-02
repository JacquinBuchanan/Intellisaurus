/***************************************************
     Quadropewd Walking Dinosour Robot

     copyright Jacquin Buchanan 2018
****************************************************/
#include "DinoRemoteControl.h"
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
MPU6050 mpu6050(Wire);


// Define IR distance sensor model and input pin:
#define IRPin 3

// Choose the IR protocol of your remote. this isi the one shipped in the kit
CNec IRLremote;

#define MIN_PULSE_WIDTH 450
#define MAX_PULSE_WIDTH 2350
#define FREQUENCY 50

#define BACK_LEG_L1 86
#define BACK_LEG_L2 68
#define BACK_LEG_HIP_OFFSET_ANGLE  65.1
#define BACK_LEG_KNEE_OFFSET_ANGLE (-57.2)

#define FRONT_LEG_L1 60
#define FRONT_LEG_L2 48
#define FRONT_LEG_HIP_OFFSET_ANGLE  62
#define FRONT_LEG_KNEE_OFFSET_ANGLE (-64.1)

#define BACK_TO_FRONT_HIP_DIF 43.6

//These are the amount each motor gear is off from true because of the 7 degree gear size;
// these are stored in the EPROM
float MotorBuildOffsets[10] = {0.0, 0.0 , 0.0 , 0.0 , 0.0 , 0.0, 0.0, 0.0, 0.0, 0.0};
int CurrentMotorAdjusting = -1;

// The distance off the ground to hip of a normal walk
#define NORMAL_WALK_HEIGHT 98

// The distance off the ground to raise foot of a normal walk
float FootRaise[5] = {20.0, 20.0, 15.0, 10.0, 10.0};

#define MIN_FOOT_HEIGHT 70
#define MAX_FOOT_HEIGHT 100

#define MIN_FOOT_X -40
#define MAX_FOOT_X 40
#define NORMAL_STEP_LENGTH 55

#define FRONT_STEP_OFFSET_FROM_HIP_FORWARDS 0
#define BACK_STEP_OFFSET_FROM_HIP_FORWARDS 10

#define FRONT_STEP_OFFSET_FROM_HIP_BACKWARDS 15
#define BACK_STEP_OFFSET_FROM_HIP_BACKWARDS 15

int speedIndex = 0;
float StepTime[5] = {1000, 700, 500, 400, 400}; //the time for one foot step forward. (i.e. 1/4 the entire walk cycle)
float RightSideStepLength = NORMAL_STEP_LENGTH; // mm distance that each foot moves forward. NOTE this is differnt for left and right to allow turning.
float LeftSideStepLength = NORMAL_STEP_LENGTH; // mm distance that each foot moves forward. NOTE this is differnt for left and right to allow turning.

boolean leftBackwards = false;
boolean rightBackwards = false;

//four phases to a walk. One for each foot moving forward
// 0 is back right,
// 1 is front right
// 2 is back left
// 3 is front left
int walkphase = 0;

int walkPhaseIndex = 0;

int backwardsWalkPhases[4] = {1,0,3,2};

//The start time for each foots phase
unsigned long  phaseStartTime[4];
float PhaseStartFootXPos[4];
float FootXPos[4] = {0, 0, 0, 0};
float FootYPos[4] = {NORMAL_WALK_HEIGHT, NORMAL_WALK_HEIGHT, NORMAL_WALK_HEIGHT, NORMAL_WALK_HEIGHT};
boolean FootStanding[4];


//mode is used to decide what the motors are doing
// 0 is tweaking motor offsets
// 1 is standing still
// 2 is slow walking
// 3 is starting to walk, (i.e. shift weight)
// 4 is jogging
uint8_t mode;


//PID balancing variables
double last_error_LR = 0.0;
double Kp_LR = 0.02;//0.05;
double Kd_LR = 0.17;//0.17;
double K_PLR = 1.2;
double integrated_error_LR = 0.0;
double Ki_LR = 0.0001;

double last_error_FB = 0.0;
double Kp_FB = 0.02;//0.8;
double Kd_FB = 0.17;//0.27;
double K_PFB = 1.2;
double integrated_error_FB = 0.0;
double Ki_FB = 0.0001;

double GUARD_GAIN = 20.0;

float xBalanceOffset = 0.0;
float yBalanceOffset = 0.0;

#define FIRST_STEP_LEAN_ANGLE 12
#define FIRST_STEP_TAIL_ANGLE 15
#define FIRST_STEP_HEAD_ANGLE 15

float LeftRightShiftAngle[5] = {7.0, 7.0, 4.0, 3.0, 0.0};
float TailShiftAngle[5] = {15.0, 15.0, 5.0, 3.0, 0.0};
float HeadShiftAngle[5] = {7.0, 5.0, 5.0, 3.0, 0.0};


float angle0 = 90;
float angle1 = 90;
float angle2 = 90;
float angle3 = 90;
float angle4 = 90;
float angle5 = 90;
float angle6 = 90;
float angle7 = 90;


#define TAIL_WIRE 8
#define NECK_WIRE 9
#define HEAD_WIRE 10

float TailPosition = 0.0;
float NeckPosition = 0.0;
float HeadPosition = 0.0;

float TailAngle = 90;
float NeckAngle = 90;
float HeadAngle = 90;

#define TAIL_POSITION_MAX (+45)
#define TAIL_POSITION_MIN (-45)

#define NECK_POSITION_MAX  (+15)
#define NECK_POSITION_MIN  (-15)

#define HEAD_POSITION_MAX  (+12)
#define HEAD_POSITION_MIN  (-22)

char MSG_STOP_AND_STAND[] = "*** Stop and stand ";

/***************************************************
    Main Setup function

   *******************************************/
void setup() {

  Wire.begin();

  Serial.begin(9600);
  Serial.println("Walking Dinosour Robot!");

  // Servo controller setup
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);

  // Start reading the remote. PinInterrupt or PinChangeInterrupt* will automatically be selected
  if (!IRLremote.begin(IR_REMOTE_RECIVER_PIN))
    Serial.println(F("You did not choose an infrared remote input pin."));

  pinMode(IRPin, INPUT);


  //read the motor offsets from the EPROM
  if (EEPROM.read(0) == 127)
  {
    for (int i = 0; i < 10; i++) {
      MotorBuildOffsets[i] = ((float)EEPROM.read(i + 1) - 128) / 2;
      //Serial.println(MotorBuildOffsets[i]);
    }
  }

  mpu6050.begin();
  
  float gyroXoffset = 0.0;
  float gyroYoffset = 0.0;

  // the gyrooffset are stored at a specific location above the motor offsets
  if (EEPROM.read(20) == 127)
  {
    gyroXoffset = ((float)EEPROM.read(21) - 128) / 2;
    gyroYoffset = ((float)EEPROM.read(22) - 128) / 2;
  }

  // I assume these offset are based on the mounting of the devcie in the robot
  mpu6050.setGyroOffsets(gyroXoffset, gyroYoffset, 0.0);

  //Start out just standing
  mode = 1;
  CurrentMotorAdjusting = -1;
  StandPose();

}


/***************************************************
    Main Loop function

   *******************************************/
void loop() {

  mpu6050.update();
  
  // Look for some user control
  while ( RemoteControlCodeAvailable())
  {


    int inByte = RemoteControlCodeRead();
    switch (inByte)
    {

      case BUTTON_0 :
      if( mode != 0) {

        //start jogging
        //Serial.println("*** Start jogging forward ");
        
        leftBackwards = false;
        rightBackwards = false;
        mode = 4;
        unsigned long NowTime = millis();
        walkPhaseIndex = 0;
        walkphase = 0;
        speedIndex = 4;
        
        RightSideStepLength = NORMAL_STEP_LENGTH;
        LeftSideStepLength = NORMAL_STEP_LENGTH;

       for (int FootID = 0; FootID < 4; FootID++)
        {
          phaseStartTime[FootID] = NowTime;
        }
  
        
        PhaseStartFootXPos[0] = FootXPos[0];
        PhaseStartFootXPos[1] = FootXPos[1];
        PhaseStartFootXPos[2] = FootXPos[2];
        PhaseStartFootXPos[3] = FootXPos[3];
        
        break;
      }
      case BUTTON_1 :
      if (mode == 0)
        {
          CurrentMotorAdjusting = inByte - BUTTON_0;

        } else {
          TailPosition += 10;
          MoveTail();
        }
        break;
        
      case BUTTON_2 :

        if (mode == 0)
        {
          CurrentMotorAdjusting = inByte - BUTTON_0;

        } else {
          
          HeadPosition += 10;
          MoveHead();
          
        }
        break;
        
      case BUTTON_3 :
        if (mode == 0)
        {
          CurrentMotorAdjusting = inByte - BUTTON_0;


        }
        break;
        
      case BUTTON_4 :

        if (mode == 0)
        {
          CurrentMotorAdjusting = inByte - BUTTON_0;


        } else {

          
          NeckPosition += 10;
          MoveNeck();
        }
        break;
        
      case BUTTON_5 :

      if (mode == 0)
        {
          CurrentMotorAdjusting = inByte - BUTTON_0;


        } else {
          TailPosition = 0.0;
          NeckPosition = 0.0;
          HeadPosition = 0.0;
          
          MoveTail();
          MoveNeck();
          MoveHead();
        }
        break;
        
      case BUTTON_6 :
      
        if (mode == 0)
        {
          CurrentMotorAdjusting = inByte - BUTTON_0;


        } else {
          
          NeckPosition -= 10;
          MoveNeck();
        }
        break;
        
      case BUTTON_7 :
        if (mode == 0)
        {
          CurrentMotorAdjusting = inByte - BUTTON_0;


        } else {
          TailPosition -= 10;
          MoveTail();
        }
        
        break;

        case BUTTON_8 :
       if (mode == 0)
        {
          CurrentMotorAdjusting = inByte - BUTTON_0;


        } else { 
        
          
          HeadPosition -= 10;
          MoveHead();
        }
        break;

        
        case BUTTON_9 :
       if (mode == 0)
        {
          CurrentMotorAdjusting = inByte - BUTTON_0;


        }
        break;


      case BUTTON_UP:
        if (mode == 0)
        {
          if (CurrentMotorAdjusting >= 0) {
            MotorBuildOffsets[CurrentMotorAdjusting] += 0.5;

            EEPROM.write(0, 127);
            for (int i = 0; i < 10; i++)
            {
              EEPROM.write(i + 1, (byte)((int)(MotorBuildOffsets[i] * 2) + 128));
            }

            Serial.print("CurrentMotorAdjusting ");
            Serial.print(CurrentMotorAdjusting);
            Serial.print("\t v : ");
            Serial.println(MotorBuildOffsets[CurrentMotorAdjusting]);

            //Recalculate the pose to move the motor
            StandPose();
          }
        } else {

          if (mode == 2 || mode == 4) {

            if(leftBackwards && rightBackwards)
            {

              if (mode == 4)
              {
                // come out of jog and into walk
                speedIndex = 3;
                mode = 2;
                  
                
              } else {
                
              
                // We are walking backwards so slow down
                // Slow down
                speedIndex -= 1;
    
                if (speedIndex < 0)
                {
    
                  speedIndex = 0;
    
                  // Stop and stand
                  mode = 1;
                  StandPose();
    
                  Serial.println(MSG_STOP_AND_STAND);
                }
              }
              Serial.print("*** Walk backwards slower "); Serial.println(speedIndex);
              
            } else {
                // We are already walking forwards so walk faster
                speedIndex += 1;
                if (speedIndex >= 5)
                  {
                    speedIndex = 4;
                  }

                 if(speedIndex == 4)
                  {
                    //speed index 4 is a jog
                    mode = 4;
                  }
                Serial.print("*** Walk forwards faster "); Serial.println(speedIndex);

            }
              
          } else {
            // Start walking forward

            Serial.println("*** Start walking forward ");
            
            leftBackwards = false;
            rightBackwards = false;
            mode = 3;
            unsigned long NowTime = millis();
            walkPhaseIndex = 0;
            speedIndex = 0;
            phaseStartTime[0] = NowTime;

          }

          RightSideStepLength = NORMAL_STEP_LENGTH;
          LeftSideStepLength = NORMAL_STEP_LENGTH;

        }
        break;

      case BUTTON_DOWN:

        if (mode == 0)
        {
          if (CurrentMotorAdjusting >= 0) {
            MotorBuildOffsets[CurrentMotorAdjusting] -= 0.5;

            EEPROM.write(0, 127);
            for (int i = 0; i < 10; i++)
              EEPROM.write(i + 1, (byte)((int)(MotorBuildOffsets[i] * 2) + 128));

            Serial.print("CurrentMotorAdjusting ");
            Serial.print(CurrentMotorAdjusting);
            Serial.print("\t v : ");
            Serial.println(MotorBuildOffsets[CurrentMotorAdjusting]);

            //Recalculate the pose to move the motor
            StandPose();
          }
        } else {


          if (mode == 2 || mode == 4) {

            if(leftBackwards && rightBackwards)
            {
              // We are already walking backwards so speed up
                speedIndex += 1;
                if (speedIndex >= 5)
                  {
                    speedIndex = 4;
                  }

                if(speedIndex == 4)
                  {
                    //speed index 4 is a jog
                    mode = 4;
                  }
                Serial.print("*** Walk backwards faster "); Serial.println(speedIndex);
            } else {
              if (mode == 4)
              {
                // come out of jog and into walk
                speedIndex = 3;
                mode = 2;
                  
                
              } else {

                // We are walking forwards so Slow down
                speedIndex -= 1;
    
                if (speedIndex < 0) {
    
                  speedIndex = 0;
    
                  // Stop and stand
                  mode = 1;
                  StandPose();
    
                  Serial.println(MSG_STOP_AND_STAND);
                }
              }
              Serial.print("*** Walk forwads slower "); Serial.println(speedIndex);
            }
          } else {
            
              // Start walking backwards
              Serial.println("*** Start walking backwards ");
  
              mode = 3;
              unsigned long NowTime = millis();
              walkPhaseIndex = 0;
              speedIndex = 0;
              phaseStartTime[0] = NowTime;
              leftBackwards = true;
              rightBackwards = true;
            
          }

          RightSideStepLength = NORMAL_STEP_LENGTH;
          LeftSideStepLength = NORMAL_STEP_LENGTH;

        }
        break;


      case BUTTON_RIGHT:
        // Turn Right
        Serial.println("*** Turn right ");

        if (mode == 2 || mode == 3 || mode == 4) {
          if(LeftSideStepLength < NORMAL_STEP_LENGTH)
          {
            LeftSideStepLength += 10;
            LeftSideStepLength = constrain(LeftSideStepLength, 20.0 , NORMAL_STEP_LENGTH);
            RightSideStepLength = NORMAL_STEP_LENGTH;
          }else {
            RightSideStepLength -= 10;
            RightSideStepLength = constrain(RightSideStepLength, 20.0 , NORMAL_STEP_LENGTH);
            LeftSideStepLength = NORMAL_STEP_LENGTH;
          }
        } else {
            
            CurrentMotorAdjusting = -1;

            leftBackwards = false;
            rightBackwards = true;

            //very short steps for turning in place
             RightSideStepLength = 2*NORMAL_STEP_LENGTH/3;
             LeftSideStepLength = NORMAL_STEP_LENGTH;

            speedIndex = 3;
            mode = 3;
            walkPhaseIndex = 0;
            unsigned long NowTime = millis();
            for (int FootID = 0; FootID < 4; FootID++)
            {
              //This is wrong for intial step but fix it later
              phaseStartTime[FootID] = NowTime - (FootID * (StepTime[speedIndex] / 3));
            }
        }
        break;

      case BUTTON_LEFT:
        // Turn left
        Serial.println("*** Turn left ");

        if (mode == 2|| mode == 3 || mode == 4) {
          if(RightSideStepLength < NORMAL_STEP_LENGTH)
          {
              RightSideStepLength += 10;
              RightSideStepLength = constrain(RightSideStepLength, 20.0 , NORMAL_STEP_LENGTH);
              LeftSideStepLength = NORMAL_STEP_LENGTH;
          } else {
            LeftSideStepLength -= 10;
            LeftSideStepLength = constrain(LeftSideStepLength, 20.0 , NORMAL_STEP_LENGTH);
            RightSideStepLength = NORMAL_STEP_LENGTH;
          }
        } else {

            CurrentMotorAdjusting = -1;
            leftBackwards = true;
            rightBackwards = false;
            
            //very short steps for turning in place
            RightSideStepLength = NORMAL_STEP_LENGTH;
            LeftSideStepLength = 2*NORMAL_STEP_LENGTH/3;

            speedIndex = 3;

            mode = 3;
            walkPhaseIndex = 0;
            unsigned long NowTime = millis();
            for (int FootID = 0; FootID < 4; FootID++)
            {
              //This is wrong for intial step but fix it later
              phaseStartTime[FootID] = NowTime - (FootID * (StepTime[speedIndex] / 3));
            }
        }
        break;

     case BUTTON_STAR:

       if(mode == 0)
         {
            Serial.println("*** Calibrating GPU. Make sure robot is standing on flat ground. ");
            mpu6050.calcGyroOffsets(true);
  
            EEPROM.write(20, 127);
            EEPROM.write(21, (byte)((int)(mpu6050.getGyroXoffset() * 2) + 128));
            EEPROM.write(22, (byte)((int)(mpu6050.getGyroYoffset() * 2) + 128));
      
         } else {
            
            Serial.println("*** Go Straight ");
    
            if (mode == 2|| mode == 3 || mode == 4) {
              LeftSideStepLength = NORMAL_STEP_LENGTH;
              RightSideStepLength = NORMAL_STEP_LENGTH;
              
            }
       }

        break;


      case BUTTON_CENTER :
        // Stop and stand
        mode = 1;
        CurrentMotorAdjusting = -1;

        StandPose();

        Serial.println(MSG_STOP_AND_STAND);

        break;


      case BUTTON_HASH :
        // Motor setup
        mode = 0;
        CurrentMotorAdjusting = -1;

        StandPose();
        Serial.println("*** Motor Adjusting Offset Mode");
        break;

    }
  }

  if (mode == 1)
  {

    xBalanceOffset = 0.0;
    
    // Use the forward IR sensor to see if we are on ground
    int NoGround = digitalRead(IRPin);

    if(NoGround == 1)
    { 
      //We arn't on the ground so just stand straight  
      StandPose();
    } else {
      LeftRightBalance(-1, 0.0);
      FrontBackBalance(-1, -1);
    }


    FootYPos[0] = constrain(FootYPos[0], MIN_FOOT_HEIGHT , MAX_FOOT_HEIGHT);
    FootYPos[1] = constrain(FootYPos[1], MIN_FOOT_HEIGHT , MAX_FOOT_HEIGHT);
    FootYPos[2] = constrain(FootYPos[2], MIN_FOOT_HEIGHT , MAX_FOOT_HEIGHT);
    FootYPos[3] = constrain(FootYPos[3], MIN_FOOT_HEIGHT , MAX_FOOT_HEIGHT);

    
    FootXPos[0] = constrain(FootXPos[0], MIN_FOOT_X , MAX_FOOT_X);
    FootXPos[1] = constrain(FootXPos[1], MIN_FOOT_X , MAX_FOOT_X);
    FootXPos[2] = constrain(FootXPos[2], MIN_FOOT_X , MAX_FOOT_X);
    FootXPos[3] = constrain(FootXPos[3], MIN_FOOT_X , MAX_FOOT_X);

    MoveLegTo(FootXPos[0], FootYPos[0] + BACK_TO_FRONT_HIP_DIF, 0);
    MoveLegTo(FootXPos[2], FootYPos[2] + BACK_TO_FRONT_HIP_DIF, 2);
    MoveLegTo(FootXPos[1], FootYPos[1], 1);
    MoveLegTo(FootXPos[3], FootYPos[3], 3);

  }



  if (mode == 3)
  {
    //shift weight to start walking
    unsigned long NowTime = millis();

    if(NowTime > phaseStartTime[0] + StepTime[speedIndex]/2) {

      //Really Start walking
      mode = 2;
      walkPhaseIndex = 0;
      walkphase = 0;
      
      if(rightBackwards && leftBackwards)
        walkphase = backwardsWalkPhases[walkPhaseIndex];
  
      for (int FootID = 0; FootID < 4; FootID++)
      {
        //This is wrong for intial step but fix it later
        if(rightBackwards && leftBackwards)
          phaseStartTime[backwardsWalkPhases[FootID]] = NowTime - (FootID * (StepTime[speedIndex] / 3));
        else
          phaseStartTime[FootID] = NowTime - (FootID * (StepTime[speedIndex] / 3));

        FootStanding[FootID] = true;
      }
  
  
      PhaseStartFootXPos[1] = FootXPos[1] = 0;
      PhaseStartFootXPos[3] = FootXPos[3] = 0;

    if(rightBackwards || leftBackwards)
        {
          //If we are walking backwards then slide the back feet forward a little to get started
          FootXPos[0] = BACK_STEP_OFFSET_FROM_HIP_BACKWARDS*2;
          FootXPos[1] = FRONT_STEP_OFFSET_FROM_HIP_BACKWARDS;
          FootXPos[2] = BACK_STEP_OFFSET_FROM_HIP_BACKWARDS*2;
          FootXPos[3] = FRONT_STEP_OFFSET_FROM_HIP_BACKWARDS;
        }

  
    } else {

        float WeightshiftPercentage =  (float)(NowTime - (phaseStartTime[0])) / (StepTime[speedIndex]/2);
    
        WeightshiftPercentage = constrain(WeightshiftPercentage, 0.0 , 1.0);

        xBalanceOffset = GetRelativeValue( 0, FIRST_STEP_LEAN_ANGLE, WeightshiftPercentage);

        TailPosition = GetRelativeValue( 0.0, FIRST_STEP_TAIL_ANGLE, WeightshiftPercentage);
        NeckPosition = GetRelativeValue( 0.0, FIRST_STEP_HEAD_ANGLE, WeightshiftPercentage);
        MoveTail();
        MoveNeck();

        if(rightBackwards || leftBackwards)
        {
          //If we are walking backwards then slide the back feet forward a little to get started
          FootXPos[0] = GetRelativeValue( 0, BACK_STEP_OFFSET_FROM_HIP_BACKWARDS*2, WeightshiftPercentage); //RightSideStepLength / 2 + BACK_STEP_OFFSET_FROM_HIP_BACKWARDS, WeightshiftPercentage);
          FootXPos[1] = GetRelativeValue( 1, FRONT_STEP_OFFSET_FROM_HIP_BACKWARDS, WeightshiftPercentage);
          FootXPos[2] = GetRelativeValue( 2, BACK_STEP_OFFSET_FROM_HIP_BACKWARDS*2, WeightshiftPercentage); // LeftSideStepLength / 2 + BACK_STEP_OFFSET_FROM_HIP_BACKWARDS, WeightshiftPercentage);
          FootXPos[3] = GetRelativeValue( 3, FRONT_STEP_OFFSET_FROM_HIP_BACKWARDS, WeightshiftPercentage);
        }

        MoveLegsForWalk( 1.0);
    }

    //Serial.print(xBalanceOffset);Serial.print(" ***PHASE - ");Serial.println(walkphase);
  }

 // Use the forward IR sensor to see if we are about to walk off a cliff.
 int NoGround = digitalRead(IRPin);
        
 if (mode == 2 || mode == 4)
  {
    //this is a precheck for mode 2 walking forward
    if(!(rightBackwards && leftBackwards))
    {

      if( NoGround == 1) 
        {
          // Stop and stand
          mode = 1;
          CurrentMotorAdjusting = -1;
          StandPose();
          Serial.println("*** Stop and stand *** Cliff or no ground  ");
        }
       
      }
    
  }
  
  if (mode == 2)
  {
    //This is the true walking mode
    // One leg off the ground at a time.
    
    boolean HighStep = true;
    unsigned long NowTime = millis();

    unsigned long WalkPhaseExtraTime = StepTime[speedIndex]/4;

    float LeftRightOffsetAngle = LeftRightShiftAngle[speedIndex];
    float TailOffsetAngle = TailShiftAngle[speedIndex];
    float HeadOffsetAngle = HeadShiftAngle[speedIndex];
    
   if(FootStanding[0] || FootStanding[1] || FootStanding[2] || FootStanding[3] )//||HighStep)
    {

      LeftRightOffsetAngle = FIRST_STEP_LEAN_ANGLE;
      TailOffsetAngle = FIRST_STEP_TAIL_ANGLE;
      HeadOffsetAngle = FIRST_STEP_HEAD_ANGLE;
    }
    
    if (NowTime > phaseStartTime[walkphase] + StepTime[speedIndex]) {

      phaseStartTime[walkphase] = NowTime;
      PhaseStartFootXPos[walkphase] = FootXPos[walkphase];

      walkPhaseIndex++;
      if (walkPhaseIndex > 3) walkPhaseIndex = 0;

      walkphase = walkPhaseIndex;

      if(rightBackwards && leftBackwards)
        walkphase = backwardsWalkPhases[walkPhaseIndex];

      phaseStartTime[walkphase] = NowTime;
      PhaseStartFootXPos[walkphase] = FootXPos[walkphase];

      //Serial.print(" ***PHASE - ");Serial.println(walkphase);

    }


    float StepPercentage = (float)(NowTime - phaseStartTime[walkphase]) / (StepTime[speedIndex] - WalkPhaseExtraTime);
    if(StepPercentage > 1.0) StepPercentage = 1.0;
    
    float footHeight =  FootRaise[speedIndex] * (-1 *  pow((StepPercentage*2) - 1, 2) +1);
    //FootRaise[speedIndex] * (1 - pow(40 , StepPercentage) / 40);
    FootYPos[walkphase] = NORMAL_WALK_HEIGHT - footHeight;


    float WeightshiftPercentage = 0.0;

    //if (NowTime > phaseStartTime[walkphase] + WalkPhaseExtraTime)
    WeightshiftPercentage = (float)(NowTime - (phaseStartTime[walkphase])) / StepTime[speedIndex];

    WeightshiftPercentage = constrain(WeightshiftPercentage, 0.0 , 1.0);

    float weightshift =  (pow(40 , WeightshiftPercentage) / 40);



    float DirectionalRightSideStepLength = RightSideStepLength;
    if(rightBackwards)
        DirectionalRightSideStepLength = -RightSideStepLength;

    float DirectionalLeftSideStepLength = LeftSideStepLength;
    if(leftBackwards)
        DirectionalLeftSideStepLength = -LeftSideStepLength;

    switch (walkphase) {

      case 0:

        if(rightBackwards)
          {
          xBalanceOffset = GetRelativeValue( LeftRightOffsetAngle, -LeftRightOffsetAngle, weightshift);
          
          TailPosition = GetRelativeValue(TailOffsetAngle, -TailOffsetAngle, weightshift);
          NeckPosition = GetRelativeValue(HeadOffsetAngle, -HeadOffsetAngle, weightshift);
          MoveTail();
          MoveNeck();
          }
        else
          xBalanceOffset = LeftRightOffsetAngle;

        // 0 is back right is lifted
        //calculate x as a function of time
        SetFootXPosition( 0, PhaseStartFootXPos[0], DirectionalRightSideStepLength / 2 , WalkPhaseExtraTime);
        SetFootXPosition( 1, PhaseStartFootXPos[1], DirectionalRightSideStepLength / 2 , WalkPhaseExtraTime);
        SetFootXPosition( 2, PhaseStartFootXPos[2], DirectionalLeftSideStepLength / 2 , WalkPhaseExtraTime);
        SetFootXPosition( 3, PhaseStartFootXPos[3], DirectionalLeftSideStepLength / 2 , WalkPhaseExtraTime);


        break;


      case 1:

        if(rightBackwards)
          xBalanceOffset = LeftRightOffsetAngle;
        else {
          xBalanceOffset = GetRelativeValue( LeftRightOffsetAngle, -LeftRightOffsetAngle, weightshift);

          TailPosition = GetRelativeValue(TailOffsetAngle, -TailOffsetAngle, weightshift);
          NeckPosition = GetRelativeValue(HeadOffsetAngle, -HeadOffsetAngle, weightshift);
          MoveTail();
          MoveNeck();
        }

        // 1 is front right is lifted
        //calculate x as a function of time
        SetFootXPosition( 0, PhaseStartFootXPos[0], DirectionalRightSideStepLength / 2 , WalkPhaseExtraTime);
        SetFootXPosition( 1, PhaseStartFootXPos[1], DirectionalRightSideStepLength / 2 , WalkPhaseExtraTime);
        SetFootXPosition( 2, PhaseStartFootXPos[2], DirectionalLeftSideStepLength / 2 , WalkPhaseExtraTime);
        SetFootXPosition( 3, PhaseStartFootXPos[3], DirectionalLeftSideStepLength / 2 , WalkPhaseExtraTime);

        break;


      case 2:
        // 2 is back left is lifted
        if(leftBackwards)
        {
          xBalanceOffset = GetRelativeValue( -LeftRightOffsetAngle, LeftRightOffsetAngle, weightshift);
          TailPosition = GetRelativeValue(-TailOffsetAngle, TailOffsetAngle, weightshift);
          NeckPosition = GetRelativeValue(-HeadOffsetAngle, HeadOffsetAngle, weightshift);
          MoveTail();
          MoveNeck();
        } else
          xBalanceOffset = -LeftRightOffsetAngle;

        //calculate x as a function of time
        SetFootXPosition( 0, PhaseStartFootXPos[0], DirectionalRightSideStepLength / 2 , WalkPhaseExtraTime);
        SetFootXPosition( 1, PhaseStartFootXPos[1], DirectionalRightSideStepLength / 2 , WalkPhaseExtraTime);
        SetFootXPosition( 2, PhaseStartFootXPos[2], DirectionalLeftSideStepLength / 2 , WalkPhaseExtraTime);
        SetFootXPosition( 3, PhaseStartFootXPos[3], DirectionalLeftSideStepLength / 2 , WalkPhaseExtraTime);

        break;


      case 3:
        // 3 is front left is lifted

        if(leftBackwards)
           xBalanceOffset = -LeftRightOffsetAngle;
        else {
          xBalanceOffset = GetRelativeValue( -LeftRightOffsetAngle, LeftRightOffsetAngle, weightshift);
          TailPosition = GetRelativeValue(-TailOffsetAngle, TailOffsetAngle, weightshift);
          NeckPosition = GetRelativeValue(-HeadOffsetAngle, HeadOffsetAngle, weightshift);
          MoveTail();
          MoveNeck();
        }
        

        //calculate x as a function of time
        SetFootXPosition( 0, PhaseStartFootXPos[0], DirectionalRightSideStepLength / 2, WalkPhaseExtraTime);
        SetFootXPosition( 1, PhaseStartFootXPos[1], DirectionalRightSideStepLength / 2, WalkPhaseExtraTime);
        SetFootXPosition( 2, PhaseStartFootXPos[2], DirectionalLeftSideStepLength / 2, WalkPhaseExtraTime);
        SetFootXPosition( 3, PhaseStartFootXPos[3], DirectionalLeftSideStepLength / 2, WalkPhaseExtraTime);

        break;


    }

    MoveLegsForWalk( StepPercentage);

  }

  if (mode == 4)
  {
    // This is jogging mode, move two legs at a time

    unsigned long NowTime = millis();

    float LeftRightOffsetAngle = LeftRightShiftAngle[speedIndex];
    
    if (NowTime > phaseStartTime[walkphase] + StepTime[speedIndex]) 
    {

      if (walkphase == 0) 
        walkphase = 1;
        else
        walkphase = 0;

      for (int FootID = 0; FootID < 4; FootID++)
        {
          phaseStartTime[FootID] = NowTime;
          PhaseStartFootXPos[FootID] = FootXPos[FootID];
        }

    }


    float StepPercentage = (float)(NowTime - phaseStartTime[walkphase]) / StepTime[speedIndex] ;
    if(StepPercentage > 1.0) StepPercentage = 1.0;
    
    float footHeight =  FootRaise[speedIndex] * (-1 *  pow((StepPercentage*2) - 1, 2) +1);
    
    //NOTE: there is no weight shift when jogging on four legs
    xBalanceOffset = 0.0;
    TailPosition = 0.0;
    MoveTail();

    float DirectionalRightSideStepLength = RightSideStepLength;
    if(rightBackwards)
        DirectionalRightSideStepLength = -RightSideStepLength;

    float DirectionalLeftSideStepLength = LeftSideStepLength;
    if(leftBackwards)
        DirectionalLeftSideStepLength = -LeftSideStepLength;

    float FrontDirectionalHipOffset = FRONT_STEP_OFFSET_FROM_HIP_FORWARDS;
    
    float BackDirectionalHipOffset = BACK_STEP_OFFSET_FROM_HIP_FORWARDS;
    
    if(rightBackwards || leftBackwards){
        FrontDirectionalHipOffset = FRONT_STEP_OFFSET_FROM_HIP_BACKWARDS;
        BackDirectionalHipOffset = BACK_STEP_OFFSET_FROM_HIP_BACKWARDS;
      }

    switch (walkphase) {

      case 0:

        // 0 is back right and front left are lifted
        //calculate x as a function of time
        FootXPos[0] = GetRelativeValue( PhaseStartFootXPos[0],   DirectionalRightSideStepLength / 2 + BackDirectionalHipOffset, StepPercentage);
        FootXPos[1] = GetRelativeValue( PhaseStartFootXPos[1], - DirectionalRightSideStepLength / 2 + FrontDirectionalHipOffset, StepPercentage);
        FootXPos[2] = GetRelativeValue( PhaseStartFootXPos[2], - DirectionalLeftSideStepLength / 2 + BackDirectionalHipOffset, StepPercentage);
        FootXPos[3] = GetRelativeValue( PhaseStartFootXPos[3],   DirectionalLeftSideStepLength / 2 + FrontDirectionalHipOffset, StepPercentage);

        FootYPos[0] = NORMAL_WALK_HEIGHT - footHeight;
        FootYPos[1] = NORMAL_WALK_HEIGHT;
        FootYPos[2] = NORMAL_WALK_HEIGHT;
        FootYPos[3] = NORMAL_WALK_HEIGHT - footHeight;

        //FrontBackBalance(0,3);

        break;


      case 1:

        // 1 is front right and back left are lifted
        //calculate x as a function of time
        FootXPos[0] = GetRelativeValue( PhaseStartFootXPos[0], - DirectionalRightSideStepLength / 2 + BackDirectionalHipOffset, StepPercentage);
        FootXPos[1] = GetRelativeValue( PhaseStartFootXPos[1],   DirectionalRightSideStepLength / 2 + FrontDirectionalHipOffset, StepPercentage);
        FootXPos[2] = GetRelativeValue( PhaseStartFootXPos[2],   DirectionalLeftSideStepLength / 2 + BackDirectionalHipOffset, StepPercentage);
        FootXPos[3] = GetRelativeValue( PhaseStartFootXPos[3], - DirectionalLeftSideStepLength / 2 + FrontDirectionalHipOffset, StepPercentage);

        FootYPos[0] = NORMAL_WALK_HEIGHT;
        FootYPos[1] = NORMAL_WALK_HEIGHT - footHeight;
        FootYPos[2] = NORMAL_WALK_HEIGHT - footHeight;
        FootYPos[3] = NORMAL_WALK_HEIGHT;
        
        //FrontBackBalance(1,2);

        break;
    }
    

    FootXPos[0] = constrain(FootXPos[0], MIN_FOOT_X , MAX_FOOT_X);
    FootXPos[1] = constrain(FootXPos[1], MIN_FOOT_X , MAX_FOOT_X);
    FootXPos[2] = constrain(FootXPos[2], MIN_FOOT_X , MAX_FOOT_X);
    FootXPos[3] = constrain(FootXPos[3], MIN_FOOT_X , MAX_FOOT_X);

    FootYPos[0] = constrain(FootYPos[0], MIN_FOOT_HEIGHT , MAX_FOOT_HEIGHT);
    FootYPos[1] = constrain(FootYPos[1], MIN_FOOT_HEIGHT , MAX_FOOT_HEIGHT);
    FootYPos[2] = constrain(FootYPos[2], MIN_FOOT_HEIGHT , MAX_FOOT_HEIGHT);
    FootYPos[3] = constrain(FootYPos[3], MIN_FOOT_HEIGHT , MAX_FOOT_HEIGHT);

    MoveLegTo(FootXPos[0], FootYPos[0] + BACK_TO_FRONT_HIP_DIF, 0);
    MoveLegTo(FootXPos[2], FootYPos[2] + BACK_TO_FRONT_HIP_DIF, 2);
    MoveLegTo(FootXPos[1], FootYPos[1], 1);
    MoveLegTo(FootXPos[3], FootYPos[3], 3);

  }



  SetServoAngle(0, angle0);
  SetServoAngle(1, angle1);
  SetServoAngle(2, angle2);
  SetServoAngle(3, angle3);
  SetServoAngle(4, angle4);
  SetServoAngle(5, angle5);
  SetServoAngle(6, angle6);
  SetServoAngle(7, angle7);

  SetServoAngle(TAIL_WIRE, TailAngle );
  SetServoAngle(NECK_WIRE, NeckAngle);
  SetServoAngle(HEAD_WIRE, HeadAngle);


  //delay(25);

}


/***************************************************
  MoveLegsForWalk()
************************************************* */
void MoveLegsForWalk(float StepPercentage)
{

    LeftRightBalance(StepPercentage >= 1.0 ? -1: walkphase, xBalanceOffset);
    FrontBackBalance(walkphase, -1);

    FootXPos[0] = constrain(FootXPos[0], MIN_FOOT_X , MAX_FOOT_X);
    FootXPos[1] = constrain(FootXPos[1], MIN_FOOT_X , MAX_FOOT_X);
    FootXPos[2] = constrain(FootXPos[2], MIN_FOOT_X , MAX_FOOT_X);
    FootXPos[3] = constrain(FootXPos[3], MIN_FOOT_X , MAX_FOOT_X);

    FootYPos[0] = constrain(FootYPos[0], MIN_FOOT_HEIGHT , MAX_FOOT_HEIGHT);
    FootYPos[1] = constrain(FootYPos[1], MIN_FOOT_HEIGHT , MAX_FOOT_HEIGHT);
    FootYPos[2] = constrain(FootYPos[2], MIN_FOOT_HEIGHT , MAX_FOOT_HEIGHT);
    FootYPos[3] = constrain(FootYPos[3], MIN_FOOT_HEIGHT , MAX_FOOT_HEIGHT);

    MoveLegTo(FootXPos[0], FootYPos[0] + BACK_TO_FRONT_HIP_DIF, 0);
    MoveLegTo(FootXPos[2], FootYPos[2] + BACK_TO_FRONT_HIP_DIF, 2);
    MoveLegTo(FootXPos[1], FootYPos[1], 1);
    MoveLegTo(FootXPos[3], FootYPos[3], 3);
}

/***************************************************
  MoveTail()

  relative angle is 0 is straight back 
************************************************* */
void MoveTail()
{
  TailPosition = constrain(TailPosition, TAIL_POSITION_MIN , TAIL_POSITION_MAX);
  TailAngle = 90 + TailPosition + MotorBuildOffsets[8];
  
}


/***************************************************
  MoveNeck()

  relative angle is 0 is straight back 
************************************************* */
void MoveNeck()
{
  NeckPosition = constrain(NeckPosition, NECK_POSITION_MIN , NECK_POSITION_MAX);
  NeckAngle = 90 + NeckPosition + MotorBuildOffsets[9];
}

/***************************************************
  MoveHead()

  relative angle is 0 is straight back 
************************************************* */
void MoveHead()
{
  HeadPosition = constrain(HeadPosition, HEAD_POSITION_MIN , HEAD_POSITION_MAX);
  HeadAngle = 90 + HeadPosition + MotorBuildOffsets[9];
}


/***************************************************
  StandPose()
************************************************* */
void StandPose()
{

  speedIndex = 0;
  xBalanceOffset = 0.0;

  FootXPos[0] = 0; FootXPos[1] = 0; FootXPos[2] = 0; FootXPos[3] = 0;
  FootYPos[0] = 92; FootYPos[1] = 92; FootYPos[2] = 92; FootYPos[3] = 92;

  MoveLegTo(FootXPos[0], FootYPos[0] + BACK_TO_FRONT_HIP_DIF, 0);
  MoveLegTo(FootXPos[2], FootYPos[2] + BACK_TO_FRONT_HIP_DIF, 2);
  MoveLegTo(FootXPos[1], FootYPos[1], 1);
  MoveLegTo(FootXPos[3], FootYPos[3], 3);

  TailPosition = 0.0;
  NeckPosition = 0.0;
  HeadPosition = 0.0;

  MoveTail();
  MoveNeck();
  MoveHead();
  
}


/***************************************************
  SetFootXPosition
************************************************* */
void SetFootXPosition( int FootID, float StartPosition, float EndPosition,  float LandFootSooner)
{

  float HipOffset = 0.0;

   if(FootID == 0 ||FootID == 2)
   {  
      HipOffset = BACK_STEP_OFFSET_FROM_HIP_FORWARDS;

     if(rightBackwards || leftBackwards)
        HipOffset = BACK_STEP_OFFSET_FROM_HIP_BACKWARDS;
   }

  if(FootID == 1 ||FootID == 3)
   {  
      HipOffset = FRONT_STEP_OFFSET_FROM_HIP_FORWARDS;

     if(rightBackwards || leftBackwards)
        HipOffset = FRONT_STEP_OFFSET_FROM_HIP_BACKWARDS;
   }
   
  unsigned long NowTime = millis();

  float StepPercentage;
  if(walkphase == FootID)
  {
    StepPercentage = (float)(NowTime - phaseStartTime[FootID]) / (StepTime[speedIndex] - LandFootSooner);
    FootStanding[FootID] = false;
    
  } else {
     StepPercentage = (float)(NowTime - phaseStartTime[FootID]) / (StepTime[speedIndex] * 3);
  }

  StepPercentage = constrain(StepPercentage, 0.0 , 1.0);

  if(!FootStanding[FootID]) {
     
    // Most feet are moving backwards 
    float AdjustedEndPosition = -EndPosition;

    // the foot that is lifted is moving forwards
    if(walkphase == FootID)
      AdjustedEndPosition = EndPosition;

    
    FootXPos[FootID] = GetRelativeValue( StartPosition, AdjustedEndPosition + HipOffset, StepPercentage);
  }

}


/***************************************************
  LeftRightBalance
************************************************* */
void LeftRightBalance(int UpFoot, float OffsetAngle)
{

   float SavedFoot;
   if (UpFoot >= 0)
      SavedFoot = FootYPos[UpFoot];

    
    // This checks if we are standing. or not. That is the only time we will dynamically balance.
    // It is possible to turn this on while walking, but need to tweak PID algorithm so not overcompensate.
   if(OffsetAngle == 0.0 && mode == 1) {

      float xoffset = mpu6050.getAngleX() + OffsetAngle;

      //check if we just fell over
      if (abs(xoffset) > 15)
        return;
    
      double Correction = CorrectSideLean(xoffset);
    
      if (Correction < 0)
      {
        FootYPos[0] -= -Correction;
        FootYPos[1] -= -Correction;
    
        FootYPos[2] += -Correction;
        FootYPos[3] += -Correction;
      } else {
    
        FootYPos[0] += Correction;
        FootYPos[1] += Correction;
    
        FootYPos[2] -= Correction;
        FootYPos[3] -= Correction;
      }
 
    } else {

      #define LEGS_WIDTH  80
      
      double OffsetHeight = tan((OffsetAngle * 71.0)/ 4068.0) * LEGS_WIDTH/2;
      
      FootYPos[0] = NORMAL_WALK_HEIGHT + OffsetHeight;
      FootYPos[1] = NORMAL_WALK_HEIGHT + OffsetHeight;
      
      FootYPos[2] = NORMAL_WALK_HEIGHT - OffsetHeight;
      FootYPos[3] = NORMAL_WALK_HEIGHT - OffsetHeight;
  }

  if (UpFoot >= 0)
    FootYPos[UpFoot] = SavedFoot;
}

/***************************************************
  FrontBackBalance
************************************************* */
void FrontBackBalance(int UpFoot1, int UpFoot2)
{

    float SavedFoot1;
  if (UpFoot1 >= 0)
    SavedFoot1 = FootYPos[UpFoot1];
    
  float SavedFoot2;
  if (UpFoot2 >= 0)
    SavedFoot2 = FootYPos[UpFoot2];
    
  // This checks if we are standing. or not. That is the only time we will dynamically balance.
  // It is possible to turn this on while walking, but need to tweak PID algorithm so not overcompensate.
  if(yBalanceOffset == 0.0 && mode == 1) {
   
    float yoffset = mpu6050.getAngleY() + yBalanceOffset;
  
    //check if we just fell over
    if (abs(yoffset) > 15)
      return;
  
    double Correction = CorrectFrontBackLean(yoffset);

    if (Correction < 0)
    {
      FootYPos[0] += -Correction;
      FootYPos[2] += -Correction;
  
      FootYPos[1] -= -Correction;
      FootYPos[3] -= -Correction;
    } else {
  
      FootYPos[0] -= Correction;
      FootYPos[2] -= Correction;
  
      FootYPos[1] += Correction;
      FootYPos[3] += Correction;
    }

  }

  if (UpFoot1 >= 0)
    FootYPos[UpFoot1] = SavedFoot1;
  
  if (UpFoot2 >= 0)
    FootYPos[UpFoot2] = SavedFoot2;
    
}

/***************************************************
  GetRelativeValue
************************************************* */
float GetRelativeValue( float XStart, float XEnd, float StepPercentageDone)
{
  return XStart + (XEnd - XStart) * StepPercentageDone;
}


/***************************************************    
  MoveLegTo

  x,y is the position of the foot,
    where the hip fo that foot is defined as 0,0 and positive y
    is down (easier since hip is always above foot),
    positive x is in front of the hip and negative x is behind.

  LEGID
  0 is back right
  1 is front right
  2 is back left
  3 is front left

 ************************************************* */
void MoveLegTo(float x, float y, int legID)
{

  if (legID == 0 || legID == 2)
  { //These are back legs
    float IK_A2 = IK_GetA2( x,  y, BACK_LEG_L1, BACK_LEG_L2);
    float IK_A1 = IK_GetA1( x,  y, BACK_LEG_L1, BACK_LEG_L2, IK_A2);

    float A1 = (IK_A1 * 57.2957795);
    float A2 =  (IK_A1 * 57.2957795) - (180 - (IK_A2 * 57.2957795));

    if (legID == 0)
    {
      //back right leg
      A1 = 180 - (A1 - BACK_LEG_HIP_OFFSET_ANGLE + 90 + MotorBuildOffsets[0]);
      A2 =  (A2 - BACK_LEG_KNEE_OFFSET_ANGLE + 90 + MotorBuildOffsets[1]);
      angle0 = A1;
      angle1 = A2;

    } else {

      //back left leg
      A1 = (A1 - BACK_LEG_HIP_OFFSET_ANGLE + 90 + MotorBuildOffsets[4]);
      A2 =  180 - (A2 - BACK_LEG_KNEE_OFFSET_ANGLE + 90 + MotorBuildOffsets[5]);
      angle4 = A1;
      angle5 = A2;

    }
  }
  else
  {
    //These are the front legs


    // The IK formula are assuming the knee bends forwards.
    // So for the front leg I just invert X
    x = -x;

    float IK_A2 = IK_GetA2( x,  y, FRONT_LEG_L1, FRONT_LEG_L2);
    float IK_A1 = IK_GetA1( x,  y, FRONT_LEG_L1, FRONT_LEG_L2, IK_A2);

    float A1 = (IK_A1 * 57.2957795);
    float A2 =  (IK_A1 * 57.2957795) - (180 - (IK_A2 * 57.2957795));

    if (legID == 1)
    {
      // Front right leg
      A1 = (A1 - FRONT_LEG_HIP_OFFSET_ANGLE + 90 + MotorBuildOffsets[3]);
      A2 =  180 - (A2 - FRONT_LEG_KNEE_OFFSET_ANGLE + 90 + MotorBuildOffsets [2]);
      angle3 = A1;
      angle2 = A2;

    } else {

      //front left leg
      A1 = 180 - (A1 - FRONT_LEG_HIP_OFFSET_ANGLE + 90 + MotorBuildOffsets[7]);
      A2 =  (A2 - FRONT_LEG_KNEE_OFFSET_ANGLE + 90 + MotorBuildOffsets[6]);
      angle7 = A1;
      angle6 = A2;

    }
  }


}



/***************************************************
  Inverse Kinematics for the legs

  This is a generalized inverse kinematic function for
  any two link leg. The length of the leg links are inputs.

  A1 is the angle at the hip.
  A2 is the angle at the knee
  l1 is the length of the upper leg
  l2 is the legnth of the lower leg
  x,y is the position of the foot,
    where the hip is defined as 0,0 and positive y
    is down (easier since hip is always above foot).

  For this robot all the lengths are millimeters, not that the code
  cares, as along as all the length units are the same.

*******************************************/
float IK_GetA2(float x, float y, float l1, float l2)
{
  return acos((x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2));
}

float IK_GetA1(float x, float y, float l1, float l2, float A2)
{
  float IK_A1 = atan(y / x) - atan((l2 * sin(A2)) / (l1 + l2 * cos(A2)));

  if (IK_A1 < 0)
    IK_A1 += PI;

  return IK_A1;
}



/*************************
   CorrectSideLean
 *************************/
double CorrectSideLean(double error)
{

  double pTerm = Kp_LR * error;
  double dTerm = Kd_LR * (error - last_error_LR);

  integrated_error_LR += error;
  double iTerm = Ki_LR * constrain(integrated_error_LR, -40, 40);
  last_error_LR = error;

  double PositionError = constrain(K_PLR * (pTerm + dTerm + iTerm), -GUARD_GAIN, GUARD_GAIN);

  return PositionError;
}


/*************************
   CorrectFrontBackLean
 *************************/
double CorrectFrontBackLean(double error)
{

  double pTerm = Kp_FB * error;
  double dTerm = Kd_FB * (error - last_error_FB);

  integrated_error_FB += error;
  double iTerm = Ki_FB * constrain(integrated_error_FB, -40, 40);
  last_error_FB = error;

  double PositionError = constrain(K_PFB * (pTerm + dTerm + iTerm), -GUARD_GAIN, GUARD_GAIN);

  return PositionError;
}

/***************************************************
  Sets the pulse width of the servo controler given an angle

*******************************************/
int SetServoAngle(int servo, float angle)
{
  int pulse_wide, analog_value;
  pulse_wide =  (angle  * (float)(MAX_PULSE_WIDTH - MIN_PULSE_WIDTH)) / 180.0 + MIN_PULSE_WIDTH;
  analog_value = (int)(((long)pulse_wide* (long)FREQUENCY * (long)4096) / (long)1000000) ;
  pwm.setPWM(servo, 0, analog_value);

  // delay(25);
}
