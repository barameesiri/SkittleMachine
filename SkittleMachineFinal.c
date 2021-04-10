//Skittle-Final-Intergration main.c
#include <msp430.h> 
//Miles Code
#include <stdio.h>
#define startByte (255)
// Motor Calibration
#define xLimit (1250) //[steps];
#define yOffset (280)
#define xOffset (1140)
int motorCalFactor = 130; //[steps / Grid Index]
#define TRUE (1)
#define FALSE (0)
int dataParsingState = 0;
//volatile int color; <-------- Change this
//Motor Control
volatile int xCount = 0;
volatile int yCount = 0;
volatile int xPos0 = 0; //Current X-position
volatile int yPos0 = 0; //Current Y-position
volatile int xPos1 = 0; //X-position To-Go
volatile int yPos1 = 0; //Y-position To-Go
volatile int Lx; //travel length in x axis
volatile int Ly; //travel length in y axis
volatile int posCmd; //position command from UART: data structure |= xIndex << 4 + yIndex = (0000 0000)b <=> xIndex*16 + yIndex
volatile int xMotorFLG = 0;
volatile int yMotorFLG = 0;
volatile int motorFLG = 0;
volatile int startFLG = 0;
volatile int homeFLG = 0; // 1 - homing axes in operation; 0 - axes homed
volatile int xHomeFLG;
volatile int yHomeFLG;
int RxByte;

typedef struct {
  int front;
  int num;
  int capacity;
  int* arr;
} Queue;
//*********


//GLOBAL FLAG****
int capture = 0; //Start-stop frequency grabber
int senseState = 0; //Change filter state and go to calculation
int colorFlag = 0; //Flag to enque to correct color queue
int redFlag = 1; //Flag for frequency grabber to enqueue buffer red
int greenFlag = 2; //Flag for frequency grabber to enqueue buffer green
int blueFlag = 3; //Flag for frequency grabber to enqueue buffer blue
int clearFlag = 4;//Flag for frequency grabber to enqueue buffer clear
int color1 = 0; //Detected color [1 = red, 2 = green, 3= Purple, 4 = Yellow, 5 = Orange,0 = nothing]
int color2 = 0;
int globalState = 0;
int gotColor = 0;

//Color Sensing for frequency grabber
unsigned int ini_cap;
unsigned int fin_cap;
unsigned int sense;

//For TB0.0 Filter Switching
unsigned int R = 0;
unsigned int G = 0;
unsigned int B = 0;
unsigned int CL = 0;
int senseTime = 1;

//Circular Buffer
//RED BUFFER
volatile unsigned int redData[90];
int frontRed = 0;
int numRed = 0;
int capacityRed = 90;
int redBufferFull = 0;
int redBufferEmpty = 0;
unsigned int temp = 0;
//GREEN BUFFER
volatile unsigned int gData[50];
int frontG = 0;
int numG = 0;
int capacityG = 50;
int gBufferFull = 0;
int gBufferEmpty = 0;
//BLUE BUFFER
volatile unsigned int bData[50];
int frontB = 0;
int numB = 0;
int capacityB = 50;
int bBufferFull = 0;
int bBufferEmpty = 0;
//CL BUFFER
volatile unsigned int clData[50];
int frontCL = 0;
int numCL = 0;
int capacityCL = 50;
int clBufferFull = 0;
int clBufferEmpty = 0;

//Motor variables
#define stepsPerRevolution (3200);
volatile int position = 3200; // [steps]
volatile int positionOne = 516;
volatile int positionTwo = 284;
volatile int pulseCount = 0;
int iTime = 1;
int motorState = 0;
int S2Toggle = 0;

//Miles*****************************************
void initQueue(Queue* q) {
  q->front = 0;
  q->num = 0;
  q->capacity = 50;
  q->arr = (int*) malloc(q->capacity * sizeof(int));
}

int isEmpty(Queue* q) {
  if (q->num == 0)
    return TRUE;
  else
    return FALSE;
}

int isFull(Queue* q) {
  if (q->num == q->capacity)
    return TRUE;
  else
    return FALSE;
}

int enqueue(Queue* q, int val) {
  if (isFull(q))
    return FALSE;
  else {
    q->arr[(q->front + q->num) % q->capacity] = val;
    q->num++;
    return TRUE;
  }
}

int dequeue(Queue* q) {
    int frontNum;
    if (isEmpty(q))
      return FALSE;
    else {
      frontNum = q->arr[q->front];
      q->arr[q->front] = -1;
      q->front = (q->front + 1) % q->capacity;
      q->num--;
      return frontNum;
    }
}

Queue RedSkittleQueue;
Queue GreenSkittleQueue;
Queue PurpleSkittleQueue;
Queue YellowSkittleQueue;
Queue OrangeSkittleQueue;

int IsFinish(){

    if(isEmpty(&RedSkittleQueue)){
        if(isEmpty(&GreenSkittleQueue))
            if(isEmpty(&PurpleSkittleQueue))
                if(isEmpty(&YellowSkittleQueue))
                    if(isEmpty(&OrangeSkittleQueue))
                        return TRUE;
    }
    else{
        return FALSE;
    }

}
void MotorTimerConfig(void){
    /*
    //Configure Output P1.4 and P1.5 for TB0.1 and TB0.2
    P1DIR |= BIT4 + BIT5;
    P1SEL0 |= BIT4 + BIT5;
    P1SEL1 &= (~BIT4) + (~BIT5);
    */
    //configure Timer B0 to trigger interrupt
    // ACLK; ID = 11b = BIT6 + BIT7 = f_clk/8; MC_1 = up MODE; Clear TxR; Enable TxIFG
    TA1CTL = TASSEL0 + MC_1 + TACLR + TAIE + (BIT6 + BIT7);
    TA1EX0 = TAIDEX_3; // Timer B0 divide source clock = 1MHz / 4 = 250kHz;

    TA1CCTL1 = OUTMOD_7+ CCIE; // TB0CCR1 input signal: reset/set output; interrupt enabled; default CAP = 0: compare mode
    TA1CCTL1 &=~CCIFG;

    TA1CCR0 = 0;
    TA1CCR1 = TA1CCR0/2;
    TA1CCR2 = 0;
    //Capture/compare register
    TA1CCTL0|=CCIE; //Timer overflow
    TA1CCTL0&=~CCIFG;
}

void UARTconfig(void){

    // Configure ports for UART0 for MSP430 USB
    P2SEL0 &= ~(BIT0 + BIT1);
    P2SEL1 |= BIT0 + BIT1;
    // Configure UART0
    UCA0CTLW0 = UCSSEL0;                    // Run the UART using ACLK
    UCA0MCTLW = UCOS16 + UCBRF_0 + 0x4900;   // Baud rate = 9600 from an 8 MHz     clock
    UCA0BRW = 52;
    UCA0IE |= UCRXIE;                       // Enable UART Rx interrupt
}

void motorStart(){
    motorFLG = 1;
    TA1CCR0 = 500;
    TA1CCR1 = TA1CCR0/2;
}
void motorStop(){
    motorFLG = 0;
    TA1CCR0 = 0;
    TA1CCR1 = 0;
}
void processPosCmd(int pos){
    xPos0 = xPos1;
    yPos0 = yPos1;
    int xIndex = pos/16;
    int yIndex = pos%16;
    xPos1 = xOffset - motorCalFactor * xIndex; // xPos = pos >> 4;
    yPos1 = yOffset + motorCalFactor * yIndex; // yPos = pos &= 0xF0;

    Lx = abs(xPos1 - xPos0);
    Ly = abs(yPos1 - yPos0);

    if(Lx == 0){
        xMotorFLG = 1;
    }else
        xMotorFLG = 0;

    if(Ly == 0){
        yMotorFLG = 1;
    }else
        yMotorFLG = 0;

    //Set x-axis direction
    if(xPos1 < xPos0){
        P3OUT |= BIT1;
    }
    else
        P3OUT &= ~BIT1;
    //Set y-axis direction
    if(yPos1 < yPos0){
        P3OUT |= BIT3;
    }
    else
        P3OUT &= ~BIT3;
}

void GetTrashPos(){
    xPos0 = xPos1;
    yPos0 = yPos1;
    yPos1 = 0; // yPos = y origin;

    Lx = abs(xPos1 - xPos0);
    Ly = abs(yPos1 - yPos0);

    if(Lx == 0){
        xMotorFLG = 1;
    }else
        xMotorFLG = 0;

    if(Ly == 0){
        yMotorFLG = 1;
    }else
        yMotorFLG = 0;

    //Set x-axis direction
    if(xPos1 < xPos0){
        P3OUT |= BIT1;
    }
    else
        P3OUT &= ~BIT1;
    //Set y-axis direction
    if(yPos1 < yPos0){
        P3OUT |= BIT3;
    }
    else
        P3OUT &= ~BIT3;
}


void checkAxisHome(){
    if(P1IN & BIT4){
        xHomeFLG = 0;
    }else{
        xHomeFLG = 1;
    }
    if(P1IN & BIT5){
        yHomeFLG = 0; //PIN Pulled to High when switch is open, axis homed
    }else{
        yHomeFLG = 1;
    }
}

//**********************************************

void addToCLQueue(unsigned int data){
    int index;
    if(numCL < 50){
        index = (frontCL + numCL) % capacityCL;
        clData[index] = data;
        numCL++;
    }else{
        clBufferFull = 1;
    }
}

int getFromCLQueue(void){
    unsigned int retreive;
    if(numCL >= 0){
        retreive = clData[frontCL];
        clData[frontCL] = 0;
        numCL--;
        frontCL = (frontCL+1)%capacityCL;
    }else{
        clBufferEmpty = 1;
    }
    return retreive;
}

void addToBQueue(unsigned int data){
    int index;
    if(numB < 50){
        index = (frontB + numB) % capacityB;
        bData[index] = data;
        numB++;
    }else{
        bBufferFull = 1;
    }
}

int getFromBQueue(void){
    unsigned int retreive;
    if(numB >= 0){
        retreive = bData[frontB];
        bData[frontB] = 0;
        numB--;
        frontB = (frontB+1)%capacityB;
    }else{
        bBufferEmpty = 1;
    }
    return retreive;
}

void addToGQueue(unsigned int data){
    int index;
    if(numG < 50){
        index = (frontG + numG) % capacityG;
        gData[index] = data;
        numG++;
    }else{
        gBufferFull = 1;
    }
}

int getFromGQueue(void){
    unsigned int retreive;
    if(numG >= 0){
        retreive = gData[frontG];
        gData[frontG] = 0;
        numG--;
        frontG = (frontG+1)%capacityG;
    }else{
        gBufferEmpty = 1;
    }
    return retreive;
}

void addToRedQueue(unsigned int data){
    int index;
    if(numRed < 50){
        index = (frontRed + numRed) % capacityRed;
        redData[index] = data;
        numRed++;
    }else{
        redBufferFull = 1;
    }
}

int getFromRedQueue(void){
    unsigned int retreive;
    if(numRed >= 0){
        retreive = redData[frontRed];
        redData[frontRed] = 0;
        numRed--;
        frontRed = (frontRed+1)%capacityRed;
    }else{
        redBufferEmpty = 1;
    }
    return retreive;
}


void configPin(){
    //PJ0 = LED1 Enqueue and Dequeue indicator
    //P1.4 = X-axis end switch
    //P1.5 = Y-axis end switch
    //P1.7 = S0 [Red]
    //P3.0 = X-axis StepPin
    //P3.1 = X-axis  DirPin
    //P3.2 = Y-axis StepPin
    //P3.3 = Y-axis DirPin
    //P3.4 = Revolver stepper
    //P3.5 = S1 [Yellow]
    //P3.6 = S3 [Purple]
    //P3.7 = S2 [Blue]
    //P4.0 = Switch 1 interrupt
    //P4.1 = Switch 2 interrupt
    P1DIR |= BIT7;
    P3DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5 + BIT6 + BIT7;
    //20% freq
    P1OUT |= BIT7;
    P3OUT &= ~BIT5;
    //Enable interrupt S1 and S2
    P4DIR &= (~BIT0) + (~BIT1); //set P4.0 to input
    P4REN |= BIT0 + BIT1; //enable pull-up resistor for P4.0
    P4OUT |= BIT0 + BIT1;
    P4IE |= BIT0 + BIT1; //p4.0 and p4.1 interrupt enabled
    P4IES |= (BIT0) + BIT1; // select FALLING edge detection
    P4IFG &= (~BIT0) + (~BIT1); //p4.0 interrupt flag cleared
    //Motor Config
    P3DIR |= BIT4;
    // Set the initial spinning direction as 0;
    P3OUT &= (~BIT1) + (~BIT3);
    //configure P1.4 as pin input for X-axis END SWITCH
    P1DIR &= ~BIT4;
    P1REN |= BIT4;
    P1OUT |= BIT4; //enable pull-up resistor
    //configure P1.5 as pin input for Y-axis END SWITCH
    P1DIR &= ~BIT5;
    P1REN |= BIT5;
    P1OUT |= BIT5; //enable pull-up resistor

    //LED Indicator
    PJDIR|=BIT0;
    PJOUT &= (~BIT0);
}

void configCLK(){
    // Configure clocks
    CSCTL0 = 0xA500;                        // Write password to modify CS registers
    CSCTL1 = DCOFSEL0 + DCOFSEL1;           // DCO = 8 MHz
    CSCTL2 = SELM0 + SELM1 + SELA0 + SELA1 + SELS0 + SELS1; // MCLK = DCO, ACLK = DCO, SMCLK = DCO
    CSCTL3 = DIVM__4; //Div MCLK by 4 = 2MHz
}

void configTimerA(){
    //TA1.CCI1A input
    P1DIR &= ~BIT6;       //P1.6 as input
    P1SEL1 |= BIT6;       //Select TA0.CCAI0A
    P1SEL0 |= BIT6;
    //By default this timer will take in P1.2 input
    TA0CTL = TASSEL_1 + MC_2 + TACLR; //TASSEL0 = TA0CLK; MC_2 = UP CONTINUOUS MODE; Clear TAR; Enable TAIFG
    TA0CCTL0 = CCIS_0 + CAP + CM_1; //Use CCIA as input + Capture mode on rising edge
    //TA0CCTL0 |= CCIE; //!!!!INTERUPT DISABLE
    TA0CCR0 = 0xFFFF;
}

void configTimerB(){
    //Frequency grabber [Timer B0]
    TB0CTL = TBSSEL_1 + ID_3 + MC_1 +TBCLR;  // Timer B div 8 = 250kHz, UP mode, Clear TBR counter
    TB0EX0 = TBIDEX__8;                     // div 8 to 31.25 kHz
    TB0CCTL0 = OUTMOD_7;        //!!!!Interupt Disable!!!!!!!!!! // Set/Reset mode , compare mode, Interrupt enable
    //TB0CCTL0 |= CCITE; //!!!INTERUPT DISABLE
    TB0CCR0 = 4000;
    TB0CCTL0 &= ~CCIFG;         //Clear interrupt

    //Motor stepping [Timer B1]
    //TBSSEL1 = ACLK; ID = 11b = BIT6 + BIT7 = f_clk/8; MC_1 = up MODE; Clear TBR; Enable TAIFG
    TB1CTL = TBSSEL_1 + MC_1 + TBCLR + (BIT6 + BIT7); //!!!!Interupt Disable!!!!!!!!!!
    //TB1CTL |= TBIE; //!!!!Interupt Disable!!!!!!!!!!
    TB1EX0 = TBIDEX_3;// Timer B0 divide source clock = 1MHz / 4 = 250kHz;
    TB1CCTL1 = OUTMOD_7; // TB0CCR1 input signal: reset/set output; interrupt enabled; default CAP = 0: compare mode
    //TB1CCTL1 |= CCIE;   //!!!!Interupt Disable!!!!!!!!!!
    TB1CCTL1 &= ~CCIFG;
    TB1CCR0 = 500;      //To achieve 500Hz step freq
    TB1CCR1 = TB1CCR0/2;
    //Capture/compare register
    //TB1CCTL0|=CCIE; //!!!!Interrupt Disable!!!!!!!!!!//Timer overflow
    TB1CCTL0&=~CCIFG;
}

void enableTA0(){
    //Freq grabber
    TA0CCTL0 |= CCIE;
}

void disableTA0(){
    //Freq grabber
    TA0CCTL0 &= ~CCIE;
}
void enableTB0(){
    //Filter changer
    TB0CCTL0 |= CCIE;
}

void disableTB0(){
    //Filter changer
    TB0CCTL0 &= ~CCIE;

}

void enableTB1(){
    //Motor
    TB1CCR0 = 500;
    TB1CCR1 = TB1CCR0/2;
    TB1CTL |= TBIE;
    TB1CCTL1 |= CCIE;
    TB1CCTL0|=CCIE;
}

void disableTB1(){
    //Motor
    TB1CCR0 = 0;
    TB1CCR1 = 0;
    TB1CTL &= ~TBIE;
    TB1CCTL1 &= ~CCIE;
    TB1CCTL0 &= ~CCIE;
}

void move58(){
    enableTB1();
    while(pulseCount < positionOne);
    disableTB1();
    pulseCount = 0;
}

void move32(){
    enableTB1();
    while(pulseCount < positionTwo);
    disableTB1();
    pulseCount = 0;
}

int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    _EINT();
    configCLK();
    configPin();
    configTimerA();
    configTimerB();
    MotorTimerConfig();
    UARTconfig();
    initQueue(&RedSkittleQueue);
    initQueue(&GreenSkittleQueue);
    initQueue(&PurpleSkittleQueue);
    initQueue(&YellowSkittleQueue);
    initQueue(&OrangeSkittleQueue);
    int i;
    while(1){
        checkAxisHome();
        if(homeFLG == 0){
            if(globalState == 0){
                        //Wait for startbyte
                        if(startFLG == 1)
                            globalState = 1;
                        else
                            globalState = 0;
                    }else if(globalState == 1){
                        //Move 58 deg
                        move58();
                        globalState = 2;
                    }else if(globalState == 2){
                        //Sense color1
                        senseTime = 1;
                        if(gotColor != 1){
                            enableTB0();
                            enableTA0();
                            while(gotColor != 1); // wait for color sensor
                            disableTA0();
                            disableTB0();
                        }
                        globalState = 3;
                    }else if(globalState == 3){
                        //Rotate 32 deg
                        gotColor = 0;
                        move32();
                        globalState = 4;
                    }else if(globalState == 4){
                        //Rotate 58 deg
                        move58();
                        globalState = 5;
                    }else if(globalState == 5){
                        //Sense color2
                        senseTime = 2;
                        if(gotColor != 1){
                            enableTB0();
                            enableTA0();
                            while(gotColor != 1); // wait for color sensor
                            disableTA0();
                            disableTB0();
                        }
                        globalState = 6;
                    }else if(globalState == 6){
                        color1 = 1;
                        gotColor = 0;
                        //Move to dropping position
                        if(IsFinish()){
                            //Operation finished
                            globalState = 8;
                        }
                        else{
                            if(color1 == 1){ //Place Red Skittles
                                if(!isEmpty(&RedSkittleQueue)){
                                    posCmd = dequeue(&RedSkittleQueue);
                                    processPosCmd(posCmd);
                                    motorStart();
                                    while(!(xMotorFLG & yMotorFLG)); //Wait to reach target (x,y) point
                                    motorStop();
                                    xCount = 0;
                                    yCount = 0;
                                }
                            }
                            else if(color1 == 2){
                                if(!isEmpty(&GreenSkittleQueue)){
                                    posCmd = dequeue(&GreenSkittleQueue);
                                    processPosCmd(posCmd);
                                    motorStart();
                                    while(!(xMotorFLG & yMotorFLG)); //Wait to reach target (x,y) point
                                    motorStop();
                                    xCount = 0;
                                    yCount = 0;
                                }
                            }
                            else if(color1 == 3){
                                if(!isEmpty(&PurpleSkittleQueue)){
                                    posCmd = dequeue(&PurpleSkittleQueue);
                                    processPosCmd(posCmd);
                                    motorStart();
                                    while(!(xMotorFLG & yMotorFLG)); //Wait to reach target (x,y) point
                                    motorStop();
                                    xCount = 0;
                                    yCount = 0;
                                }
                            }
                            else if(color1 == 4){
                                if(!isEmpty(&YellowSkittleQueue)){
                                    posCmd = dequeue(&YellowSkittleQueue);
                                    processPosCmd(posCmd);
                                    motorStart();
                                    while(!(xMotorFLG & yMotorFLG)); //Wait to reach target (x,y) point
                                    motorStop();
                                    xCount = 0;
                                    yCount = 0;
                                }
                            }
                            else if(color1 == 5){
                                if(!isEmpty(&OrangeSkittleQueue)){
                                    posCmd = dequeue(&OrangeSkittleQueue);
                                    processPosCmd(posCmd);
                                    motorStart();
                                    while(!(xMotorFLG & yMotorFLG)); //Wait to reach target (x,y) point
                                    motorStop();
                                    xCount = 0;
                                    yCount = 0;
                                }
                            }
                            else if(color1 == 0){
                                //TrashColor
                                GetTrashPos();
                                motorStart();
                                while(!(xMotorFLG & yMotorFLG)); //Wait to reach target (x,y) point
                                motorStop();
                                xCount = 0;
                                yCount = 0;
                            }
                            globalState = 7;
                        }
                    }else if(globalState == 7){
                        //color1 = color2
                        color1 = color2;
                        //Rot 32 deg
                        move32();
                        //Pause before loading the new skittles for a bit
                        for(i=0;i<2000;i++){
                            _NOP();
                        }
                        globalState = 4;
                    }else if(globalState == 8){
                        //Done printing job
                        startFLG = 0;
                        motorStop();
                        homeFLG = 1;
                        P3OUT |= BIT1 + BIT3;
                        motorStart();
                        globalState = 0;
                    }
        }
    }
    return 0;
}

//Motor driving
#pragma vector = TIMER1_B0_VECTOR
__interrupt void TIMER1_B0_ISR(void)
{
    P3OUT &= ~BIT4;
    TB1CCTL0 &=~CCIFG;
}

//Motor driving
#pragma vector = TIMER1_B1_VECTOR
__interrupt void TIMER1_B1_ISR(void){
    switch(TB1IV){
    case TB1IV_TBCCR1:        //TB0CCR1 CCIFG
        P3OUT |= BIT4;  //Turn the motor-on
        pulseCount++;
        TB1CCTL1 &= ~CCIFG;
        break;
    }
}

//Filter changing
#pragma vector = TIMER0_B0_VECTOR
__interrupt void TIMBER0_B0_ISR(void){
        int color = 0;
        if(senseState == 0){
            capture = 1;
            colorFlag = redFlag;
            P3OUT &= (~BIT6) + (~BIT7);
            senseState = 1;
        }else if(senseState == 1){
            capture = 1;
            colorFlag = greenFlag;
            //GREEN
            P3OUT |= BIT6 + BIT7;
            senseState = 2;
        }else if(senseState == 2){
            capture = 1;
            colorFlag = blueFlag;
            //BLUE
            P3OUT |= BIT6;
            P3OUT &= ~BIT7;
            senseState = 3;
        }else if(senseState == 3){
            capture = 1;
            //P3.7 = S2
            //P3.6 = S3
            colorFlag = clearFlag;
            P3OUT |= BIT7;
            P3OUT &= ~BIT6;
            senseState = 4;
        }else if(senseState == 4){
            //Calculating Average
            capture = 0;
            while(numRed > 0 && redBufferEmpty != 1){
                temp = getFromRedQueue();
                R = (R + temp)/2;
            }
            while(numG > 0 && gBufferEmpty != 1){
                temp = getFromGQueue();
                G = (G + temp)/2;
            }
            while(numB > 0 && bBufferEmpty != 1){
                temp = getFromBQueue();
                B = (B + temp)/2;
            }
            while(numCL >0 && clBufferEmpty != 1){
                temp  = getFromCLQueue();
                CL = (CL + temp)/2;
            }

            //Check for orange
            if(R >= 1800 && R <= 1910){
               if(G >= 2410 && G <= 2550){
                   if(B >= 2220 && B <= 2360){
                         //Orange
                         color = 5;
                   }
               }
            }else if(R >= 1630 && R <= 1827){
                if(G >=1940 && G <= 2160){
                    if(B >= 2060 && B <= 2235){
                        //Yellow
                        color = 4;
                    }
                }
            }else if(R >= 1880 && R <= 2210){
                if(G >= 2240 && G <= 2300){
                    if(B >= 2230 && B <= 2300){
                        //Green
                        color = 2;
                    }
                }
            }else if(R >= 2010 && R <= 2090){
                if(G >=2530 && G <= 2590){
                    if(B >= 2250 && B <= 2330){
                        //Red
                        color = 1;
                    }
                }
            }else if(R >= 2215 && R <= 2250){
                if(G >=2500 && G <= 2600){
                    if(B >= 2240 && B <= 2350){
                        //Purple
                        color = 3;
                    }
                }
            }else{
                color = 0;
            }
            if(senseTime == 1){
                color1 = color;
            }else if(senseTime == 2){
                color2 = color;
            }
            senseState = 0;
            capture = 1;
            gotColor = 1;
        }
        TB0CCTL0 &= ~CCIFG; //Clear TB1CCTL0 interrupt flag
}

//Frequency grabber
#pragma vector = TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
{
        unsigned int temp;
        int count = 0;
        if(capture == 1){
            if(TA0CCTL0&CM_1)
               {
                   ini_cap = TA0CCR0; //Time when rising edge occur
                   TA0CCTL0 = CCIS_0 + CAP + CM_2 + CCIE; // Switch to lowering edge detection
               }else
               {
                   fin_cap = TA0CCR0; //Time when lowering edge occur
                   sense = fin_cap - ini_cap;
                   TA0CCTL0 = CCIS_0 + CAP + CM_1 + CCIE; // Switch to rising edge detection
                   if(colorFlag == redFlag){
                      addToRedQueue(sense);
                   }else if(colorFlag == greenFlag){
                       //G = sense;
                       addToGQueue(sense);
                   }else if(colorFlag == blueFlag){
                       //B = sense;
                       addToBQueue(sense);
                   }else if(colorFlag == clearFlag){
                       //CL = sense;
                       addToCLQueue(sense);
                   }
               }
        }

    TA0CCTL0 &= ~CCIFG; //Clear interrupt flag
}

#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
    RxByte = UCA0RXBUF;
    if(dataParsingState == 0 && RxByte == 255){
        dataParsingState = 1;
    }
    else if(dataParsingState == 1){
        if(RxByte == 1){
            dataParsingState = 2; //Red
        }
        else if(RxByte == 2){
            dataParsingState = 3; //Green
        }
        else if(RxByte == 3){
            dataParsingState = 4; //Purple
        }
        else if(RxByte == 4){
            dataParsingState = 5; //Yellow
        }
        else if(RxByte == 5){
            dataParsingState = 6;  //Orange
        }
    }
    else if(dataParsingState == 2){
        enqueue(&RedSkittleQueue, RxByte);
        dataParsingState = 0;
    }
    else if(dataParsingState == 3){
        enqueue(&GreenSkittleQueue, RxByte);
        dataParsingState = 0;
    }
    else if(dataParsingState == 4){
        enqueue(&PurpleSkittleQueue, RxByte);
        dataParsingState = 0;
    }
    else if(dataParsingState == 5){
        enqueue(&YellowSkittleQueue ,RxByte);
        dataParsingState = 0;
    }
    else if(dataParsingState == 6){
        enqueue(&OrangeSkittleQueue, RxByte);
        dataParsingState = 0;
    }

    while (!(UCA0IFG & UCTXIFG));           // Wait until the previous Tx is finished
    UCA0TXBUF = RxByte; //echo
}



//configure P3.0 X-axis StepPin;
//          P3.1 X-axis DirPin; Default = 0
//          P3.2 Y-axis StepPin;
//          P3.3 Y-axis DirPin; Default = 0
//configure P2.1 X-axis END SWITCH
//configure P2.2 Y-axis END SWITCH
//
#pragma vector = TIMER1_A0_VECTOR
__interrupt void TIMER1_A0_ISR(void)
{
    P3OUT &= ~BIT0;
    P3OUT &= ~BIT2;
    TA1CCTL0 &=~CCIFG;
}
#pragma vector = TIMER1_A1_VECTOR
__interrupt void TIMER1_A1_ISR(void){
    switch(__even_in_range(TA1IV,14)){
    case 0: break; //no interrupt
    case 2:        //TA1CCR1 CCIFG

        if(homeFLG == 0){
            if(motorFLG == 1){
                //move in X axis
                if(xCount < Lx){
                    P3OUT |= BIT0;
                    xCount++;
                }
                else{
                    xMotorFLG = 1; // X position reached
                }
                if(yCount < Ly){
                    P3OUT |= BIT2;
                    yCount++;
                }
                else{
                    yMotorFLG = 1; // X position reached
                }
            }
        }
        else if(homeFLG == 1){
            if(P1IN & BIT4)
                xHomeFLG = 0;
            else
                P3OUT |= BIT0;

            if(P1IN & BIT5)
                yHomeFLG = 0;
            else
                P3OUT |= BIT2;

            if(xHomeFLG == 0 && yHomeFLG == 0){
                homeFLG = 0;
                xPos0 = 0;
                yPos0 = 0;
                xPos1 = 0;
                yPos1 = 0;
                xMotorFLG = 1;
                yMotorFLG = 1;
                motorStop();
            }
        }

        TA1CCTL1 &= ~CCIFG;
        break;
    case 4:
        TA1CCTL2 &= ~CCIFG;
        break; //TA1CCR2 CCIFG
    default: break;
    }
}
//port 4 interrupt service routine
#pragma vector = PORT4_VECTOR
__interrupt void Port_4(void){
    int num;
   switch(__even_in_range(P4IV, 16)){ //the switch case has only even number of selections: http://www.ti.com/lit/ug/slau132u/slau132u.pdf
        case 0: break; // No Interrupt
        case 2:         // P4.0: S1
            startFLG = 1;
            xMotorFLG = 0;
            yMotorFLG = 0;
            P4IFG &= (~BIT0); //reset flag P4.0
            break;
        case 4:         // P4.1 S2
            startFLG = 0;
            motorStop();
            homeFLG = 1;
            P3OUT |= BIT1 + BIT3;
            motorStart();

            PJOUT|=BIT0; //Resetting queue indicator

            while(!isEmpty(&RedSkittleQueue)){
                dequeue(&RedSkittleQueue);
            }

            while(!isEmpty(&GreenSkittleQueue)){
                dequeue(&GreenSkittleQueue);
            }

            while(!isEmpty(&PurpleSkittleQueue)){
                dequeue(&PurpleSkittleQueue);
            }

            while(!isEmpty(&YellowSkittleQueue)){
                dequeue(&YellowSkittleQueue);
            }

            while(!isEmpty(&OrangeSkittleQueue)){
                dequeue(&OrangeSkittleQueue);
            }

            PJOUT &= (~BIT0);
            P4IFG &= (~BIT1); //reset flag P4.0
            break; // P4.1
        case 6: break; // P4.2
        case 8: break;// P4.3
        case 10: break; // P4.4
        case 12: break; // P4.5
        case 14: break; // P4.6
        case 16: break; // P4.7
    }
}

