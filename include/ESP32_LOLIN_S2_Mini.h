#define BOARD "FT_S2MINI"

#define LAMP1PIN 13
#define LAMP2PIN 12
#define LAMP3PIN 10
#define LAMP4PIN 11
#define LAMP5PIN 8
#define LAMP6PIN 9
#define LAMP7PIN 6
#define LAMP8PIN 7
#define LAMP9PIN 4
#define LAMP10PIN 5
#define LAMP11PIN 2
#define LAMP12PIN 3
#define LAMP13PIN 14
#define LAMP14PIN 1

#define MOT1PIN 18
#define MOT2PIN 33
#define MOT3PIN 35
#define MOT4PIN 37
#define MOT5PIN 21
#define MOT6PIN 34
#define MOT7PIN 36
#define MOT8PIN 38

#define DIRPIN 16
#define LIMITSENSOR1PIN 39
#define LIMITSENSOR2PIN 40

#define TIMER0_INTERVAL_MS        0.5    // Used for the stepper motor increments.  Generates an interrupt every 500uS
#define TIMER1_INTERVAL_MS        0.1   // Used for LED PWM

// Don't worry about these defines

#define _TIMERINTERRUPT_LOGLEVEL_     3