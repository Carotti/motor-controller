#include "mbed.h"
#include "rtos.h"
#include "hash/SHA256.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
const int8_t lead = 2;  //2 for forwards, -2 for backwards

int orState = 0;

uint8_t sequence[] = {  0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
                        0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
                        0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
                        0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
                        0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
                        0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint64_t* key = (uint64_t*)((int)sequence + 48);

uint64_t* nonce = (uint64_t*)((int)sequence + 56); // LITTLE ENDIAN!!

uint8_t hash[32];

volatile uint64_t newKey;
Mutex newKey_mutex;

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

typedef struct {
    uint8_t code;
    uint32_t data;
} message_t;

Mail<message_t, 16> outMessages;

typedef enum {
    hello,
    rotorOrigin,
    nonceFoundFirst, // First half of the nonce data
    nonceFoundSecond, // Second half of the nonce data
    hashRate
} messageTypes;

Thread commOutT;
Thread commDecodeT;

Queue<void, 8> inCharQ;

//Initialise the serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);

void putMessage(uint8_t code, uint32_t data) {
    message_t* msg = outMessages.alloc();
    msg->code = code;
    msg->data = data;
    outMessages.put(msg);
}

void commOutFn() {
    while(1) {
        osEvent newEvent = outMessages.get();
        message_t* pMsg = (message_t*) newEvent.value.p;
        switch(pMsg->code) {
            case hello:
                pc.printf("Hello\n\r");
                break;
            case rotorOrigin:
                pc.printf("Rotor origin: %d\n\r", pMsg->data);
                break;
            case nonceFoundFirst:
                pc.printf("Nonce found: 0x%08x", pMsg->data);
                break;
            case nonceFoundSecond:
                pc.printf("%08x\n\r", pMsg->data);
                break;
            case hashRate:
                pc.printf("Hash rate is: %d #/s\n\r", pMsg->data);
                break;
        }
        outMessages.free(pMsg);
    }
}

void serialISR() {
    uint8_t newChar = pc.getc();
    inCharQ.put((void*)newChar);
}

void processCommand(uint8_t* command) {
    switch (command[0]) {
        case 'K':
            newKey_mutex.lock();
            sscanf((char*)command, "K%x", &newKey);
            newKey_mutex.unlock();
            break;
        case 'R':
            break;
        case 'V':
            break;
    }
}

void commDecodeFn() {
    pc.attach(&serialISR);
    uint8_t commandArr[128];
    unsigned charNo = 0;
    while(1) {
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t) newEvent.value.p;
        if (newChar == '\r') {
            commandArr[charNo] = '\0';
            charNo = 0;
            processCommand(commandArr);
        }
        else
        {
            commandArr[charNo++] = newChar;
        }
    }
}

//Set a given drive state
void motorOut(int8_t driveState){

    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;

    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
    }

int rotorState;

    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
}

//Basic synchronisation routine
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(2.0);

    //Get the rotor state
    return readRotorState();
}

// Photointerruptor service routine
void photoISR() {
    int8_t intState = 0;
    int8_t intStateOld = 0;

    intState = readRotorState();
    if (intState != intStateOld) {
        intStateOld = intState;
        motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
    }
}

//Main
int main() {
    commOutT.start(commOutFn);
    commDecodeT.start(commDecodeFn);

    putMessage(hello, 0);

    //Run the motor synchronisation
    orState = motorHome();
    putMessage(rotorOrigin, orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    // Attach the photointerruptors to the service routine
    I1.rise(&photoISR);
    I1.fall(&photoISR);
    I2.rise(&photoISR);
    I2.fall(&photoISR);
    I3.rise(&photoISR);
    I3.fall(&photoISR);

    SHA256 sha;
    int numHashes = 0;
    Timer hashTimer;

    hashTimer.start();

    while (1) {
        newKey_mutex.lock();
        *key = newKey;
        newKey_mutex.unlock();

        sha.computeHash(hash, sequence, 64);
        numHashes++;

        if (hash[0] == 0 && hash[1] == 0) {
            // Successful hash
            putMessage(nonceFoundFirst, (uint32_t)(*nonce >> 32));
            putMessage(nonceFoundSecond, (uint32_t)(*nonce));
        } else {
            (*nonce)++;
        }

        if (hashTimer.read() > 1.0f) {
            putMessage(hashRate, numHashes);
            numHashes = 0;
            hashTimer.reset();
        }
    }
}
