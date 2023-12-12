#include <iostream>
#include <cstdint>

#include "AbstractDevice.h"


struct Instruction {
    uint32_t opcode;
    uint32_t reg1;
    uint32_t reg2;
    uint32_t reg3;
    uint32_t literal;
};

Instruction ControlArray(uint32_t instruction) {
    Instruction result;

    result.opcode = (instruction >> 27) & 0x1F;  // Extracting the 5-bit opcode
    result.reg1 = (instruction >> 22) & 0x1F;    // Extracting the first register specifier
    result.reg2 = (instruction >> 17) & 0x1F;    // Extracting the second register specifier
    result.reg3 = (instruction >> 12) & 0x1F;    // Extracting the third register specifier
    result.literal = instruction & 0xFFF;        // Extracting the 12-bit literal


    switch(result.opcode){
        case 0b00000:
            // set control signals and pass function for "add r1 r2 r3"
            break;
        case 0b00001:
            // set control signals and pass function for "addi r1 L"
        case 0b00010:
            // set control signals and pass function for "sub r1 r2 r3"
        case 0b00011:
            // set control signals and pass function for "subi r1 L"
        case 0b00100:
            // set control signals and pass function for "mul r1 r2 r3"
        case 0b00101:
            // set control signals and pass function for "div r1 r2 r3"
        case 0b00110:
            // set control signals and pass function for "and r1 r2 r3"
        case 0b00111:
            // set control signals and pass function for "or r1 r2 r3"
        case 0b01000:
            // set control signals and pass function for "xor r1 r2 r3"
        case 0b01001:
            // set control signals and pass function for "not r1 r2" r1=~r3
        case 0b01010:
            // set control signals and pass function for "shftr r1 r2 r3" r1=r2>>r3
        case 0b01011:
            // set control signals and pass function for "shftri r1 L"
        case 0b01100:
            // set control signals and pass function for "shftl r1 r2 r3"
        case 0b01101:
            // set control signals and pass function for "shftli r1 L"
        case 0b01110:
            // set control signals and pass function for "br r1" pc=r1
        case 0b01111:
            // set control signals and pass function for "brr r1" pc = pc + 1
        case 0b10000:
            // set control signals and pass function for "brr L" pc = pc + L
        case 0b10001:
            // set control signals and pass function for "brnz r1 r2" pc = (r2==0)? pc+4 : r1
        case 0b10010:
            // set control signals and pass function for "call r1" mem[r31-8]= pc+4 also pc= r1
        case 0b10011:
            // set control signals and pass function for "return" pc= mem[r31-8]
        case 0b10100:
            // set control signals and pass function for "halt"
        case 0b10101:
            // set control signals and pass function for "mov r1  r2(L)" r1=mem[r2+L]
        case 0b10110:
            // set control signals and pass function for "mov r1 r2" r1=r2
        case 0b10111:
            // set control signals and pass function for "mov r1 L" r1[52:63] = L
        case 0b11000:
            // set control signals and pass function for "mov r1(L) r2" mem[r1+l] = r2
        case 0b11001:
            // set control signals and pass function for "addf r1 r2 r3"
        case 0b11010:
            // set control signals and pass function for "subf r1 r2 r3"
        case 0b11011:
            // set control signals and pass function for "mulf r1 r2 r3"
        case 0b11100:
            // set control signals and pass function for "divf r1 r2 r3"
        case 0b11101:
            // set control signals and pass function for "input r1 r2" r1=input[r2]
        case 0b11110:
            // set control signals and pass function for "out r1 r2" output[r1]=r2
            break;
    }

    return result;
}

/**
 * A simple port class with a setter and getter.
 */
class Port {
public:
    void setValue(long long val) {
        value = val;
    }

    long long getValue() {
        return value;
    }

private:
    long long value;
};

/**
 * Adder Device
 */
class Addr : public Device {
public:
    Addr() {
        this->area = 400;
        this->power = 0.5;
        this->numCycles = 1;
    }

    // Function for performing the device's main function
    void PerformFunction() {
        std::cout << "port1 value: " << inputPorts[0]->getValue() << std::endl;
        std::cout << "port2 value: " << inputPorts[1]->getValue() << std::endl;
        outputVal = inputPorts[0]->getValue() + inputPorts[1]->getValue();
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
        (*outputLatch).setValue(outputVal);
    }

    // Function for reacting to control signals
    void OnControlSignal() {
        return;
    }

    // Function for connecting the output latches
    void connectOutputLatches(Port* latch) {
        outputLatch = latch;
    }

    // Function for connecting input ports
    void ConnectInputPorts(int id, Port* port) {
        inputPorts[id] = port;
    }

private:
    double area;
    double power;
    double numCycles;
    Port* inputPorts[2];
    Port* outputLatch; // the latch is defined with port class because it serves the same functionality
    long long outputVal;

};

/**
 * Shifter Device
 */
class Shifter : public Device {
public:
    Shifter() {
        this->area = 200;
        this->power = 0.5;
        this->numCycles = 1;
    }

    // Function for performing the device's main function
    void PerformFunction() {

        std::cout << "port1 value: " << inputPorts[0]->getValue() << std::endl;
        std::cout << "port2 value: " << inputPorts[1]->getValue() << std::endl;
        if (shiftDirection == 0) {
            outputVal = inputPorts[0]->getValue() >> inputPorts[1]->getValue();
        }
        else {
            outputVal = inputPorts[0]->getValue() << inputPorts[1]->getValue();
        }
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
        (*outputLatch).setValue(outputVal);
    }

    // Function for reacting to control signals
    void OnControlSignal(int signal) {
        shiftDirection = signal;
    }

    // Function for connecting the output latches
    void connectOutputLatches(Port* latch) {
        outputLatch = latch;
    }

    // Function for connecting input ports
    void ConnectInputPorts(int id, Port* port) {
        inputPorts[id] = port;
    }

private:
    double area;
    double power;
    double numCycles;
    int shiftDirection; // 0 -> right shift, 1 -> left shift
    Port* inputPorts[2];
    Port* outputLatch; // the latch is defined with port class because it serves the same functionality
    long long outputVal;

};

/**
 * Multiplier Device
 */
class Multiplier : public Device {
public:
    Multiplier() {
        this->area = 2000;
        this->power = 1.5;
        this->numCycles = 3;
    }

    // Function for performing the device's main function
    void PerformFunction() {
        std::cout << "port1 value: " << inputPorts[0]->getValue() << std::endl;
        std::cout << "port2 value: " << inputPorts[1]->getValue() << std::endl;
        outputVal = inputPorts[0]->getValue() * inputPorts[1]->getValue();
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
        (*outputLatch).setValue(outputVal);
    }


    // Function for reacting to control signals
    void OnControlSignal() {
        return;
    }

    // Function for connecting the output latches
    void connectOutputLatches(Port* latch) {
        outputLatch = latch;
    }

    // Function for connecting input ports
    void ConnectInputPorts(int id, Port* port) {
        inputPorts[id] = port;
    }

private:
    double area;
    double power;
    double numCycles;
    Port* inputPorts[2];
    Port* outputLatch; // the latch is defined with port class because it serves the same functionality
    long long outputVal;

};

/**
 * Divider Device
 */
class Divider : public Device {
public:
    Divider() {
        this->area = 5000;
        this->power = 1;
        this->numCycles = 8;
    }

    // Function for performing the device's main function
    void PerformFunction() {
        std::cout << "port1 value: " << inputPorts[0]->getValue() << std::endl;
        std::cout << "port2 value: " << inputPorts[1]->getValue() << std::endl;
        outputVal = inputPorts[0]->getValue() / inputPorts[1]->getValue();
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
        (*outputLatch).setValue(outputVal);
    }


    // Function for reacting to control signals
    void OnControlSignal() {
        return;
    }

    // Function for connecting the output latches
    void connectOutputLatches(Port* latch) {
        outputLatch = latch;
    }

    // Function for connecting input ports
    void ConnectInputPorts(int id, Port* port) {
        inputPorts[id] = port;
    }

private:
    double area;
    double power;
    double numCycles;
    Port* inputPorts[2];
    Port* outputLatch; // the latch is defined with port class because it serves the same functionality
    long long outputVal;

};


/**
 * Logic Device (under construction)
 */
class Logic : public Device {
public:
    Logic() {
        this->area = 600;
        this->power = 0.75;
        this->numCycles = 1;
    }

    // Function for performing the device's main function
    void PerformFunction() {
        switch  (logicalFunction) {
            case 0: // NOT operation
                outputVal = ~inputPorts[0]->getValue();
                break;

            case 1: // AND operation
                outputVal = inputPorts[0]->getValue() & inputPorts[1]->getValue();
                break;

            case 2: // OR operation
                outputVal = inputPorts[0]->getValue() | inputPorts[1]->getValue();
                break;

            case 3: // XOR operation
                outputVal = inputPorts[0]->getValue() ^ inputPorts[1]->getValue();
                break;
        }
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
        (*outputLatch).setValue(outputVal);
    }

    // Function for reacting to control signals
    void OnControlSignal(int signal) {
        logicalFunction = signal;
    }

    // Function for connecting the output latches
    void connectOutputLatches(Port* latch) {
        outputLatch = latch;
    }

    // Function for connecting input ports
    void ConnectInputPorts(int id, Port* port) {
        inputPorts[id] = port;
    }

private:
    double area;
    double power;
    double numCycles;
    int logicalFunction; // 0 -> NOT, 1 -> AND, 2 -> OR, 3 -> XOR
    Port* inputPorts[2];
    Port* outputLatch; // the latch is defined with port class because it serves the same functionality
    long long outputVal;

};


/**
 * Comparator Device (under construction)
 */
class Comparator : public Device {
public:
    Comparator() {
        this->area = 400;
        this->power = 0.5;
        this->numCycles = 1;
    }

    // Function for performing the device's main function
    void PerformFunction() {
        outputVal = ~(inputPorts[0]->getValue() == inputPorts[1]->getValue());
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
        (*outputLatch).setValue(outputVal);
    }

    // Function for reacting to control signals
    void OnControlSignal() {
        return;
    }

    // Function for connecting the output latches
    void connectOutputLatches(Port* latch) {
        outputLatch = latch;
    }

    // Function for connecting input ports
    void ConnectInputPorts(int id, Port* port) {
        inputPorts[id] = port;
    }

private:
    double area;
    double power;
    double numCycles;
    Port* inputPorts[2];
    Port* outputLatch; // the latch is defined with port class because it serves the same functionality
    long long outputVal;

};


/**
 * Comparator Device (under construction)
 */
class TwosComplement : public Device {
public:
    TwosComplement() {
        this->area = 200;
        this->power = 0.25;
        this->numCycles = 1;
    }

    // Function for performing the device's main function
    void PerformFunction() {
        outputVal = (~inputPorts[0]->getValue())+1; // this would only work if this was in bits
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
        (*outputLatch).setValue(outputVal);
    }

    // Function for reacting to control signals
    void OnControlSignal() {
        return;
    }

    // Function for connecting the output latches
    void connectOutputLatches(Port* latch) {
        outputLatch = latch;
    }

    // Function for connecting input ports
    void ConnectInputPorts(int id, Port* port) {
        inputPorts[id] = port;
    }

private:
    double area;
    double power;
    double numCycles;
    Port* inputPorts[1];
    Port* outputLatch; // the latch is defined with port class because it serves the same functionality
    long long outputVal;

};


/**
 * Multiplexer Device
 */
class Multiplexer : public Device {
public:
    Multiplexer() {
        this->area = 500;
        this->power = 0.25;
        this->numCycles = 0.5;
    }

    // Function for performing the device's main function
    void PerformFunction() {
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        outputLatch->setValue(outputVal);
    }

    // Function for reacting to control signals
    void OnControlSignal(int signal) {
        switch (signal) {
            case 0:
                outputVal = inputPorts[0]->getValue();

                break;
            case 1:
                outputVal = inputPorts[1]->getValue();

                break;
            case 2:
                outputVal = inputPorts[2]->getValue();
                break;
            case 3:
                outputVal = inputPorts[3]->getValue();
                break;
        }
    }

    // Function for connecting the output latches
    void connectOutputLatches(Port* latch) {
        outputLatch = latch;
    }

    // Function for connecting input ports
    void ConnectInputPorts(int id, Port* port) {
        inputPorts[id] = port;
    }

private:
    double area;
    double power;
    double numCycles;
    Port* inputPorts[4];
    Port* outputLatch; // the latch is defined with port class because it serves the same functionality
    long long outputVal;

};

/**
 * Demultiplexer Device
 */
class Demultiplexer : public Device {
public:
    Demultiplexer() {
        this->area = 500;
        this->power = 0.25;
        this->numCycles = 0.5;
    }

    // Function for performing the device's main function
    void PerformFunction() {
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
    }

    // Function for reacting to control signals
    void OnControlSignal(int signal) {
        long long inputVal = inputPort->getValue();
        switch (signal) {
            case 0:
                outputLatch[0]->setValue(inputVal);

                break;
            case 1:
                outputLatch[1]->setValue(inputVal);

                break;
            case 2:
                outputLatch[2]->setValue(inputVal);
                break;
            case 3:
                outputLatch[3]->setValue(inputVal);
                break;
        }
    }

    // Function for connecting the output latches
    void connectOutputLatches(int id, Port* latch) {
        outputLatch[id] = latch;
    }

    // Function for connecting input ports
    void ConnectInputPorts(Port* port) {
        inputPort = port;
    }

    private:
    double area;
    double power;
    double numCycles;
    Port* inputPort;
    Port* outputLatch[4]; // the latch is defined with port class because it serves the same functionality
    long long outputVal;

};


/**
 * Add4 Device
 */
class Add4 : public Device {
public:
    Add4() {
        this->area = 100;
        this->power = 0.1;
        this->numCycles = 1;
    }

    // Function for performing the device's main function
    void PerformFunction() {
        std::cout << "port1 value: " << inputPorts[0]->getValue() << std::endl;
        outputVal = inputPorts[0]->getValue() + 4;
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
        outputLatch->setValue(outputVal);
    }

    // Function for reacting to control signals
    void OnControlSignal() {
        return;
    }

    // Function for connecting the output latches
    void connectOutputLatches(Port* latch) {
        outputLatch = latch;
    }

    // Function for connecting input ports
    void ConnectInputPorts(int id, Port* port) {
        inputPorts[id] = port;
    }

private:
    double area;
    double power;
    double numCycles;
    Port* inputPorts[1];
    Port* outputLatch; // the latch is defined with port class because it serves the same functionality
    int outputVal;

};


/**
 * Control Array Device
 */
class ControlArray : public Device {
public:
    ControlArray() {
        this->area = 500;
        this->power = 0.25;
        this->numCycles = 0.5;
    }

    // Function for performing the device's main function
    void PerformFunction() {
        Instruction result;
        uint32_t instruction = inputPorts[0]->getValue();

        result.opcode = (instruction >> 27) & 0x1F;  // Extracting the 5-bit opcode
        result.reg1 = (instruction >> 22) & 0x1F;    // Extracting the first register specifier
        result.reg2 = (instruction >> 17) & 0x1F;    // Extracting the second register specifier
        result.reg3 = (instruction >> 12) & 0x1F;    // Extracting the third register specifier
        result.literal = instruction & 0xFFF;        // Extracting the 12-bit literal


        switch(result.opcode){
            case 0b00000:
                // set control signals and pass function for "add r1 r2 r3"
                break;
            case 0b00001:
                // set control signals and pass function for "addi r1 L"
            case 0b00010:
                // set control signals and pass function for "sub r1 r2 r3"
            case 0b00011:
                // set control signals and pass function for "subi r1 L"
            case 0b00100:
                // set control signals and pass function for "mul r1 r2 r3"
            case 0b00101:
                // set control signals and pass function for "div r1 r2 r3"
            case 0b00110:
                // set control signals and pass function for "and r1 r2 r3"
            case 0b00111:
                // set control signals and pass function for "or r1 r2 r3"
            case 0b01000:
                // set control signals and pass function for "xor r1 r2 r3"
            case 0b01001:
                // set control signals and pass function for "not r1 r2" r1=~r3
            case 0b01010:
                // set control signals and pass function for "shftr r1 r2 r3" r1=r2>>r3
            case 0b01011:
                // set control signals and pass function for "shftri r1 L"
            case 0b01100:
                // set control signals and pass function for "shftl r1 r2 r3"
            case 0b01101:
                // set control signals and pass function for "shftli r1 L"
            case 0b01110:
                // set control signals and pass function for "br r1" pc=r1
            case 0b01111:
                // set control signals and pass function for "brr r1" pc = pc + 1
            case 0b10000:
                // set control signals and pass function for "brr L" pc = pc + L
            case 0b10001:
                // set control signals and pass function for "brnz r1 r2" pc = (r2==0)? pc+4 : r1
            case 0b10010:
                // set control signals and pass function for "call r1" mem[r31-8]= pc+4 also pc= r1
            case 0b10011:
                // set control signals and pass function for "return" pc= mem[r31-8]
            case 0b10100:
                // set control signals and pass function for "halt"
            case 0b10101:
                // set control signals and pass function for "mov r1  r2(L)" r1=mem[r2+L]
            case 0b10110:
                // set control signals and pass function for "mov r1 r2" r1=r2
            case 0b10111:
                // set control signals and pass function for "mov r1 L" r1[52:63] = L
            case 0b11000:
                // set control signals and pass function for "mov r1(L) r2" mem[r1+l] = r2
            case 0b11001:
                // set control signals and pass function for "addf r1 r2 r3"
            case 0b11010:
                // set control signals and pass function for "subf r1 r2 r3"
            case 0b11011:
                // set control signals and pass function for "mulf r1 r2 r3"
            case 0b11100:
                // set control signals and pass function for "divf r1 r2 r3"
            case 0b11101:
                // set control signals and pass function for "input r1 r2" r1=input[r2]
            case 0b11110:
                // set control signals and pass function for "out r1 r2" output[r1]=r2
                break;
        }
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        outputLatch->setValue(outputVal);
    }


    // Function for connecting the output latches
    void connectOutputLatches(Port* latch) {
        outputLatch = latch;
    }

    // Function for connecting input ports
    void ConnectInputPorts(int id, Port* port) {
        inputPorts[id] = port;
    }

private:
    double area;
    double power;
    double numCycles;
    Port* inputPorts[1];
    Port* outputLatch; // the latch is defined with port class because it serves the same functionality
    long long outputVal;

};


/**
 * Register Device
 */
class Register : public Device {
public:
    Register(double area, double power, double cycles) {
        this->area = area;
        this->power = power;
        this->numCycles = cycles;
    }

    // Function for performing the device's main function
    void PerformFunction() {
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
    }

    // Function for reacting to control signals (signal meaning: 0 -> read, 1 -> write)
    void OnControlSignal(int signal) {
        if (signal == 0) {
            outputLatch->setValue(value);
        }
        else if (signal == 1) {
            value = inputPort->getValue();
        }
    }

    // Function for connecting the output latches
    void connectOutputLatches(Port* latch) {
        outputLatch = latch;
    }

    // Function for connecting input ports
    void ConnectInputPorts(Port* port) {
        inputPort = port;
    }

private:
    double area;
    double power;
    double numCycles;
    long long value;
    Port* inputPort;
    Port* outputLatch;

};


/**
 * Register File
 */
class RegisterFile : public Device {
public:
    RegisterFile() {
        this->area = 20000;
        this->power = 4;
        this->numCycles = 1;
        for (int i = 0; i < 32; i++) {
            registers.push_back(Register(200, 0.05, 0.5));
        }
    }

    // Function for performing the device's main function
    void PerformFunction() {
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
    }

    // Function for reacting to control signals
    void OnControlSignal(int signal) {
        int address1;
        int address2;
        switch (signal) {
            case 0:
                break;
            case 1:
                address1 = inputPorts[0]->getValue();
                registers[address1].connectOutputLatches(outputLatches[0]);
                registers[address1].OnControlSignal(0);
                break;
            case 2:
                address1 = inputPorts[0]->getValue();
                address2 = inputPorts[1]->getValue();
                registers[address1].connectOutputLatches(outputLatches[0]);
                registers[address2].connectOutputLatches(outputLatches[1]);
                registers[address1].OnControlSignal(0);
                registers[address2].OnControlSignal(0);
                break;
            case 3:
                address1 = inputPorts[1]->getValue();
                registers[address1].ConnectInputPorts(inputPorts[0]);
                registers[address1].OnControlSignal(1);
                break;
        }
    }
    void OnClockSignal(int signal) {
        switch (signal) {
            case 0:
                break;
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
        }
    }

    // Function for updating the output latches
    // Function for connecting the output latches
    void connectOutputLatches(int id, Port* latch) {
        outputLatches[id] = latch;
    }
    // Function for connecting input ports
    void ConnectInputPorts(int id, Port* port) {
        inputPorts[id] = port;
    }

private:
    double area;
    double power;
    double numCycles;
    std::vector<Register> registers;
    long long value;
    int* port;
    Port* inputPorts[2];
    Port* outputLatches[2];
    int* outputLatch;
    int* outputLatch2;
    long long value1;
    long long value2;

};

/**
 * Multi Ported Register File
 */
class MultiPortedRegisterFile : public Device {
public:
    MultiPortedRegisterFile() {
        this->area = 25000;
        this->power = 6;
        this->numCycles = 1;
        for (int i = 0; i < 32; i++) {
            registers.push_back(Register(200, 0.05, 0.5));
        }
    }

    // Function for performing the device's main function
    void PerformFunction() {
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
    }

    // Function for reacting to control signals
    void OnControlSignal(int signal) {
        int address1;
        int address2;
        int address3;
        int address4;
        switch (signal) {
            case 0: // 0x00 NOP
                break;
            case 1: // 0x01 R
                address1 = inputPorts[0]->getValue();
                registers[address1].connectOutputLatches(outputLatches[0]);
                registers[address1].OnControlSignal(0);
                break;
            case 2: // 0x10 RR
                address1 = inputPorts[0]->getValue();
                address2 = inputPorts[1]->getValue();
                registers[address1].connectOutputLatches(outputLatches[0]);
                registers[address2].connectOutputLatches(outputLatches[1]);
                registers[address1].OnControlSignal(0);
                registers[address2].OnControlSignal(0);
                break;
            case 3: // 0x11 W
                address1 = inputPorts[1]->getValue();
                registers[address1].ConnectInputPorts(inputPorts[0]);
                registers[address1].OnControlSignal(1);
                break;
            case 4: // 0x100 RRR
                // 3 reads
                address1 = inputPorts[0]->getValue();
                address2 = inputPorts[1]->getValue();
                address3 = inputPorts[2]->getValue();
                registers[address1].connectOutputLatches(outputLatches[0]);
                registers[address2].connectOutputLatches(outputLatches[1]);
                registers[address3].connectOutputLatches(outputLatches[3]);
                registers[address1].OnControlSignal(0);
                registers[address2].OnControlSignal(0);
                registers[address3].OnControlSignal(0);
                break;
            case 5: // 0x101 RRRR
                // 4 reads
                address1 = inputPorts[0]->getValue();
                address2 = inputPorts[1]->getValue();
                address3 = inputPorts[2]->getValue();
                registers[address1].connectOutputLatches(outputLatches[0]);
                registers[address2].connectOutputLatches(outputLatches[1]);
                registers[address3].connectOutputLatches(outputLatches[3]);
                registers[address1].OnControlSignal(0);
                registers[address2].OnControlSignal(0);
                registers[address3].OnControlSignal(0);
                break;
            case 6: // 0x110 RRW
                // 2 reads
                address1 = inputPorts[0]->getValue();
                address2 = inputPorts[1]->getValue();
                registers[address1].connectOutputLatches(outputLatches[0]);
                registers[address2].connectOutputLatches(outputLatches[1]);
                registers[address1].OnControlSignal(0);
                registers[address2].OnControlSignal(0);
                // 1 write
                address3 = inputPorts[3]->getValue();
                registers[address1].ConnectInputPorts(inputPorts[2]);
                registers[address1].OnControlSignal(1);
                break;
            case 7: // 0x111 WW
                // Perform first write
                address1 = inputPorts[1]->getValue();
                registers[address1].ConnectInputPorts(inputPorts[0]);
                registers[address1].OnControlSignal(1);
                // Perform second write
                address2 = inputPorts[3]->getValue();
                registers[address1].ConnectInputPorts(inputPorts[2]);
                registers[address1].OnControlSignal(1);
        }
    }
    void OnClockSignal(int signal) {
        switch (signal) {
            case 0:
                break;
            case 1:
                break;
            case 2:
                break;
            case 3:
                break;
        }
    }

    // Function for connecting the output latches
    void connectOutputLatches(int id, Port* latch) {
        outputLatches[id] = latch;
    }
    // Function for connecting input ports
    void ConnectInputPorts(int id, Port* port) {
        inputPorts[id] = port;
    }

private:
    double area;
    double power;
    double numCycles;
    std::vector<Register> registers;
    long long value;
    int* port;
    Port* inputPorts[4];
    Port* outputLatches[4];
    int* outputLatch;
    int* outputLatch2;
    long long value1;
    long long value2;

};


class Processor {
public:
    Processor() {

    }
private:
};


int main() {
    std::cout << "Hello, World!" << std::endl;

    uint32_t encodedInstruction = 0x483000;
    Instruction decoded = decodeInstruction(encodedInstruction);

    // Print each field of the decoded instruction
    std::cout << "Opcode: " << decoded.opcode << std::endl;
    std::cout << "Register 1: " << decoded.reg1 << std::endl;
    std::cout << "Register 2: " << decoded.reg2 << std::endl;
    std::cout << "Register 3: " << decoded.reg3 << std::endl;
    std::cout << "Literal: " << decoded.literal << std::endl;

    RegisterFile registerFile;
    Port port1;
    Port port2;
    Port latch1;
    Port latch2;

    registerFile.ConnectInputPorts(0, &port1);
    registerFile.ConnectInputPorts(1, &port2);

    // write value 4 to register at adddress 4
    port1.setValue(4);
    port2.setValue(decoded.reg2);
    registerFile.OnControlSignal(3);

    // write value 3 to register at adddress 3
    port1.setValue(decoded.reg3);
    port2.setValue(3);
    registerFile.OnControlSignal(3);


    // read values at register 4 and 3
    port1.setValue(decoded.reg2);
    port2.setValue(decoded.reg3);
    registerFile.connectOutputLatches(0, &latch1);
    registerFile.connectOutputLatches(1, &latch2);
    registerFile.OnControlSignal(2);
    std::cout << "registr value: " << latch1.getValue() << std::endl;
    std::cout << "registr value: " << latch2.getValue() << std::endl;


    // add values in register 4 and 3
    Addr addr;
    Port latch;
    addr.ConnectInputPorts(0, &port1);
    addr.ConnectInputPorts(1, &port2);
    addr.connectOutputLatches(&latch);
    addr.OnClockSignal();
    std::cout << "result: " << latch.getValue() << std::endl;


    // multiply in register 4 and 3
    Multiplier multiply;
    multiply.ConnectInputPorts(0, &port1);
    multiply.ConnectInputPorts(1, &port2);
    multiply.connectOutputLatches(&latch);
    multiply.OnClockSignal();
    std::cout << "result: " << latch.getValue() << std::endl;

    return 0;
}