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

Instruction decodeInstruction(uint32_t instruction) {
    Instruction result;

    result.opcode = (instruction >> 27) & 0x1F;  // Extracting the 5-bit opcode
    result.reg1 = (instruction >> 22) & 0x1F;    // Extracting the first register specifier
    result.reg2 = (instruction >> 17) & 0x1F;    // Extracting the second register specifier
    result.reg3 = (instruction >> 12) & 0x1F;    // Extracting the third register specifier
    result.literal = instruction & 0xFFF;        // Extracting the 12-bit literal

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
    Addr(int area, double power, int cycles) {
        this->area = area;
        this->power = power;
        this->numCycles = cycles;
    }

    // // Function for processing the data from input ports
    // void ProcessDataInput() {
    //     inputVal1 = *port1;
    //     inputVal2 = *port2;
    // }

    // Function for performing the device's main function
    void PerformFunction() {
        std::cout << "port1 value: " << inputPorts[0].getValue() << std::endl;
        std::cout << "port2 value: " << inputPorts[1].getValue() << std::endl;
        outputVal = inputPorts[0].getValue() + inputPorts[1].getValue();
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        (*outputLatch).setValue(outputVal);
    }

    // Function for reacting to control signals
    void OnControlSignal() {
        return;
    }

    // Function for updating the output latches
    void connectOutputLatches(Port* latch) {
        outputLatch = latch;
    }

    // Function for connecting input ports
    void ConnectInputPorts(int id, Port* port) {
        inputPorts[id] = *port;
    }

private:
    int area;
    int power;
    int numCycles;
    Port inputPorts[2];
    // int* port1;
    // int* port2;
    Port* outputLatch; // the latch is defined with port class because it serves the same functionality
    // int* outputLatch;
    int inputVal1;
    int inputVal2;
    long long outputVal;

};

/**
 * Multiplier Device
 */
class Multiplier : public Device {
public:
    Multiplier(int area, double power, int cycles) {
        this->area = area;
        this->power = power;
        this->numCycles = cycles;
    }

    void ProcessDataInput() {
        inputVal1 = *port1;
        inputVal2 = *port2;
    }

    // Function for performing the device's main function
    void PerformFunction() {
        outputVal = inputVal1 * inputVal2;
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        *outputLatch = outputVal;
    }


    // Function for reacting to control signals
    void OnControlSignal() {
        PerformFunction();
        return;
    }

    // Function for updating the output latches
    void connectOutputLatches(int** latches) {
        outputLatch = latches[0];
    }

    // Function for connecting input ports
    void ConnectInputPorts(int** ports) {
        port1 = ports[0];
        port2 = ports[1];
    }

private:
    int area;
    int power;
    int numCycles;
    int* port1;
    int* port2;
    int* outputLatch;
    int inputVal1;
    int inputVal2;
    int outputVal;

};

/**
 * Divider Device
 */
class Divider : public Device {
public:
    Divider(int area, double power, int cycles) {
        this->area = area;
        this->power = power;
        this->numCycles = cycles;
    }

    void ProcessDataInput() {
        inputVal1 = *port1;
        inputVal2 = *port2;
    }

    // Function for performing the device's main function
    void PerformFunction() {
        outputVal = inputVal1 / inputVal2;
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        *outputLatch = outputVal;
    }


    // Function for reacting to control signals
    void OnControlSignal() {
        return;
    }

    // Function for updating the output latches
    void connectOutputLatches(int** latches) {
        outputLatch = latches[0];
    }

    // Function for connecting input ports
    void ConnectInputPorts(int** ports) {
        port1 = ports[0];
        port2 = ports[1];
    }

private:
    int area;
    int power;
    int numCycles;
    int* port1;
    int* port2;
    int* outputLatch;
    int inputVal1;
    int inputVal2;
    int outputVal;

};

/**
 * Shifter Device
 */
class Shifter : public Device {
public:
    Shifter(int area, double power, int cycles) {
        this->area = area;
        this->power = power;
        this->numCycles = cycles;
    }

    // Function for processing the data from input ports
    void ProcessDataInput() {
        inputVal1 = *port1;
        inputVal2 = *port2;
    }

    // Function for performing the device's main function
    void PerformFunction() {
        if (shiftDirection == 0) {
            outputVal = inputVal1 >> inputVal2;
        }
        else {
            outputVal = inputVal1 << inputVal2;
        }
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        *outputLatch = outputVal;
    }

    // Function for reacting to control signals
    void OnControlSignal(int signal) {
        shiftDirection = signal;
        PerformFunction();
    }

    // Function for updating the output latches
    void connectOutputLatches(int** latches) {
        outputLatch = latches[0];
    }

    // Function for connecting input ports
    void ConnectInputPorts(int** ports) {
        port1 = ports[0];
        port2 = ports[1];
    }

private:
    int area;
    int power;
    int numCycles;
    int shiftDirection; // 0 -> right shift, 1 -> left shift
    int* port1;
    int* port2;
    int* outputLatch;
    int inputVal1;
    int inputVal2;
    int outputVal;

};

/**
 * Logic Device (under construction)
 */
class Logic : public Device {
public:
    Logic(int area, double power, int cycles) {
        this->area = area;
        this->power = power;
        this->numCycles = cycles;
    }

    // Function for processing the data from input ports
    void ProcessDataInput() {
        inputVal1 = *port1;
        inputVal2 = *port2;
    }

    // Function for performing the device's main function
    void PerformFunction() {
        switch  (logicalFunction) {
            case 0: // NOT operation
                outputVal = ~inputVal1;
                break;

            case 1: // AND operation
                outputVal = inputVal1 & inputVal2;
                break;

            case 2: // OR operation
                outputVal = inputVal1 | inputVal2;
                break;

            case 3: // XOR operation
                outputVal = inputVal1 ^ inputVal2;
                break;
        }
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        *outputLatch = outputVal;
    }

    // Function for reacting to control signals
    void OnControlSignal(int signal) {
        logicalFunction = signal;
    }

    // Function for updating the output latches
    void connectOutputLatches(int** latches) {
        outputLatch = latches[0];
    }

    // Function for connecting input ports
    void ConnectInputPorts(int** ports) {
        port1 = ports[0];
        port2 = ports[1];
    }

private:
    int area;
    int power;
    int numCycles;
    int logicalFunction; // 0 -> NOT, 1 -> AND, 2 -> OR, 3 -> XOR
    int* port1;
    int* port2;
    int* outputLatch;
    int inputVal1;
    int inputVal2;
    int outputVal;

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

    void ProcessDataInput(long long val) {
        value = val;
    }

    // Function for performing the device's main function
    void PerformFunction() {
        return;
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        *outputLatch = value;
    }

    // Function for reacting to control signals
    long long OnControlSignal() {
        return value;
    }

    // Function for updating the output latches
    void connectOutputLatches(int** latches) {
        outputLatch = latches[0];
    }

    // Function for connecting input ports
    void ConnectInputPorts(int** ports) {
        port = ports[0];
    }

private:
    double area;
    double power;
    double numCycles;
    long long value;
    int* port;
    int* outputLatch;

};

/**
 * Register File
 */
class RegisterFile : public Device {
public:
    RegisterFile(int area, double power, int cycles) {
        this->area = area;
        this->power = power;
        this->numCycles = cycles;
        for (int i = 0; i < 32; i++) {
            registers.push_back(Register(200, 0.05, 0.5));
        }
    }

    void ProcessDataInput(int addr, long long val) {
        registers[addr].ProcessDataInput(val);
    }

    // Function for performing the device's main function
    void PerformFunction() {
        return;
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        *outputLatch = value;
    }

    // Function for reacting to control signals
    void OnControlSignal(int addr1, int addr2) {
        value1 = registers[addr1].OnControlSignal();
        value2 = registers[addr1].OnControlSignal();
    }

    // Function for updating the output latches
    void connectOutputLatches(int** latches) {
        outputLatch = latches[0];
    }

    // Function for connecting input ports
    void ConnectInputPorts(int** ports) {
        port = ports[0];
    }

private:
    int area;
    int power;
    int numCycles;
    std::vector<Register> registers;
    long long value;
    int* port;
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

    // int p1 = 16;
    // int p2 = 2;
    // int* ports[] = {&p1, &p2};
    // int latch;
    // int* latches[] = {&latch};
    // Shifter shifter(200, 0.5, 2);
    // shifter.ConnectInputPorts(ports);
    // shifter.connectOutputLatches(latches);
    // shifter.OnControlSignal(1);
    // shifter.ProcessDataInput();
    // shifter.PerformFunction();
    // shifter.OnClockSignal();
    // std::cout << "result: " << **latches << std::endl;

    uint32_t encodedInstruction = 0x483000;
    Instruction decoded = decodeInstruction(encodedInstruction);

    // Print each field of the decoded instruction
    std::cout << "Opcode: " << decoded.opcode << std::endl;
    std::cout << "Register 1: " << decoded.reg1 << std::endl;
    std::cout << "Register 2: " << decoded.reg2 << std::endl;
    std::cout << "Register 3: " << decoded.reg3 << std::endl;
    std::cout << "Literal: " << decoded.literal << std::endl;

    int p1 = decoded.reg2;
    int p2 = decoded.reg3;
    Port port1;
    Port port2;
    port1.setValue(decoded.reg2);
    port2.setValue(decoded.reg3);
    // ports[0] = &p1;
    // ports[1] = &p2;
    // latches[0] = &latch;
    Port latch;
    Addr addr(400, 0.5, 1);
    addr.ConnectInputPorts(0, &port1);
    addr.ConnectInputPorts(1, &port2);
    addr.connectOutputLatches(&latch);
    // addr.ProcessDataInput();
    addr.PerformFunction();
    addr.OnClockSignal();
    std::cout << "result: " << latch.getValue() << std::endl;

    return 0;
}