#include <iostream>
#include <cstdint>
#include <bitset>
#include <fstream>
#include <vector>
#include <string>
#include <iostream>
#include <cstdlib>

#include "AbstractDevice.h"

long long PC = 0;
int HALT = 0;
int TOTAL_CYCLES = 0;
int MULT_CYCLES = 0;
int DIV_CYCLES = 0;
int TOTAL_POWER = 0;


struct Instruction {
    uint32_t opcode;
    uint32_t reg1;
    uint32_t reg2;
    uint32_t reg3;
    uint32_t literal;
};

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
        inputPorts[0] = new Port();
        inputPorts[1] = new Port();
        outputLatches[0] = new Port();
    }

    // Function for performing the device's main function
    void PerformFunction() {
        outputVal = inputPorts[0]->getValue() + inputPorts[1]->getValue();
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
        outputLatches[0]->setValue(outputVal);
    }


public:
    Port* inputPorts[2];
    Port* outputLatches[1];
    double area;
    double power;
    double numCycles;

private:
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
        inputPorts[0] = new Port();
        inputPorts[1] = new Port();
        outputLatches[0] = new Port();
    }

    // Function for performing the device's main function
    void PerformFunction() {
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
        outputLatches[0]->setValue(outputVal);
    }

    // Function for reacting to control signals
    void OnControlSignal(int signal) {
        shiftDirection = signal;
    }


public:
    Port* inputPorts[2];
    Port* outputLatches[1];
    double numCycles;
    double area;
    double power;

private:
    int shiftDirection; // 0 -> right shift, 1 -> left shift
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
        inputPorts[0] = new Port();
        inputPorts[1] = new Port();
        outputLatches[0] = new Port();
    }

    // Function for performing the device's main function
    void PerformFunction() {
        outputVal = inputPorts[0]->getValue() * inputPorts[1]->getValue();
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
        outputLatches[0]->setValue(outputVal);
    }


public:
    Port* inputPorts[2];
    Port* outputLatches[1];
    double area;
    double power;
    double numCycles;

private:
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
        inputPorts[0] = new Port();
        inputPorts[1] = new Port();
        outputLatches[0] = new Port();
    }

    // Function for performing the device's main function
    void PerformFunction() {
        outputVal = inputPorts[0]->getValue() / inputPorts[1]->getValue();
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
        outputLatches[0]->setValue(outputVal);
    }


public:
    Port* inputPorts[2];
    Port* outputLatches[1];
    double area;
    double power;
    double numCycles;

private:
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
        inputPorts[0] = new Port();
        inputPorts[1] = new Port();
        outputLatches[0] = new Port();
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
        outputLatches[0]->setValue(outputVal);
    }

    // Function for reacting to control signals
    void OnControlSignal(int signal) {
        logicalFunction = signal;
    }


public:
    Port* inputPorts[2];
    Port* outputLatches[1];
    double area;
    double power;
    double numCycles;

private:
    int logicalFunction; // 0 -> NOT, 1 -> AND, 2 -> OR, 3 -> XOR
    long long outputVal;

};


/**
 * Comparator Device
 */
class Comparator : public Device {
public:
    Comparator() {
        this->area = 400;
        this->power = 0.5;
        this->numCycles = 1;
        inputPorts[0] = new Port();
        inputPorts[1] = new Port();
        outputLatches[0] = new Port();
    }

    // Function for performing the device's main function
    void PerformFunction() {
        outputVal = ~(inputPorts[0]->getValue() == inputPorts[1]->getValue());
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
        outputLatches[1]->setValue(outputVal);
    }


public:
    Port* inputPorts[2];
    Port* outputLatches[1];
    double area;
    double power;
    double numCycles;

private:
    long long outputVal;
};


/**
 * Two's Complement Device
 */
class TwosComplement : public Device {
public:
    TwosComplement() {
        this->area = 200;
        this->power = 0.25;
        this->numCycles = 1;
        inputPorts[0] = new Port();
        outputLatches[0] = new Port();
    }

    // Function for performing the device's main function
    void PerformFunction() {
        outputVal = -inputPorts[0]->getValue(); // We checked and C++ does take care of this.
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
        outputLatches[0]->setValue(outputVal);
    }


public:
    Port* inputPorts[1];
    Port* outputLatches[1];
    double area;
    double power;
    double numCycles;

private:
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
        inputPorts[0] = new Port();
        inputPorts[1] = new Port();
        inputPorts[2] = new Port();
        inputPorts[3] = new Port();
        outputLatches[0] = new Port();
    }

    // Function for performing the device's main function
    void PerformFunction() {
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        outputLatches[0]->setValue(outputVal);
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

public:
    Port* inputPorts[4];
    Port* outputLatches[1];
    double area;
    double power;
    double numCycles;

private:
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
        inputPorts[0] = new Port();
        outputLatches[0] = new Port();
        outputLatches[1] = new Port();
        outputLatches[2] = new Port();
        outputLatches[3] = new Port();
    }

    // Function for performing the device's main function
    void PerformFunction() {
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
    }

    // Function for reacting to control signals
    void OnControlSignal(int signal) {
        long long inputVal = inputPorts[0]->getValue();
        switch (signal) {
            case 0:
                outputLatches[0]->setValue(inputVal);
                break;
            case 1:
                outputLatches[1]->setValue(inputVal);
                break;
            case 2:
                outputLatches[2]->setValue(inputVal);
                break;
            case 3:
                outputLatches[3]->setValue(inputVal);
                break;
        }
    }


public:
    Port* inputPorts[1];
    Port* outputLatches[4];
    double area;
    double power;
    double numCycles;

private:
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
        inputPorts[0] = new Port();
        outputLatches[0] = new Port();
    }

    // Function for performing the device's main function
    void PerformFunction() {
        outputVal = inputPorts[0]->getValue() + 4;
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
        outputLatches[0]->setValue(outputVal);
    }

    // Function for reacting to control signals
    void OnControlSignal() {
        return;
    }


public:
    Port* inputPorts[1];
    Port* outputLatches[1];
    double area;
    double power;
    double numCycles;
private:
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

public:
    double area;
    double power;
    double numCycles;

private:
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
        inputPorts[0] = new Port();
        inputPorts[1] = new Port();
        outputLatches[0] = new Port();
        outputLatches[1] = new Port();
    }

    // Function for performing the device's main function
    void PerformFunction() {
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
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

    // Function for reacting to control signals
    void OnControlSignal(int signal) {
        this->signal = signal;
    }

    // Function for connecting the output latches
    void connectOutputLatches(int id, Port* latch) {
        outputLatches[id] = latch;
    }
    // Function for connecting input ports
    void ConnectInputPorts(int id, Port* port) {
        inputPorts[id] = port;
    }

public:
    Port* inputPorts[2];
    Port* outputLatches[2];
    double area;
    double power;
    double numCycles;

private:
    std::vector<Register> registers;
    int signal;
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
        for (int i = 0; i < 4; i++) {
            inputPorts[i] = new Port();
            outputLatches[i] = new Port();
        }
    }

    // Function for performing the device's main function
    void PerformFunction() {
    }

    // Function for reacting to control signals
    void OnControlSignal(int signal) {
        this->signal = signal;
    }

    // Function for reacting to the clock signal
    void OnClockSignal(int signal) {
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


public:
    Port* inputPorts[4];
    Port* outputLatches[4];
    double area;
    double power;
    double numCycles;

private:
    std::vector<Register> registers;
    int signal;
};


class InstructionExecute  {
public:
    InstructionExecute() {
    }

    void decreaseCycles(int cycles) {
        MULT_CYCLES -= cycles;
        DIV_CYCLES -= cycles;

    }

    static void addCycles(int cycles) {
        TOTAL_CYCLES += cycles;
    }

    static void addPowerConsumption(int power) {
        TOTAL_POWER += power;
    }

    // instruction 0x0
    void Instruction0x0(int register_d, int register_s, int register_t) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);
        registerFile.inputPorts[1]->setValue(register_t);

        // send clock signal to read registers
        registerFile.OnControlSignal(2);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect register file output latches to adder input ports
        addr.inputPorts[0] = registerFile.outputLatches[0];
        addr.inputPorts[1] = registerFile.outputLatches[1];

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = addr.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal to perform addition
        addr.OnClockSignal();
        addCycles(addr.numCycles);
        addPowerConsumption(addr.power);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0x1
    void Instruction0x1(int register_d, int immediate) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_d);

        // send clock signal to read register
        registerFile.OnControlSignal(1);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect register file output latches to adder input ports
        addr.inputPorts[0] = registerFile.outputLatches[0];
        addr.inputPorts[1]->setValue(immediate);

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = addr.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal to perform addition
        addr.OnClockSignal();
        addCycles(addr.numCycles);
        addPowerConsumption(addr.power);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0x2
    void Instruction0x2(int register_d, int register_s, int register_t) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);
        registerFile.inputPorts[1]->setValue(register_t);

        // send clock signal to read registers
        registerFile.OnControlSignal(2);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect register file output latches to adder input ports
        addr.inputPorts[0] = registerFile.outputLatches[0];
        long long temp = registerFile.outputLatches[1]->getValue(); // this is for the two's comp to do sub
        addr.inputPorts[1]->setValue(-temp);

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = addr.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal to perform addition
        addr.OnClockSignal();
        addCycles(addr.numCycles);
        addPowerConsumption(addr.power);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0x3
    void Instruction0x3(int register_d, int immediate) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_d);

        // send clock signal to read registers
        registerFile.OnControlSignal(1);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect register file output latches to adder input ports
        addr.inputPorts[0] = registerFile.outputLatches[0];
        addr.inputPorts[1]->setValue(-immediate);

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = addr.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal to perform addition
        addr.OnClockSignal();
        addCycles(addr.numCycles);
        addPowerConsumption(addr.power);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

     // instruction 0x4
    void Instruction0x4(int register_d, int register_s, int register_t) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);
        registerFile.inputPorts[1]->setValue(register_t);

        // send clock signal to read registers
        registerFile.OnControlSignal(2);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect register file output latches to adder input ports
        multiplier.inputPorts[0] = registerFile.outputLatches[0];
        multiplier.inputPorts[1] = registerFile.outputLatches[1];

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = multiplier.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal to perform addition
        multiplier.OnClockSignal();
        addCycles(multiplier.numCycles);
        addPowerConsumption(multiplier.power);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

     // instruction 0x5
    void Instruction0x5(int register_d, int register_s, int register_t) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);
        registerFile.inputPorts[1]->setValue(register_t);

        // send clock signal to read registers
        registerFile.OnControlSignal(2);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect register file output latches to adder input ports
        divider.inputPorts[0] = registerFile.outputLatches[0];
        divider.inputPorts[1] = registerFile.outputLatches[1];

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = divider.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal to perform addition
        divider.OnClockSignal();
        addCycles(divider.numCycles);
        addPowerConsumption(divider.power);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0x6 logical and
    void Instruction0x6(int register_d, int register_s, int register_t) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);
        registerFile.inputPorts[1]->setValue(register_t);

        // send clock signal to read registers
        registerFile.OnControlSignal(2);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect register file output latches to adder input ports
        logic.inputPorts[0] = registerFile.outputLatches[0];
        logic.inputPorts[1] = registerFile.outputLatches[1];

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = logic.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal to perform addition
        logic.OnControlSignal(1);
        logic.OnClockSignal();
        addCycles(logic.numCycles);
        addPowerConsumption(logic.power);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0x7 logical or
    void Instruction0x7(int register_d, int register_s, int register_t) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);
        registerFile.inputPorts[1]->setValue(register_t);

        // send clock signal to read registers
        registerFile.OnControlSignal(2);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect register file output latches to adder input ports
        logic.inputPorts[0] = registerFile.outputLatches[0];
        logic.inputPorts[1] = registerFile.outputLatches[1];

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = logic.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal to perform addition
        logic.OnControlSignal(2);
        logic.OnClockSignal();
        addCycles(logic.numCycles);
        addPowerConsumption(logic.power);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0x8 logical xor
    void Instruction0x8(int register_d, int register_s, int register_t) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);
        registerFile.inputPorts[1]->setValue(register_t);

        // send clock signal to read registers
        registerFile.OnControlSignal(2);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect register file output latches to adder input ports
        logic.inputPorts[0] = registerFile.outputLatches[0];
        logic.inputPorts[1] = registerFile.outputLatches[1];

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = logic.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal to perform addition
        logic.OnControlSignal(3);
        logic.OnClockSignal();
        addCycles(logic.numCycles);
        addPowerConsumption(logic.power);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0x9 logical not
    void Instruction0x9(int register_d, int register_s, int register_t) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);

        // send clock signal to read registers
        registerFile.OnControlSignal(1);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect register file output latches to adder input ports
        logic.inputPorts[0] = registerFile.outputLatches[0];

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = logic.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal to perform addition
        logic.OnControlSignal(0);
        logic.OnClockSignal();
        addCycles(logic.numCycles);
        addPowerConsumption(logic.power);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0xa shiftr
    void Instruction0xa(int register_d, int register_s, int register_t) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);
        registerFile.inputPorts[1]->setValue(register_t);

        // send clock signal to read registers
        registerFile.OnControlSignal(2);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect register file output latches to adder input ports
        shifter.inputPorts[0] = registerFile.outputLatches[0];
        shifter.inputPorts[1] = registerFile.outputLatches[1];

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = shifter.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal to perform addition
        shifter.OnControlSignal(0);
        shifter.OnClockSignal();
        addCycles(shifter.numCycles);
        addPowerConsumption(shifter.power);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0xb shiftri
    void Instruction0xb(int register_d, int immediate) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_d);

        // send clock signal to read registers
        registerFile.OnControlSignal(1);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect register file output latches to adder input ports
        shifter.inputPorts[0] = registerFile.outputLatches[0];
        shifter.inputPorts[1]->setValue(immediate);

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = shifter.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal to perform addition
        shifter.OnControlSignal(0);
        shifter.OnClockSignal();
        addCycles(shifter.numCycles);
        addPowerConsumption(shifter.power);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0xc shiftl
    void Instruction0xc(int register_d, int register_s, int register_t) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);
        registerFile.inputPorts[1]->setValue(register_t);

        // send clock signal to read registers
        registerFile.OnControlSignal(2);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect register file output latches to adder input ports
        shifter.inputPorts[0] = registerFile.outputLatches[0];
        shifter.inputPorts[1] = registerFile.outputLatches[1];

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = shifter.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal to perform addition
        shifter.OnControlSignal(1);
        shifter.OnClockSignal();
        addCycles(shifter.numCycles);
        addPowerConsumption(shifter.power);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0xd shiftri
    void Instruction0xd(int register_d, int immediate) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_d);

        // send clock signal to read registers
        registerFile.OnControlSignal(1);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect register file output latches to adder input ports
        shifter.inputPorts[0] = registerFile.outputLatches[0];
        shifter.inputPorts[1]->setValue(immediate);

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = shifter.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal to perform addition
        shifter.OnControlSignal(1);
        shifter.OnClockSignal();
        addCycles(shifter.numCycles);
        addPowerConsumption(shifter.power);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0x0e br rd
    void Instruction0x0e(int register_d) {
         // This is a buffer cycle that we have to go through, because we pass by the register the first time
        addCycles(1);

        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_d);

        // This is a buffer cycle that we have to go through, because we pass by the ALU
        addCycles(1);

        // send clock signal to read registers
        registerFile.OnControlSignal(1);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // need to initialise PC
        // connect addr output latch to register file 1st input port
        PC = registerFile.outputLatches[0]->getValue();
    }

    // instruction 0x0f brr rd
    void Instruction0x0f(int register_d) {

        // This is a buffer cycle that we have to go through, because we pass by the register the first time
        addCycles(1);

        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_d);

        // This is a buffer cycle that we have to go through, because we pass by the ALU
        addCycles(1);

        // send clock signal to read registers
        registerFile.OnControlSignal(1);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // need to initialise PC
        // connect addr output latch to register file 1st input port
        PC += registerFile.outputLatches[0]->getValue();
    }

    // instruction 0x10 brr L
    void Instruction0x10(int immediate) {
        // This is a buffer cycle that we have to go through, because we pass by the register the first time
        addCycles(1);

        // This is a buffer cycle that we have to go through, because we pass by the ALU
        addCycles(1);

        // This is a buffer cycle that we have to go through, because we pass by the regiser the second time
        addCycles(1);

        PC += immediate;
    }

    // instruction 0x11 brnz rd, rs
    void Instruction0x11(int register_d, int register_s) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);

        // send clock signal to read registers
        registerFile.OnControlSignal(1);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // This is a buffer cycle that we have to go through, because we pass by the ALU
        addCycles(1);

        if (registerFile.outputLatches[0] == 0) {
            PC += 4;

        } else {

            // set value for register file 2nd input port
            registerFile.inputPorts[1]->setValue(register_d);
        }

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0x14 brgt rd, rs,, rt
    void Instruction0x14(int register_d, int register_s, int register_t) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);
        registerFile.inputPorts[1]->setValue(register_t);

        // send clock signal to read registers
        registerFile.OnControlSignal(2);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // This is a buffer cycle that we have to go through, because we pass by the ALU
        addCycles(1);

        if (registerFile.outputLatches[0] <= registerFile.outputLatches[1]) {
            PC += 4;

        } else {
            // set value for register file 2nd input port
            registerFile.inputPorts[1]->setValue(register_d);
        }

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0x15 mov
    void Instruction0x15(int register_d, int register_s, int immediate) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);

        // send clock signal to read registers
        registerFile.OnControlSignal(1);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // This is a buffer cycle that we have to go through because we pass by the ALU
        addCycles(1);

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = registerFile.outputLatches[0] + immediate;
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0x16 mov
    void Instruction0x16(int register_d, int register_s) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);

        // send clock signal to read registers
        registerFile.OnControlSignal(1);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // This is a buffer cycle that we have to go through because we pass by the ALU
        addCycles(1);

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = registerFile.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0x17 mov rd, L
    void Instruction0x17(int register_d, int register_s, int immediate) {
        // This is a buffer cycle that we have to go through, because we pass by the register the first time
        addCycles(1);

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0]->setValue(immediate);
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d);

        // This is a buffer cycle that we have to go through, because we pass by the ALU
        addCycles(1);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

    }

    // instruction 0x18 mov (rd)(L), rs
    void Instruction0x18(int register_d, int register_s, int immediate) {
        // set the input ports for the register file
        registerFile.inputPorts[0]->setValue(register_s);

        // send clock signal to read registers
        registerFile.OnControlSignal(1);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);

        // connect addr output latch to register file 1st input port
        registerFile.inputPorts[0] = registerFile.outputLatches[0];
        // set value for register file 2nd input port
        registerFile.inputPorts[1]->setValue(register_d + immediate);
        // This is a buffer cycle that we have to go through, because we pass by the ALU
        addCycles(1);

        // send clock signal for write
        registerFile.OnControlSignal(3);
        registerFile.OnClockSignal();
        addCycles(registerFile.numCycles);
        addPowerConsumption(registerFile.power);
    }

    // instruction 0x1f Halt
    void Instruction0x1f() {
        HALT = 1;
        std::exit(0);
    }


private:
    RegisterFile registerFile;
    Addr addr;
    Multiplier multiplier;
    Divider divider;
    Shifter shifter;
    Logic logic;

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
        inputPorts[0] = new Port();
        outputLatches[0] = new Port();
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
                execute.Instruction0x0(result.reg1, result.reg2, result.reg3);
                break;
            case 0b00001:
                // set control signals and pass function for "addi r1 L"
                execute.Instruction0x1(result.reg1, result.literal);
                break;
            case 0b00010:
                // set control signals and pass function for "sub r1 r2 r3"
                execute.Instruction0x2(result.reg1, result.reg2, result.reg3);
                break;
            case 0b00011:
                // set control signals and pass function for "subi r1 L"
                execute.Instruction0x3(result.reg1, result.literal);
                break;
            case 0b00100:
                // set control signals and pass function for "mul r1 r2 r3"
                execute.Instruction0x4(result.reg1, result.reg2, result.reg3);
                break;
            case 0b00101:
                // set control signals and pass function for "div r1 r2 r3"
                execute.Instruction0x5(result.reg1, result.reg2, result.reg3);
                break;
            case 0b00110:
                // set control signals and pass function for "and r1 r2 r3"
                execute.Instruction0x6(result.reg1, result.reg2, result.reg3);
                break;
            case 0b00111:
                // set control signals and pass function for "or r1 r2 r3"
                execute.Instruction0x7(result.reg1, result.reg2, result.reg3);
                break;
            case 0b01000:
                // set control signals and pass function for "xor r1 r2 r3"
                execute.Instruction0x8(result.reg1, result.reg2, result.reg3);
                break;
            case 0b01001:
                // set control signals and pass function for "not r1 r2" r1=~r3
                execute.Instruction0x9(result.reg1, result.reg2, result.reg3);
                break;
            case 0b01010:
                // set control signals and pass function for "shftr r1 r2 r3" r1=r2>>r3
                execute.Instruction0xa(result.reg1, result.reg2, result.reg3);
                break;
            case 0b01011:
                // set control signals and pass function for "shftri r1 L"
                execute.Instruction0xb(result.reg1, result.literal);
                break;
            case 0b01100:
                // set control signals and pass function for "shftl r1 r2 r3"
                execute.Instruction0xc(result.reg1, result.reg2, result.reg3);
                break;
            case 0b01101:
                // set control signals and pass function for "shftli r1 L"
                execute.Instruction0xd(result.reg1, result.literal);
                break;
            case 0b01110:
                // set control signals and pass function for "br r1" PC=r1
                execute.Instruction0x0e(result.reg1);
                break;
            case 0b01111:
                // set control signals and pass function for "brr r1" PC = PC + 1
                execute.Instruction0x0f(result.reg1);
                break;
            case 0b10000:
                // set control signals and pass function for "brr L" PC = PC + L
                execute.Instruction0x10(result.literal);
                break;
            case 0b10001:
                // set control signals and pass function for "brnz r1 r2" PC = (r2==0)? PC+4 : r1
                execute.Instruction0x11(result.reg1, result.reg2);
                break;
            case 0b10100:
                // set control signals and pass function for "brgt"
                execute.Instruction0x14(result.reg1, result.reg2, result.reg3);
                break;
            case 0b11111:
                // set control signals and pass function for "halt"
                execute.Instruction0x1f();
                break;
            case 0b10110:
                // set control signals and pass function for "mov r1 r2" r1=r2
                execute.Instruction0x16(result.reg1, result.reg2);
                break;
            case 0b10111:
                // set control signals and pass function for "mov r1 L" r1[52:63] = L
                execute.Instruction0x17(result.reg1, result.reg2, result.literal);
                break;
            case 0b11000:
                // set control signals and pass function for "mov r1(L) r2" mem[r1+l] = r2
                execute.Instruction0x18(result.reg1, result.reg2, result.reg3);
                break;
            case 0b11001:
                // set control signals and pass function for "addf r1 r2 r3"
                execute.Instruction0x0(result.reg1, result.reg2, result.reg3);
                break;
            case 0b11010:
                // set control signals and pass function for "subf r1 r2 r3"
                execute.Instruction0x2(result.reg1, result.reg2, result.reg3);
                break;
            case 0b11011:
                // set control signals and pass function for "mulf r1 r2 r3"
                execute.Instruction0x4(result.reg1, result.reg2, result.reg3);
                break;
            case 0b11100:
                // set control signals and pass function for "divf r1 r2 r3"
                execute.Instruction0x5(result.reg1, result.reg2, result.reg3);
                break;
        }
    }

    // Function for reacting to the clock signal
    void OnClockSignal() {
        PerformFunction();
    }

public:
    Port* inputPorts[1];
    Port* outputLatches[1];

private:
    InstructionExecute execute;
    double area;
    double power;
    double numCycles;
    long long outputVal;
};


void InstructionFetch() {
    std::string line;
    ControlArray controlArray;

    std::ifstream file1("write_registers.txt");
    if (file1.is_open()) {
        int lineNumber = 0;
        while (getline(file1, line)) {
            PC += 4;
            if (lineNumber % 2 == 0) {
                InstructionExecute::addCycles(1); // fetch cycle
                const char *binaryString = line.c_str();
                long long instruction = strtoll(binaryString, NULL, 2);
                controlArray.inputPorts[0]->setValue(instruction);
                controlArray.OnClockSignal();
                InstructionExecute::addCycles(1); // decode cycle
            }
            lineNumber++;
        }
        file1.close();
    }


    std::ifstream file2("test5.txt");
    int lineNumber = 0;
    if (file2.is_open()) {
        while (getline(file2, line)) {
            PC += 4;
            if (lineNumber % 2 == 0) {
                InstructionExecute::addCycles(1); // fetch cycle
                const char *binaryString = line.c_str();
                long long instruction = strtoll(binaryString, NULL, 2);
                controlArray.inputPorts[0]->setValue(instruction);
                controlArray.OnClockSignal();
                InstructionExecute::addCycles(1); // decode cycle
            }
            lineNumber++;
        }
        file2.close();
    }
}

int main() {
    InstructionFetch();
    return 0;
}