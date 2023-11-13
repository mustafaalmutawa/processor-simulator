#include <iostream>

#include "AbstractDevice.h"

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

    // Function for processing the data from input ports
    void ProcessDataInput() {
        inputVal1 = *port1;
        inputVal2 = *port2;
    }

    // Function for performing the device's main function
    void PerformFunction() {
        outputVal = inputVal1 + inputVal2;
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
    void UpdateOutputLatches(int** latches) {
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
        return;
    }

    // Function for updating the output latches
    void UpdateOutputLatches(int** latches) {
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
 * Shifter Device (under construction)
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
        outputVal = inputVal1 * inputVal2;
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
    void UpdateOutputLatches(int** latches) {
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


int main() {
    std::cout << "Hello, World!" << std::endl;

    int p1 = 3;
    int p2 = 4;
    int* ports[] = {&p1, &p2};
    int latch;
    int* latches[] = {&latch};
    Addr addr(400, 0.5, 1);
    addr.ConnectInputPorts(ports);
    addr.UpdateOutputLatches(latches);
    addr.ProcessDataInput();
    addr.PerformFunction();
    addr.OnClockSignal();
    std::cout << "result: " << **latches << std::endl;

    return 0;
}