#ifndef ABSTRACT_DEVICE_H
#define ABSTRACT_DEVICE_H

class Device {
public:
    // Constructors
    // Device() {}

    // Pure Virtual Function for processing the data from input ports
    virtual void ProcessDataInput() = 0;

    // Pure Virtual Function for performing the device's main function
    virtual void PerformFunction() = 0;

    // Pure Virtual Function for reacting to the clock signal
    virtual void OnClockSignal() = 0;

    // Pure Virtual Function for reacting to control signals
    virtual void OnControlSignal() = 0;

    // Pure Virtual Function for updating the output latches
    virtual void UpdateOutputLatches(int** latches) = 0;

    // Function for connecting input ports
    virtual void ConnectInputPorts(int** ports) {
        // Implementation for connecting input
    }
};


#endif // ABSTRACT_DEVICE_H
