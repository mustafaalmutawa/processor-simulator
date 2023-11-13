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
};


#endif // ABSTRACT_DEVICE_H
