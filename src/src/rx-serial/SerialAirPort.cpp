#if defined(TARGET_RX)

#include "SerialAirPort.h"
#include "device.h"
#include "common.h"
#include "eml_bridge.h"

#if defined(EML_BRIDGE)
SerialAirPort::SerialAirPort(Stream &out, Stream &in) : SerialIO(&out, &in), _airportInput(in) {}

uint32_t SerialAirPort::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData)
{
    return DURATION_NEVER;
}

int SerialAirPort::getMaxSerialReadSize()
{
    return 64;
}

void SerialAirPort::processBytes(uint8_t *bytes, u_int16_t size) {}

void SerialAirPort::sendQueuedData(uint32_t maxBytesToSend)
{
    emlBridgeDrainSerial(*_outputPort, maxBytesToSend);
}

void SerialAirPort::processSerialInput()
{
    emlBridgeReadSerial(_airportInput);
}
#else
// Keep the UART queues in static DRAM even though the SerialIO wrapper is
// constructed during setup. There is only one UART0 AirPort instance.
static FIFO<AP_MAX_BUF_LEN> airportInput;
static FIFO<AP_MAX_BUF_LEN> airportOutput;

SerialAirPort::SerialAirPort(Stream &out, Stream &in) : SerialIO(&out, &in),
    apInputBuffer(airportInput), apOutputBuffer(airportOutput), _airportInput(in) {}

uint32_t SerialAirPort::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData)
{
    return DURATION_NEVER;
}

int SerialAirPort::getMaxSerialReadSize()
{
    return std::min((uint16_t)64, apInputBuffer.free());
}

void SerialAirPort::processBytes(uint8_t *bytes, u_int16_t size)
{
    if (connectionState == connected)
    {
        if (!apInputBuffer.atomicTryPushBytes(bytes, size)) uartStats.refused += size;
    }
}

void SerialAirPort::sendQueuedData(uint32_t maxBytesToSend)
{
    emlDrainSerial(*_outputPort, apOutputBuffer, uartStats, maxBytesToSend);
}

void SerialAirPort::processSerialInput()
{
    emlReadSerial(_airportInput, apInputBuffer, connectionState == connected, uartStats);
}
#endif // EML_BRIDGE
#endif
