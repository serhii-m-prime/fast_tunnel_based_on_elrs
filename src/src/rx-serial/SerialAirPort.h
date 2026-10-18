#pragma once

#include "OTA.h"
#include "SerialIO.h"
#include "FIFO.h"

class SerialAirPort final : public SerialIO {
public:
    explicit SerialAirPort(Stream &out, Stream &in);
    ~SerialAirPort() override = default;

    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

    int getMaxSerialReadSize() override;
    void sendQueuedData(uint32_t maxBytesToSend) override;
    void processSerialInput() override;

#if !defined(EML_BRIDGE)
    // EML_BRIDGE: UART bytes go straight to the EML transport rings (eml_bridge.h)
    FIFO<AP_MAX_BUF_LEN> &apInputBuffer;
    FIFO<AP_MAX_BUF_LEN> &apOutputBuffer;
    EmlUartStats_s uartStats;
    volatile uint32_t radioRejectedFrames = 0;

    bool isTlmQueued() const { return apInputBuffer.size() > 0; }
#endif
private:
    Stream &_airportInput;
    void processBytes(uint8_t *bytes, u_int16_t size) override;
};
