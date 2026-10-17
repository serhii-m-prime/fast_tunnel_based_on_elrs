#include "RXEndpoint.h"
#include "device.h"
#include "options.h"

extern RXEndpoint crsfReceiver;

static bool initialize()
{
    return !firmwareOptions.is_airport;
}

static int event()
{
    crsfReceiver.updateParameters();
    return DURATION_NEVER;
}

static int start()
{
    crsfReceiver.registerParameters();
    event();
    return DURATION_NEVER;
}

device_t RXLUA_device = {
  .initialize = initialize,
  .start = start,
  .event = event,
  .timeout = nullptr,
  .subscribe = EVENT_ALL
};
