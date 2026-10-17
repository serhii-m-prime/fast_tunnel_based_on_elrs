#include "TXModuleEndpoint.h"
#include "common.h"
#include "device.h"
#include "options.h"

static bool initialize()
{
  return !firmwareOptions.is_airport;
}

static int event()
{
  if (connectionState < FAILURE_STATES)
  {
    crsfTransmitter.updateParameters();
  }
  return DURATION_NEVER;
}

static int start()
{
  if (connectionState < FAILURE_STATES)
  {
    event();
  }
  return DURATION_NEVER;
}

device_t TXLUA_device = {
  .initialize = initialize,
  .start = start,
  .event = event,
  .timeout = nullptr,
  .subscribe = EVENT_ALL
};
