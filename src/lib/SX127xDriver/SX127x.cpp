#include "SX127x.h"
#include "logging.h"
#include "RFAMP_hal.h"
#include "SX127xPll.h"

SX127xHal hal;
RFAMP_hal RFAMP;
SX127xDriver *SX127xDriver::instance = NULL;

#if defined(EML_BRIDGE) && defined(SX127X_FSK_SUPPORT)
bool SX127xDriver::ProbePll(uint32_t freq, uint32_t timeoutUs)
{
  if (!timeoutUs || timeoutUs > 20000U) return false;
  RFAMP.TXRXdisable();
  SetMode(SX127x_OPMODE_SLEEP, SX12XX_Radio_All);
  ModFSKorLoRa = SX127x_OPMODE_FSK_OOK;
  hal.writeRegister(SX127X_REG_OP_MODE, ModFSKorLoRa | lowFrequencyMode, SX12XX_Radio_All);
  currOpmode = SX127x_OPMODE_SLEEP;
  InvalidateLoraCache();
  SetFrequencyReg(freq, SX12XX_Radio_All);
  auto flags = []() { return hal.readRegister(SX127X_FSK_REG_IRQ_FLAGS1, SX12XX_Radio_1); };
  auto clock = []() { return (uint32_t)micros(); };
  auto pause = []() { delayMicroseconds(20); };
  // Require unlock in Standby: a stale asserted lock must never pass a new probe.
  bool ok = sx127xWaitPll(flags, clock, pause, timeoutUs, false);
  uint8_t frf[3];
  hal.readRegister(SX127X_REG_FRF_MSB, frf, 3, SX12XX_Radio_1);
  ok = ok && (((uint32_t)frf[0] << 16) | ((uint32_t)frf[1] << 8) | frf[2]) == freq;
  if (ok)
  {
    SetMode(SX127x_OPMODE_FSRX, SX12XX_Radio_All);
    ok = sx127xWaitPll(flags, clock, pause, timeoutUs, true);
  }
  SetMode(SX127x_OPMODE_STANDBY, SX12XX_Radio_All);
  return ok;
}

bool SX127xDriver::ProbeLoraPll(uint32_t freq)
{
  if (ModFSKorLoRa != SX127x_OPMODE_LORA) return false;
  RFAMP.TXRXdisable();
  SetFrequencyReg(freq, SX12XX_Radio_All);
  SetMode(SX127x_OPMODE_RXCONTINUOUS, SX12XX_Radio_All);
  delayMicroseconds(1000); // exceeds specified synthesizer startup; no SPI in ISR here
  const uint8_t status = hal.readRegister(0x1C, SX12XX_Radio_1); // RegHopChannel.PllTimeout
  SetMode(SX127x_OPMODE_STANDBY, SX12XX_Radio_All);
  return !(status & 0x80U); // negative indication only; positive FSK probe is also required
}
#endif

const uint8_t SX127x_AllowedSyncwords[105] =
    {0, 5, 6, 7, 11, 12, 13, 15, 18,
     21, 23, 26, 29, 30, 31, 33, 34,
     37, 38, 39, 40, 42, 44, 50, 51,
     54, 55, 57, 58, 59, 61, 63, 65,
     67, 68, 71, 77, 78, 79, 80, 82,
     84, 86, 89, 92, 94, 96, 97, 99,
     101, 102, 105, 106, 109, 111, 113, 115,
     117, 118, 119, 121, 122, 124, 126, 127,
     129, 130, 138, 143, 161, 170, 172, 173,
     175, 180, 181, 182, 187, 190, 191, 192,
     193, 196, 199, 201, 204, 205, 208, 209,
     212, 213, 219, 220, 221, 223, 227, 229,
     235, 239, 240, 242, 243, 246, 247, 255};

//////////////////////////////////////////////

SX127xDriver::SX127xDriver(): SX12xxDriverCommon()
{
  instance = this;
  // default values from datasheet
  currSyncWord = SX127X_SYNC_WORD;
  currBW =SX127x_BW_125_00_KHZ;
  currSF = SX127x_SF_7;
  currCR = SX127x_CR_4_5;
  currOpmode = SX127x_OPMODE_SLEEP;
  ModFSKorLoRa = SX127x_OPMODE_LORA;
  // Dummy default values which are overwritten during setup
  currPreambleLen = 0;
  PayloadLength = 8;
  currFreq = 0;
  headerExplMode = false;
  crcEnabled = false;
  lowFrequencyMode = SX1278_HIGH_FREQ;
  strongestReceivingRadio = SX12XX_Radio_1;
#if defined(SX127X_FSK_SUPPORT)
  fskRxConfig = 0;
  fskLastRssi = 0;
#endif
}

bool SX127xDriver::Begin()
{
  hal.init();
  hal.IsrCallback_1 = &SX127xDriver::IsrCallback_1;
  hal.IsrCallback_2 = &SX127xDriver::IsrCallback_2;

  hal.reset();
  DBGLN("SX127x Begin");

  RFAMP.init();

  // currFreq must be set before calling Radio.Begin so that lowFrequencyMode can be set correctly.
  if (currFreq < (uint32_t)((double)525000000 / (double)FREQ_STEP))
  {
    lowFrequencyMode = SX1278_LOW_FREQ;
    DBGLN("Setting 'lowFrequencyMode' used for 433MHz.");
  }

  SetMode(SX127x_OPMODE_STANDBY, SX12XX_Radio_All);

  if (!DetectChip(SX12XX_Radio_1))
  {
    End();
    return false;
  }

  if (GPIO_PIN_NSS_2 != UNDEF_PIN)
  {
    if (!DetectChip(SX12XX_Radio_2))
    {
      End();
      return false;
    }
  }

  ConfigLoraDefaults();
  // Force the next power update, and use the defaults for RFO_HF or PA_BOOST
  pwrCurrent = PWRPENDING_NONE;
  if (POWER_OUTPUT_VALUES2 == nullptr)
  {
    if (OPT_USE_SX1276_RFO_HF)
    {
      SetOutputPower(SX127X_MAX_OUTPUT_POWER_RFO_HF);
    }
    else
    {
      SetOutputPower(SX127X_MAX_OUTPUT_POWER);
    }
  }
  CommitOutputPower();

  return true;
}

void SX127xDriver::End()
{
  SetMode(SX127x_OPMODE_SLEEP, SX12XX_Radio_All);
  hal.end();
  RFAMP.TXRXdisable();
  RemoveCallbacks();
}

void SX127xDriver::cwRepeat(SX12XX_Radio_Number_t radioNumber)
{
  uint8_t dummy[0] = {};

  ClearIrqFlags(radioNumber);

  hal.writeRegister(SX127X_REG_FIFO_ADDR_PTR, SX127X_FIFO_TX_BASE_ADDR_MAX, radioNumber);
  hal.writeRegister(SX127X_REG_FIFO, dummy, 0, radioNumber);

  SetMode(SX127x_OPMODE_TX, radioNumber);
}

void SX127xDriver::startCWTest(uint32_t freq, SX12XX_Radio_Number_t radioNumber)
{
  SetPreambleLength(8);
  SetSpreadingFactor(SX127x_SF_9);
  SetBandwidthCodingRate(SX127x_BW_125_00_KHZ, SX127x_CR_4_5);
  const int32_t regFreq = ((uint32_t)((double)freq / (double)FREQ_STEP));
  SetFrequencyReg(regFreq, SX12XX_Radio_All);
  if (freq > 900000000)
  {
    hal.writeRegister(0x01, 0x80, radioNumber);
    hal.writeRegister(0x44, 0x7B, radioNumber);
    hal.writeRegister(0x3D, 0xA1, radioNumber);
    hal.writeRegister(0x36, 0x01, radioNumber);
    hal.writeRegister(0x1e, 0x08, radioNumber);
    hal.writeRegister(0x45, 0xDF, radioNumber);
    hal.writeRegister(0x46, 0x03, radioNumber);
    hal.writeRegister(0x4D, 0x87, radioNumber);
    hal.writeRegister(0x52, 0x60, radioNumber);
  }
  else
  {
    hal.writeRegister(0x01, 0x88, radioNumber);
    hal.writeRegister(0x3D, 0xA1, radioNumber);
    hal.writeRegister(0x36, 0x01, radioNumber);
    hal.writeRegister(0x1e, 0x08, radioNumber);
  }

  RFAMP.TXenable(radioNumber);
  hal.writeRegister(SX127X_REG_PA_CONFIG, pwrCurrent, radioNumber);

  cwRepeat(radioNumber);
}

void SX127xDriver::ConfigLoraDefaults()
{
#if defined(SX127X_FSK_SUPPORT)
  const bool fromFsk = ModFSKorLoRa != SX127x_OPMODE_LORA;
  if (fromFsk)
  {
    // Coming back from FSK: the LoRa page must be rewritten in full
    ModFSKorLoRa = SX127x_OPMODE_LORA;
    InvalidateLoraCache();
  }
#endif
  hal.writeRegister(SX127X_REG_OP_MODE, SX127x_OPMODE_SLEEP, SX12XX_Radio_All);
  hal.writeRegister(SX127X_REG_OP_MODE, ModFSKorLoRa, SX12XX_Radio_All); //must be written in sleep mode
  SetMode(SX127x_OPMODE_STANDBY, SX12XX_Radio_All);
#if defined(SX127X_FSK_SUPPORT)
  if (fromFsk)
  {
    // The FSK setup wrote RegSyncConfig/RegNodeAdrs/RegImageCal at the addresses LoRa uses
    // for PpmCorrection (0x27), InvertIQ (0x33) and InvertIQ2 (0x3B): restore the defaults
    hal.writeRegister(SX127x_PPMOFFSET, 0x00, SX12XX_Radio_All);
    hal.writeRegister(0x33, 0x27, SX12XX_Radio_All);
    hal.writeRegister(0x3B, 0x1D, SX12XX_Radio_All);
  }
#endif

  hal.writeRegister(SX127X_REG_PAYLOAD_LENGTH, PayloadLength, SX12XX_Radio_All);
  SetSyncWord(currSyncWord);
  hal.writeRegister(SX127X_REG_FIFO_TX_BASE_ADDR, SX127X_FIFO_TX_BASE_ADDR_MAX, SX12XX_Radio_All);
  hal.writeRegister(SX127X_REG_FIFO_RX_BASE_ADDR, SX127X_FIFO_RX_BASE_ADDR_MAX, SX12XX_Radio_All);
  hal.writeRegisterBits(SX127X_REG_DIO_MAPPING_1, SX127X_DIO0_RXTX_DONE, SX127X_DIO0_MASK, SX12XX_Radio_All); //undocumented "hack", looking at Table 18 from datasheet SX127X_REG_DIO_MAPPING_1 = 11 appears to be unspported by infact it generates an intterupt on both RXdone and TXdone, this saves switching modes.
  hal.writeRegister(SX127X_REG_LNA, SX127X_LNA_BOOST_ON, SX12XX_Radio_All);
  hal.writeRegister(SX1278_REG_MODEM_CONFIG_3, SX1278_AGC_AUTO_ON | SX1278_LOW_DATA_RATE_OPT_OFF, SX12XX_Radio_All);
  hal.writeRegisterBits(SX127X_REG_OCP, SX127X_OCP_ON | SX127X_OCP_150MA, SX127X_OCP_MASK, SX12XX_Radio_All); //150ma max current
  SetPreambleLength(SX127X_PREAMBLE_LENGTH_LSB);
}

void SX127xDriver::SetBandwidthCodingRate(SX127x_Bandwidth bw, SX127x_CodingRate cr)
{
  if ((currBW != bw) || (currCR != cr))
  {
    if (currSF == SX127x_SF_6) // set SF6 optimizations
    {
      hal.writeRegister(SX127X_REG_MODEM_CONFIG_1, bw | cr | SX127x_HEADER_IMPL_MODE, SX12XX_Radio_All);
      SetCRCMode(false);
    }
    else
    {
      if (headerExplMode)
      {
        hal.writeRegister(SX127X_REG_MODEM_CONFIG_1, bw | cr | SX127x_HEADER_EXPL_MODE, SX12XX_Radio_All);
      }
      else
      {
        hal.writeRegister(SX127X_REG_MODEM_CONFIG_1, bw | cr | SX127x_HEADER_IMPL_MODE, SX12XX_Radio_All);
      }
      SetCRCMode(crcEnabled);
    }

    #if !defined(RADIO_SX1272) //does not apply to SX1272
      if (bw == SX127x_BW_500_00_KHZ)
      {
        // Errata 2.1 Sensitivity Optimization with a 500 kHz Bandwidth https://www.semtech.com/products/wireless-rf/lora-connect/sx1276
        hal.writeRegister(0x36, 0x02, SX12XX_Radio_All);
        hal.writeRegister(0x3a, lowFrequencyMode ? 0x7F : 0x64, SX12XX_Radio_All);


        // Errata 2.3 Receiver Spurious Reception of a LoRa Signal - https://www.semtech.com/products/wireless-rf/lora-connect/sx1276
        hal.writeRegisterBits(SX127X_REG_DETECT_OPTIMIZE, 0x80, 0x80, SX12XX_Radio_All);
      }
      else
      {
        // Errata 2.1 Sensitivity Optimization with a 500 kHz Bandwidth https://www.semtech.com/products/wireless-rf/lora-connect/sx1276
        hal.writeRegister(0x36, 0x03, SX12XX_Radio_All);

        // Errata 2.3 Receiver Spurious Reception of a LoRa Signal - https://www.semtech.com/products/wireless-rf/lora-connect/sx1276
        // !!! Note - Registers 0x2F and 0x30 also need to be corrected if ELRS every uses other BW.  Check errata. !!!
        hal.writeRegisterBits(SX127X_REG_DETECT_OPTIMIZE, 0x00, 0x80, SX12XX_Radio_All);
      }
    #endif

    currCR = cr;
    currBW = bw;
  }
}

void SX127xDriver::SetCRCMode(bool on)
{
  if(on)
  {
    #if defined(RADIO_SX1272)
      hal.writeRegisterBits(SX127X_REG_MODEM_CONFIG_1, SX1272_RX_CRC_MODE_ON, SX1272_RX_CRC_MODE_MASK, SX12XX_Radio_All);
    #else
      hal.writeRegisterBits(SX127X_REG_MODEM_CONFIG_2, SX1278_RX_CRC_MODE_ON, SX1278_RX_CRC_MODE_MASK, SX12XX_Radio_All);
    #endif
  }
  else
  {
    #if defined(RADIO_SX1272)
      hal.writeRegisterBits(SX127X_REG_MODEM_CONFIG_1, SX1272_RX_CRC_MODE_OFF, SX1272_RX_CRC_MODE_MASK, SX12XX_Radio_All);
    #else
      hal.writeRegisterBits(SX127X_REG_MODEM_CONFIG_2, SX1278_RX_CRC_MODE_OFF, SX1278_RX_CRC_MODE_MASK, SX12XX_Radio_All);
    #endif
  }
}

bool SyncWordOk(uint8_t syncWord)
{
  for (unsigned int i = 0; i < sizeof(SX127x_AllowedSyncwords); i++)
  {
    if (syncWord == SX127x_AllowedSyncwords[i])
    {
      return true;
    }
  }
  return false;
}

void SX127xDriver::SetSyncWord(uint8_t syncWord)
{
  uint8_t _syncWord = syncWord;

  while (SyncWordOk(_syncWord) == false)
  {
    _syncWord++;
  }

  if(syncWord != _syncWord){
    DBGLN("Using syncword: %d instead of: %d", _syncWord, syncWord);
  }

  hal.writeRegister(SX127X_REG_SYNC_WORD, _syncWord, SX12XX_Radio_All);
  currSyncWord = _syncWord;
}

/***
 * @brief: Schedule an output power change after the next transmit
 * The radio must be in SX127x_OPMODE_STANDBY to change the power
 ***/
void SX127xDriver::SetOutputPower(uint8_t Power)
{
  uint8_t pwrNew;
  Power &= SX127X_PA_POWER_MASK;

  if (OPT_USE_SX1276_RFO_HF)
  {
    pwrNew = SX127X_PA_SELECT_RFO | Power;
  }
  else
  {
    pwrNew = SX127X_PA_SELECT_BOOST | Power;
  }

  if ((pwrPending == PWRPENDING_NONE && pwrCurrent != pwrNew) || pwrPending != pwrNew)
  {
    pwrPending = pwrNew;
  }
}

void ICACHE_RAM_ATTR SX127xDriver::CommitOutputPower()
{
  if (pwrPending == PWRPENDING_NONE)
    return;

  pwrCurrent = pwrPending & 0xFF;
  pwrPending = PWRPENDING_NONE;
  hal.writeRegister(SX127X_REG_PA_CONFIG, pwrCurrent, SX12XX_Radio_All);
}

void SX127xDriver::SetPreambleLength(uint8_t PreambleLen)
{
  if (currPreambleLen != PreambleLen)
  {
    hal.writeRegister(SX127X_REG_PREAMBLE_LSB, PreambleLen, SX12XX_Radio_All);
    currPreambleLen = PreambleLen;
  }
}

void SX127xDriver::SetSpreadingFactor(SX127x_SpreadingFactor sf)
{
  if (currSF != sf)
  {
    hal.writeRegisterBits(SX127X_REG_MODEM_CONFIG_2, sf | SX127X_TX_MODE_SINGLE,  SX127X_SPREADING_FACTOR_MASK | SX127X_TX_MODE_MASK, SX12XX_Radio_All);
    if (sf == SX127x_SF_6)
    {
      hal.writeRegisterBits(SX127X_REG_DETECT_OPTIMIZE, SX127X_DETECT_OPTIMIZE_SF_6, SX127X_DETECT_OPTIMIZE_SF_MASK, SX12XX_Radio_All);
      hal.writeRegister(SX127X_REG_DETECTION_THRESHOLD, SX127X_DETECTION_THRESHOLD_SF_6, SX12XX_Radio_All);
    }
    else
    {
      hal.writeRegisterBits(SX127X_REG_DETECT_OPTIMIZE, SX127X_DETECT_OPTIMIZE_SF_7_12, SX127X_DETECT_OPTIMIZE_SF_MASK, SX12XX_Radio_All);
      hal.writeRegister(SX127X_REG_DETECTION_THRESHOLD, SX127X_DETECTION_THRESHOLD_SF_7_12, SX12XX_Radio_All);
    }
    currSF = sf;
  }
}

void ICACHE_RAM_ATTR SX127xDriver::SetFrequencyReg(uint32_t regfreq, SX12XX_Radio_Number_t radioNumber, bool doRx)
{
  currFreq = regfreq;
  SetMode(SX127x_OPMODE_STANDBY, radioNumber);

  uint8_t FRQ_MSB = (uint8_t)((regfreq >> 16) & 0xFF);
  uint8_t FRQ_MID = (uint8_t)((regfreq >> 8) & 0xFF);
  uint8_t FRQ_LSB = (uint8_t)(regfreq & 0xFF);

  WORD_ALIGNED_ATTR uint8_t outbuff[3] = {FRQ_MSB, FRQ_MID, FRQ_LSB}; //check speedup

  hal.writeRegister(SX127X_REG_FRF_MSB, outbuff, sizeof(outbuff), radioNumber);

  if (doRx)
  {
    RXnb();
  }
}

bool SX127xDriver::DetectChip(SX12XX_Radio_Number_t radioNumber)
{
  DBG("Detecting radio %x... ", radioNumber);
  uint8_t i = 0;
  bool flagFound = false;
  while ((i < 3) && !flagFound)
  {
    uint8_t version = hal.readRegister(SX127X_REG_VERSION, radioNumber);
    DBG("%x", version);
    if (version == SX127X_VERSION)
    {
      flagFound = true;
    }
    else
    {
      DBGLN(" not found! (%d of 3 tries) REG_VERSION == 0x%x", i+1, version);
      delay(200);
      i++;
    }
  }

  if (!flagFound)
  {
    DBGLN(" not found!");
    return false;
  }
  else
  {
    DBGLN(" found! (match by REG_VERSION == 0x%x", SX127X_VERSION);
  }
  return true;
}

/////////////////////////////////////TX functions/////////////////////////////////////////

void ICACHE_RAM_ATTR SX127xDriver::TXnbISR()
{
  currOpmode = SX127x_OPMODE_STANDBY; //goes into standby after transmission
  //TXdoneMicros = micros();
  // The power level must be changed when in SX127x_OPMODE_STANDBY, so this lags power
  // changes by at most 1 packet, but does not interrupt any pending RX/TX
  CommitOutputPower();
  TXdoneCallback();
}

void ICACHE_RAM_ATTR SX127xDriver::TXnb(uint8_t * data, bool sendGeminiBuffer, uint8_t * dataGemini, SX12XX_Radio_Number_t radioNumber)
{
  // if (currOpmode == SX127x_OPMODE_TX)
  // {
  //   DBGLN("abort TX");
  //   return; // we were already TXing so abort. this should never happen!!!
  // }

  transmittingRadio = radioNumber;

  SetMode(SX127x_OPMODE_STANDBY, SX12XX_Radio_All);

  if (radioNumber == SX12XX_Radio_NONE)
  {
      return;
  }

#if defined(SX127X_FSK_SUPPORT)
  if (ModFSKorLoRa == SX127x_OPMODE_FSK_OOK)
  {
    // FSK has no FIFO pointers (0x0D is RegRxConfig): clear the FIFO, fill it in Standby and
    // start TX; TxStartCondition=FifoNotEmpty begins after the PA ramp. Gemini is not used.
    hal.writeRegister(SX127X_FSK_REG_IRQ_FLAGS2, SX127X_FSK_IRQ2_FIFO_OVERRUN, radioNumber);
    RFAMP.TXenable(radioNumber);
    hal.writeRegister(SX127X_REG_FIFO, data, PayloadLength, radioNumber);
    SetMode(SX127x_OPMODE_TX, radioNumber);
    return;
  }
#endif

#if defined(DEBUG_RCVR_SIGNAL_STATS)
    if (radioNumber == SX12XX_Radio_All || radioNumber == SX12XX_Radio_1)
    {
        instance->rxSignalStats[0].telem_count++;
    }
    if (radioNumber == SX12XX_Radio_All || radioNumber == SX12XX_Radio_2)
    {
        instance->rxSignalStats[1].telem_count++;
    }
#endif

  RFAMP.TXenable(radioNumber);
  hal.writeRegister(SX127X_REG_FIFO_ADDR_PTR, SX127X_FIFO_TX_BASE_ADDR_MAX, radioNumber);
  if (sendGeminiBuffer)
  {
    hal.writeRegister(SX127X_REG_FIFO, data, PayloadLength, SX12XX_Radio_1);
    hal.writeRegister(SX127X_REG_FIFO, dataGemini, PayloadLength, SX12XX_Radio_2);
  }
  else
  {
    hal.writeRegister(SX127X_REG_FIFO, data, PayloadLength, radioNumber);
  }

  SetMode(SX127x_OPMODE_TX, radioNumber);
}

///////////////////////////////////RX Functions Non-Blocking///////////////////////////////////////////

bool ICACHE_RAM_ATTR SX127xDriver::RXnbISR(SX12XX_Radio_Number_t radioNumber)
{
  uint8_t const FIFOaddr = hal.readRegister(SX127X_REG_FIFO_RX_CURRENT_ADDR, radioNumber);
  hal.writeRegister(SX127X_REG_FIFO_ADDR_PTR, FIFOaddr, radioNumber);
  hal.readRegister(SX127X_REG_FIFO, RXdataBuffer, PayloadLength, radioNumber);
  return RXdoneCallback(SX12XX_RX_OK);
}

void ICACHE_RAM_ATTR SX127xDriver::RXnb()
{
  RFAMP.RXenable();
  SetMode(SX127x_OPMODE_RXCONTINUOUS, SX12XX_Radio_All);
}

void ICACHE_RAM_ATTR SX127xDriver::CheckForSecondPacket()
{
  SX12XX_Radio_Number_t radio[2] = {SX12XX_Radio_1, SX12XX_Radio_2};
  uint8_t processingRadioIdx = (instance->processingPacketRadio == SX12XX_Radio_1) ? 0 : 1;
  uint8_t secondRadioIdx = !processingRadioIdx;

  // processingRadio always passed the sanity check here
  gotRadio[processingRadioIdx] = true;
  gotRadio[secondRadioIdx] = false;

  hasSecondRadioGotData = false;

  if (GPIO_PIN_NSS_2 != UNDEF_PIN)
  {
    uint16_t secondIrqStatus = instance->GetIrqFlags(radio[secondRadioIdx]);
    if(secondIrqStatus & SX127X_CLEAR_IRQ_FLAG_RX_DONE)
    {
      uint8_t const FIFOaddr = hal.readRegister(SX127X_REG_FIFO_RX_CURRENT_ADDR, radio[secondRadioIdx]);
      hal.writeRegister(SX127X_REG_FIFO_ADDR_PTR, FIFOaddr, radio[secondRadioIdx]);
      hal.readRegister(SX127X_REG_FIFO, RXdataBufferSecond, PayloadLength, radio[secondRadioIdx]);

      hasSecondRadioGotData = true;
    }
  }
}

void ICACHE_RAM_ATTR SX127xDriver::GetLastPacketStats()
{
  SX12XX_Radio_Number_t radio[2] = {SX12XX_Radio_1, SX12XX_Radio_2};
  uint8_t processingRadioIdx = (instance->processingPacketRadio == SX12XX_Radio_1) ? 0 : 1;
  uint8_t secondRadioIdx = !processingRadioIdx;

  int8_t rssi[2];
  int8_t snr[2];

  gotRadio[secondRadioIdx] = hasSecondRadioGotData;
  #if defined(DEBUG_RCVR_SIGNAL_STATS)
  if(!hasSecondRadioGotData)
  {
    instance->rxSignalStats[secondRadioIdx].fail_count++;
  }
  #endif

  for (uint8_t i = 0; i < 2; i++)
  {
    if (gotRadio[i])
    {
      rssi[i] = GetLastPacketRSSI(radio[i]);
      snr[i] = GetLastPacketSNRRaw(radio[i]);
      // https://www.mouser.com/datasheet/2/761/sx1276-1278113.pdf
      // Section 3.5.5 (page 87)
      int8_t negOffset = (snr[i] < 0) ? (snr[i] / RADIO_SNR_SCALE) : 0;
      rssi[i] += negOffset;

      // If radio # is 0, update LastPacketRSSI, otherwise LastPacketRSSI2
      (i == 0) ? LastPacketRSSI = rssi[i] : LastPacketRSSI2 = rssi[i];
      // Update whatever SNRs we have
      LastPacketSNRRaw = snr[i];
    }
  }

  // by default, set the strongest receiving radio to be the current processing radio (which got a successful packet)
  instance->strongestReceivingRadio = instance->processingPacketRadio;

  // when both radio got the packet, use the better RSSI one
  if (gotRadio[0] && gotRadio[1])
  {
    LastPacketSNRRaw = instance->fuzzy_snr(snr[0], snr[1], instance->FuzzySNRThreshold);
    // Update the strongest receiving radio to be the one with better signal strength
    instance->strongestReceivingRadio = (rssi[0] > rssi[1]) ? radio[0] : radio[1];
  }

#if defined(DEBUG_RCVR_SIGNAL_STATS)
  // stat updates
  for (uint8_t i = 0; i < 2; i++)
  {
    if (gotRadio[i])
    {
      instance->rxSignalStats[i].irq_count++;
      instance->rxSignalStats[i].rssi_sum += rssi[i];
      instance->rxSignalStats[i].snr_sum += snr[i];
      if (snr[i] > instance->rxSignalStats[i].snr_max)
      {
        instance->rxSignalStats[i].snr_max = snr[i];
      }
      LastPacketSNRRaw = snr[i];
    }
  }
  if (gotRadio[0] || gotRadio[1])
  {
    instance->irq_count_or++;
  }
  if (gotRadio[0] && gotRadio[1])
  {
    instance->irq_count_both++;
  }
#endif
}

void ICACHE_RAM_ATTR SX127xDriver::SetMode(SX127x_RadioOPmodes mode, SX12XX_Radio_Number_t radioNumber)
{
  /*
  Comment out since it is difficult to keep track of dual radios.
  When checking SPI it is also useful to see every possible SPI transaction to make sure it fits when required.
  */
  // if (mode == currOpmode)
  // {
  //    return;
  // }

  hal.writeRegister(SX127X_REG_OP_MODE, mode | lowFrequencyMode, radioNumber);
  currOpmode = mode;
}

void SX127xDriver::Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq, uint8_t preambleLen, bool InvertIQ, uint8_t _PayloadLength)
{
  Config(bw, sf, cr, freq, preambleLen, currSyncWord, InvertIQ, _PayloadLength);
}

void SX127xDriver::Config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t freq, uint8_t preambleLen, uint8_t syncWord, bool InvertIQ, uint8_t _PayloadLength)
{
  // TXnb/RXnbISR move exactly PayloadLength bytes between the FIFO and RXdataBuffer or the
  // caller's OTA_Packet_s, so a length outside the buffers is refused and the last valid one kept
  if (_PayloadLength == 0 || _PayloadLength > RXBuffSize)
  {
    ERRLN("SX127x payload length %u exceeds %u", _PayloadLength, RXBuffSize);
  }
  else
  {
    PayloadLength = _PayloadLength;
  }
  ConfigLoraDefaults();
  SetPreambleLength(preambleLen);
  SetSpreadingFactor((SX127x_SpreadingFactor)sf);
  SetBandwidthCodingRate((SX127x_Bandwidth)bw, (SX127x_CodingRate)cr);
  SetFrequencyReg(freq, SX12XX_Radio_All);
}

uint32_t ICACHE_RAM_ATTR SX127xDriver::GetCurrBandwidth()
{
  switch (currBW)
  {
  case SX127x_BW_125_00_KHZ:
    return 125E3;
  case SX127x_BW_250_00_KHZ:
    return 250E3;
  case SX127x_BW_500_00_KHZ:
    return 500E3;
  }
  return -1;
}

uint32_t ICACHE_RAM_ATTR SX127xDriver::GetCurrBandwidthNormalisedShifted() // this is basically just used for speedier calc of the freq offset, pre compiled for 32mhz xtal
{

  switch (currBW)
  {
  case SX127x_BW_125_00_KHZ:
    return 64;
  case SX127x_BW_250_00_KHZ:
    return 32;
  case SX127x_BW_500_00_KHZ:
    return 16;
  }
  return -1;
}

/**
 * Set the PPMcorrection register to adjust data rate to frequency error
 * @param offset is in Hz or FREQ_STEP (FREQ_HZ_TO_REG_VAL) units, whichever
 *    was used to SetFrequencyHz/SetFrequencyReg
 */
void ICACHE_RAM_ATTR SX127xDriver::SetPPMoffsetReg(int32_t offset, SX12XX_Radio_Number_t radioNumber)
{
#if defined(SX127X_FSK_SUPPORT)
  if (ModFSKorLoRa == SX127x_OPMODE_FSK_OOK)
    return; // 0x27 is RegSyncConfig in FSK
#endif
  int8_t offsetPPM = (offset * 1000000 / currFreq) * 95 / 100;
  hal.writeRegister(SX127x_PPMOFFSET, (uint8_t)offsetPPM, radioNumber);
}

bool ICACHE_RAM_ATTR SX127xDriver::GetFrequencyErrorbool(SX12XX_Radio_Number_t radioNumber)
{
  return (hal.readRegister(SX127X_REG_FEI_MSB, radioNumber) & 0b1000) >> 3; // returns true if pos freq error, neg if false
}

int32_t ICACHE_RAM_ATTR SX127xDriver::GetFrequencyError()
{
#if defined(SX127X_FSK_SUPPORT)
  if (ModFSKorLoRa == SX127x_OPMODE_FSK_OOK)
    return 0; // LoRa FEI registers are the FSK sync value
#endif

  WORD_ALIGNED_ATTR uint8_t reg[3] = {0x0, 0x0, 0x0};
  hal.readRegister(SX127X_REG_FEI_MSB, reg, sizeof(reg), processingPacketRadio);

  uint32_t RegFei = ((reg[0] & 0b0111) << 16) + (reg[1] << 8) + reg[2];

  int32_t intFreqError = RegFei;

  if ((reg[0] & 0b1000) >> 3)
  {
    intFreqError -= 524288; // Sign bit is on
  }

  int32_t fErrorHZ = (intFreqError >> 3) * (SX127xDriver::GetCurrBandwidthNormalisedShifted()); // bit shift hackery so we don't have to use floaty bois; the >> 3 is intentional and is a simplification of the formula on page 114 of sx1276 datasheet
  fErrorHZ >>= 4;

  return fErrorHZ;
}

uint8_t ICACHE_RAM_ATTR SX127xDriver::UnsignedGetLastPacketRSSI(SX12XX_Radio_Number_t radioNumber)
{
  return hal.readRegister(SX127X_REG_PKT_RSSI_VALUE, radioNumber);
}

int8_t ICACHE_RAM_ATTR SX127xDriver::GetLastPacketRSSI(SX12XX_Radio_Number_t radioNumber)
{
#if defined(SX127X_FSK_SUPPORT)
  if (ModFSKorLoRa == SX127x_OPMODE_FSK_OOK)
    return fskLastRssi; // RegRssiValue latched at PayloadReady
#endif
  return ((lowFrequencyMode ? -164 : -157) + hal.readRegister(SX127X_REG_PKT_RSSI_VALUE, radioNumber));
}

int8_t ICACHE_RAM_ATTR SX127xDriver::GetCurrRSSI(SX12XX_Radio_Number_t radioNumber)
{
  return ((lowFrequencyMode ? -164 : -157) + hal.readRegister(SX127X_REG_RSSI_VALUE, radioNumber));
}

int8_t ICACHE_RAM_ATTR SX127xDriver::GetLastPacketSNRRaw(SX12XX_Radio_Number_t radioNumber)
{
#if defined(SX127X_FSK_SUPPORT)
  if (ModFSKorLoRa == SX127x_OPMODE_FSK_OOK)
    return 0; // no SNR estimate in FSK
#endif
  return (int8_t)hal.readRegister(SX127X_REG_PKT_SNR_VALUE, radioNumber);
}

uint8_t ICACHE_RAM_ATTR SX127xDriver::GetIrqFlags(SX12XX_Radio_Number_t radioNumber)
{
#if defined(SX127X_FSK_SUPPORT)
  if (ModFSKorLoRa == SX127x_OPMODE_FSK_OOK)
    return 0; // 0x12 is RegRxBw in FSK
#endif
  return hal.readRegister(SX127X_REG_IRQ_FLAGS, radioNumber);
}

void ICACHE_RAM_ATTR SX127xDriver::ClearIrqFlags(SX12XX_Radio_Number_t radioNumber)
{
#if defined(SX127X_FSK_SUPPORT)
  if (ModFSKorLoRa == SX127x_OPMODE_FSK_OOK)
    return;
#endif
  hal.writeRegister(SX127X_REG_IRQ_FLAGS, SX127X_CLEAR_IRQ_FLAG_ALL, radioNumber);
}

void ICACHE_RAM_ATTR SX127xDriver::IsrCallback_1()
{
    instance->IsrCallback(SX12XX_Radio_1);
}

void ICACHE_RAM_ATTR SX127xDriver::IsrCallback_2()
{
    instance->IsrCallback(SX12XX_Radio_2);
}

void ICACHE_RAM_ATTR SX127xDriver::IsrCallback(SX12XX_Radio_Number_t radioNumber)
{
    instance->processingPacketRadio = radioNumber;
#if defined(SX127X_FSK_SUPPORT)
    if (instance->ModFSKorLoRa == SX127x_OPMODE_FSK_OOK)
    {
        instance->FskIsrCallback(radioNumber);
        return;
    }
#endif
    SX12XX_Radio_Number_t irqClearRadio = radioNumber;

    uint8_t irqStatus = instance->GetIrqFlags(radioNumber);
    if (irqStatus & SX127X_CLEAR_IRQ_FLAG_TX_DONE)
    {
        RFAMP.TXRXdisable();
        instance->TXnbISR();
        irqClearRadio = SX12XX_Radio_All;
    }
    else if (irqStatus & SX127X_CLEAR_IRQ_FLAG_RX_DONE)
    {
        if (instance->RXnbISR(radioNumber))
        {
            irqClearRadio = SX12XX_Radio_All;
        }
#if defined(DEBUG_RCVR_SIGNAL_STATS)
        else
        {
            instance->rxSignalStats[(radioNumber == SX12XX_Radio_1) ? 0 : 1].fail_count++;
        }
#endif
    }
    else if (irqStatus == SX127X_CLEAR_IRQ_FLAG_NONE)
    {
        return;
    }

    instance->ClearIrqFlags(irqClearRadio);
}

// int16_t MeasureNoiseFloor() TODO disabled for now
// {
//     int NUM_READS = RSSI_FLOOR_NUM_READS * FHSSgetChannelCount();
//     float returnval = 0;

//     for (uint32_t freq = 0; freq < FHSSgetChannelCount(); freq++)
//     {
//         FHSSsetCurrIndex(freq);
//         Radio.SetMode(SX127X_CAD);

//         for (int i = 0; i < RSSI_FLOOR_NUM_READS; i++)
//         {
//             returnval = returnval + Radio.GetCurrRSSI();
//             delay(5);
//         }
//     }
//     returnval = returnval / NUM_READS;
//     return (returnval);
// }

// uint8_t SX127xDriver::RunCAD() TODO
// {
//   SetMode(SX127X_STANDBY);

//   hal.writeRegisterBits(SX127X_REG_DIO_MAPPING_1, SX127X_DIO0_CAD_DONE | SX127X_DIO1_CAD_DETECTED, 7, 4);

//   SetMode(SX127X_CAD);
//   ClearIrqFlags();

//   uint32_t startTime = millis();

//   while (!digitalRead(SX127x_dio0))
//   {
//     if (millis() > (startTime + 500))
//     {
//       return (CHANNEL_FREE);
//     }
//     else
//     {
//       //yield();
//       if (digitalRead(SX127x_dio1))
//       {
//         ClearIrqFlags();
//         return (PREAMBLE_DETECTED);
//       }
//     }
//   }

//   ClearIrqFlags();
//   return (CHANNEL_FREE);
// }

#if defined(SX127X_FSK_SUPPORT)
void SX127xDriver::InvalidateLoraCache()
{
  // Values that no setting can equal, so the next Config() rewrites every LoRa register
  currBW = (SX127x_Bandwidth)0xFF;
  currSF = (SX127x_SpreadingFactor)0xFF;
  currCR = (SX127x_CodingRate)0xFF;
  currPreambleLen = 0;
}

bool SX127xDriver::ConfigFsk(const SX127xFskParams_s &params, uint32_t freq, uint8_t payloadLength)
{
  uint8_t bwReg;
  if (!sx127xFskParamsValid(params) || !sx127xFskBwReg(params.rxbw_hz, &bwReg) ||
      payloadLength == 0 || payloadLength > SX127X_FSK_FIFO_SIZE || payloadLength > RXBuffSize)
  {
    ERRLN("SX127x FSK parameters rejected");
    return false;
  }
  PayloadLength = payloadLength;

  // LongRangeMode changes only in Sleep: enter Sleep in the current modem, then select FSK
  hal.writeRegister(SX127X_REG_OP_MODE, SX127x_OPMODE_SLEEP | lowFrequencyMode, SX12XX_Radio_All);
  hal.writeRegister(SX127X_REG_OP_MODE, SX127x_OPMODE_FSK_OOK | SX127x_OPMODE_SLEEP | lowFrequencyMode, SX12XX_Radio_All);
  ModFSKorLoRa = SX127x_OPMODE_FSK_OOK;
  InvalidateLoraCache();
  SetMode(SX127x_OPMODE_STANDBY, SX12XX_Radio_All);

  const SX127xFskBitrateReg_s br = sx127xFskBitrateReg(params.bitrate);
  hal.writeRegister(SX127X_FSK_REG_BITRATE_MSB, (uint8_t)(br.reg >> 8), SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_BITRATE_LSB, (uint8_t)br.reg, SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_BITRATE_FRAC, br.frac, SX12XX_Radio_All);
  const uint16_t fdev = sx127xFskFdevReg(params.fdev_hz);
  hal.writeRegister(SX127X_FSK_REG_FDEV_MSB, (uint8_t)((fdev >> 8) & 0x3F), SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_FDEV_LSB, (uint8_t)fdev, SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_PA_RAMP, params.shaping | SX127X_FSK_PA_RAMP_40_US, SX12XX_Radio_All);
  hal.writeRegisterBits(SX127X_REG_OCP, SX127X_OCP_ON | SX127X_OCP_150MA, SX127X_OCP_MASK, SX12XX_Radio_All);
  hal.writeRegister(SX127X_REG_LNA, SX127X_LNA_BOOST_ON, SX12XX_Radio_All);

  // AFC + AGC on every preamble detection; AFC filter as wide as the channel filter
  fskRxConfig = SX127X_FSK_AFC_AUTO_ON | SX127X_FSK_AGC_AUTO_ON | SX127X_FSK_RX_TRIGGER_PREAMBLE;
  hal.writeRegister(SX127X_FSK_REG_RX_CONFIG, fskRxConfig, SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_RSSI_CONFIG, SX127X_FSK_RSSI_SMOOTHING_32, SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_RX_BW, SX127X_FSK_RXBW_DCC_4_PERCENT | bwReg, SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_AFC_BW, SX127X_FSK_AFCBW_DCC_DEFAULT | bwReg, SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_AFC_FEI, SX127X_FSK_AFC_AUTO_CLEAR_ON, SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_PREAMBLE_DETECT, SX127X_FSK_PREAMBLE_DETECT_2B, SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_PREAMBLE_MSB, 0, SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_PREAMBLE_LSB, params.preamble_bytes, SX12XX_Radio_All);

  // Fixed length, whitening, no address and no hardware CRC (EML CRC16): the FIFO holds
  // exactly the N payload bytes
  hal.writeRegister(SX127X_FSK_REG_SYNC_CONFIG, SX127X_FSK_AUTO_RESTART_RX | SX127X_FSK_SYNC_ON | (uint8_t)(params.sync_size - 1U), SX12XX_Radio_All);
  WORD_ALIGNED_ATTR uint8_t sync[8];
  for (uint8_t i = 0; i < params.sync_size; i++)
    sync[i] = params.sync[i];
  hal.writeRegister(SX127X_FSK_REG_SYNC_VALUE1, sync, params.sync_size, SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_PACKET_CONFIG1, SX127X_FSK_PACKET_FIXED | SX127X_FSK_DCFREE_WHITENING, SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_PACKET_CONFIG2, SX127X_FSK_DATA_MODE_PACKET, SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_PAYLOAD_LENGTH, payloadLength, SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_FIFO_THRESH, SX127X_FSK_TX_START_FIFO_NOT_EMPTY | 0x0F, SX12XX_Radio_All);
  hal.writeRegister(SX127X_FSK_REG_SEQ_CONFIG1, 0x00, SX12XX_Radio_All); // LoRa errata value lands here
  hal.writeRegister(SX127X_FSK_REG_DIO_MAPPING1, SX127X_FSK_DIO0_PAYLOAD_PACKET, SX12XX_Radio_All);

  SetFrequencyReg(freq, SX12XX_Radio_All);

  // Image/RSSI calibration runs at POR on 434 MHz only; repeat it on the operating band
  // (FSK Standby, datasheet §2.1.3.8). Bounded to 20 ms.
  const uint8_t imageCal = hal.readRegister(SX127X_FSK_REG_IMAGE_CAL, SX12XX_Radio_1);
  hal.writeRegister(SX127X_FSK_REG_IMAGE_CAL, imageCal | SX127X_FSK_IMAGECAL_START, SX12XX_Radio_All);
  const uint32_t calStart = micros();
  while ((hal.readRegister(SX127X_FSK_REG_IMAGE_CAL, SX12XX_Radio_1) & SX127X_FSK_IMAGECAL_RUNNING) &&
         micros() - calStart < 20000U) {}

  hal.writeRegister(SX127X_FSK_REG_IRQ_FLAGS2, SX127X_FSK_IRQ2_FIFO_OVERRUN, SX12XX_Radio_All); // clear FIFO
  DBGLN("SX127x FSK br=%u fdev=%u bw=%u", sx127xFskActualBitrate(params.bitrate), sx127xFskActualFdev(params.fdev_hz), params.rxbw_hz);
  return true;
}

// DIO0 in FSK packet mode: PacketSent in TX, PayloadReady in RX
void ICACHE_RAM_ATTR SX127xDriver::FskIsrCallback(SX12XX_Radio_Number_t radioNumber)
{
  const uint8_t flags = hal.readRegister(SX127X_FSK_REG_IRQ_FLAGS2, radioNumber);
  if (flags & SX127X_FSK_IRQ2_PACKET_SENT)
  {
    // Unlike LoRa, FSK stays in TX after PacketSent
    SetMode(SX127x_OPMODE_STANDBY, SX12XX_Radio_All);
    RFAMP.TXRXdisable();
    TXnbISR();
    return;
  }
  if (flags & SX127X_FSK_IRQ2_PAYLOAD_READY)
  {
    // RSSI is not latched in FSK: sample it before the FIFO read, smoothed over 32 samples
    fskLastRssi = (int8_t)(-(int16_t)(hal.readRegister(SX127X_FSK_REG_RSSI_VALUE, radioNumber) >> 1));
    hal.readRegister(SX127X_REG_FIFO, RXdataBuffer, PayloadLength, radioNumber);
    RXdoneCallback(SX12XX_RX_OK);
  }
  if (flags & SX127X_FSK_IRQ2_FIFO_OVERRUN)
  {
    ++FskFifoOverruns;
    // Clears the flag and the FIFO, then restart reception
    hal.writeRegister(SX127X_FSK_REG_IRQ_FLAGS2, SX127X_FSK_IRQ2_FIFO_OVERRUN, radioNumber);
    hal.writeRegister(SX127X_FSK_REG_RX_CONFIG, fskRxConfig | SX127X_FSK_RESTART_RX_NO_PLL, radioNumber);
  }
}
#endif
