#ifndef H_EML_SYNC
#define H_EML_SYNC

#include "eml_frame.h"

#define EML_PROTOCOL_VERSION        1U
#define EML_SYNC_PAYLOAD_SIZE       28U

static_assert(EML_SYNC_PAYLOAD_SIZE <= EML_MAX_PAYLOAD_LEN_32, "Sync must fit EML32");

enum EmlSyncPhase_e : uint8_t
{
    EML_SYNC_HELLO = 0U,  // coordinator -> responder, receiver_epoch=0
    EML_SYNC_OFFER = 1U,  // responder -> coordinator, echo coordinator epoch
    EML_SYNC_ACCEPT = 2U, // coordinator -> responder, echo responder epoch
    EML_SYNC_READY = 3U   // responder -> coordinator, both epochs confirmed
};

enum EmlPhyProfile_e : uint8_t
{
    EML_PHY_LORA_500_SF6_CR45 = 1U,
    EML_PHY_FSK_250 = 2U // protocol ID only; requires future SX127x FSK implementation
};

// Logical representation only. Every multi-byte wire field is explicitly BE.
// Identity/configuration IDs are provisioned; epochs identify fresh attempts.
struct EmlSync_s
{
    uint8_t version;
    uint8_t phase;
    uint8_t phy_profile;
    uint8_t frame_size;
    uint32_t sender_id;
    uint32_t receiver_id;
    uint32_t sender_epoch;
    uint32_t receiver_epoch;
    uint32_t configuration_id;
    uint16_t slot_counter;
    uint16_t hop_index;
};

enum EmlSyncStatus_e : uint8_t
{
    EML_SYNC_OK = 0U,
    EML_SYNC_ERR_NULL_PTR,
    EML_SYNC_ERR_LENGTH,
    EML_SYNC_ERR_VERSION,
    EML_SYNC_ERR_PHASE,
    EML_SYNC_ERR_PHY,
    EML_SYNC_ERR_FRAME_SIZE,
    EML_SYNC_ERR_IDENTITY,
    EML_SYNC_ERR_EPOCH,
    EML_SYNC_ERR_CONFIGURATION,
    EML_SYNC_ERR_SLOT_PARITY
};

/**
 * Validate wire-level Sync fields. The session owner must additionally check
 * expected peer IDs, both epochs, configuration/capability, frame-size equality,
 * hop_index against its table and message phase against its handshake state.
 */
static inline EmlSyncStatus_e emlSyncValidate(const EmlSync_s *sync)
{
    if (sync == nullptr)
        return EML_SYNC_ERR_NULL_PTR;
    if (sync->version != EML_PROTOCOL_VERSION)
        return EML_SYNC_ERR_VERSION;
    if (sync->phase > EML_SYNC_READY)
        return EML_SYNC_ERR_PHASE;
    if (sync->phy_profile != EML_PHY_LORA_500_SF6_CR45 && sync->phy_profile != EML_PHY_FSK_250)
        return EML_SYNC_ERR_PHY;
    if (sync->frame_size != EML_FRAME_SIZE_32 && sync->frame_size != EML_FRAME_SIZE_64)
        return EML_SYNC_ERR_FRAME_SIZE;
    if (sync->sender_id == 0U || sync->receiver_id == 0U || sync->sender_id == sync->receiver_id)
        return EML_SYNC_ERR_IDENTITY;
    if (sync->sender_epoch == 0U ||
        (sync->phase == EML_SYNC_HELLO ? sync->receiver_epoch != 0U : sync->receiver_epoch == 0U))
        return EML_SYNC_ERR_EPOCH;
    if (sync->configuration_id == 0U)
        return EML_SYNC_ERR_CONFIGURATION;
    // TX coordinator owns even slots (HELLO/ACCEPT); RX responder owns odd ones.
    if ((sync->slot_counter & 1U) != (sync->phase & 1U))
        return EML_SYNC_ERR_SLOT_PARITY;
    return EML_SYNC_OK;
}

static inline void emlSyncWrite32(uint8_t *out, uint32_t value)
{
    out[0] = (uint8_t)(value >> 24U);
    out[1] = (uint8_t)(value >> 16U);
    out[2] = (uint8_t)(value >> 8U);
    out[3] = (uint8_t)value;
}

static inline uint32_t emlSyncRead32(const uint8_t *in)
{
    return ((uint32_t)in[0] << 24U) | ((uint32_t)in[1] << 16U) |
           ((uint32_t)in[2] << 8U) | (uint32_t)in[3];
}

// Validate before writing; a larger destination's tail remains untouched.
static inline EmlSyncStatus_e emlSyncEncode(const EmlSync_s *sync, uint8_t *out, size_t outCapacity)
{
    if (sync == nullptr || out == nullptr)
        return EML_SYNC_ERR_NULL_PTR;
    if (outCapacity < EML_SYNC_PAYLOAD_SIZE)
        return EML_SYNC_ERR_LENGTH;
    const EmlSync_s value = *sync;
    const EmlSyncStatus_e status = emlSyncValidate(&value);
    if (status != EML_SYNC_OK)
        return status;

    out[0] = value.version;
    out[1] = value.phase;
    out[2] = value.phy_profile;
    out[3] = value.frame_size;
    emlSyncWrite32(out + 4U, value.sender_id);
    emlSyncWrite32(out + 8U, value.receiver_id);
    emlSyncWrite32(out + 12U, value.sender_epoch);
    emlSyncWrite32(out + 16U, value.receiver_epoch);
    emlSyncWrite32(out + 20U, value.configuration_id);
    out[24] = (uint8_t)(value.slot_counter >> 8U);
    out[25] = (uint8_t)value.slot_counter;
    out[26] = (uint8_t)(value.hop_index >> 8U);
    out[27] = (uint8_t)value.hop_index;
    return EML_SYNC_OK;
}

// Exactly 28 bytes; decode into a temporary so failures never publish fields.
static inline EmlSyncStatus_e emlSyncDecode(const uint8_t *in, size_t inLength, EmlSync_s *out)
{
    if (in == nullptr || out == nullptr)
        return EML_SYNC_ERR_NULL_PTR;
    if (inLength != EML_SYNC_PAYLOAD_SIZE)
        return EML_SYNC_ERR_LENGTH;
    EmlSync_s value = {};
    value.version = in[0];
    value.phase = in[1];
    value.phy_profile = in[2];
    value.frame_size = in[3];
    value.sender_id = emlSyncRead32(in + 4U);
    value.receiver_id = emlSyncRead32(in + 8U);
    value.sender_epoch = emlSyncRead32(in + 12U);
    value.receiver_epoch = emlSyncRead32(in + 16U);
    value.configuration_id = emlSyncRead32(in + 20U);
    value.slot_counter = (uint16_t)((uint16_t)in[24] << 8U) | in[25];
    value.hop_index = (uint16_t)((uint16_t)in[26] << 8U) | in[27];
    const EmlSyncStatus_e status = emlSyncValidate(&value);
    if (status == EML_SYNC_OK)
        *out = value;
    return status;
}

#endif // H_EML_SYNC
