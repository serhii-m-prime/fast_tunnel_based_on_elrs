#pragma once

#include <stdint.h>
#include "eml_control.h"

// Foreground-only dynamic-power policy. Reports describe how well the peer is
// receiving this node. Bad feedback raises power quickly; reductions require a
// sustained clean link. Once reports have started, missing feedback fails safe
// to the configured maximum instead of silently remaining at a weak level.
class EmlDynamicPower
{
    uint8_t min_ = 0, max_ = 0, current_ = 0, degradeLq_ = 60;
    uint32_t lastReportMs_ = 0, lastChangeMs_ = 0, goodSinceMs_ = 0;
    bool enabled_ = false, haveReport_ = false, goodValid_ = false;

public:
    void begin(bool enabled, uint8_t minimum, uint8_t maximum, uint8_t current,
               uint8_t degradeLq, uint32_t nowMs)
    {
        enabled_ = enabled; min_ = minimum; max_ = maximum;
        current_ = current < minimum ? minimum : current > maximum ? maximum : current;
        degradeLq_ = degradeLq; lastReportMs_ = lastChangeMs_ = nowMs;
        goodSinceMs_ = 0; haveReport_ = goodValid_ = false;
    }
    bool enabled() const { return enabled_; }
    uint8_t power() const { return current_; }
    bool receive(const EmlLinkReport_s &r, uint32_t nowMs)
    {
        if (!enabled_) return false;
        haveReport_ = true; lastReportMs_ = nowMs;
        const bool bad = r.lq < degradeLq_ || r.per > (uint8_t)(100U - degradeLq_) || r.rssi < -105;
        const bool good = r.lq >= 95 && r.per <= 5 && r.rssi >= -75 && (!r.snr_valid || r.snr >= 5);
        if (bad)
        {
            goodValid_ = false;
            if (current_ < max_ && nowMs - lastChangeMs_ >= 250U)
            { ++current_; lastChangeMs_ = nowMs; return true; }
            return false;
        }
        if (!good) { goodValid_ = false; return false; }
        if (!goodValid_) { goodValid_ = true; goodSinceMs_ = nowMs; return false; }
        if (current_ > min_ && nowMs - goodSinceMs_ >= 5000U && nowMs - lastChangeMs_ >= 5000U)
        {
            --current_; lastChangeMs_ = goodSinceMs_ = nowMs;
            return true;
        }
        return false;
    }
    bool tick(uint32_t nowMs)
    {
        if (enabled_ && haveReport_ && nowMs - lastReportMs_ >= 2000U && current_ != max_)
        {
            current_ = max_; lastChangeMs_ = nowMs; goodValid_ = false;
            return true;
        }
        return false;
    }
};
