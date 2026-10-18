#include "../../test/support/eml_tdd_sim.h"
#include <iostream>
#include <iomanip>

static bool firstRow = true;
static uint64_t percentile(std::vector<uint64_t> values, unsigned percent)
{
    if (values.empty()) return 0;
    std::sort(values.begin(), values.end());
    return values[((values.size() - 1U) * percent + 99U) / 100U];
}

template <uint8_t Window>
static void measure(const EmlMacProfile_s &p, const char *name, bool adaptive, unsigned scenario, unsigned loss)
{
    EmlTddSimulation<Window> sim(p, adaptive, Window > 1);
    sim.establish(); sim.loss = loss;
    const uint64_t startUs = sim.nowUs(), startAir = sim.airtime_us;
    size_t target[2] = {32768,0};
    const char *label = "forward";
    if (scenario == 1) { target[0]=0; target[1]=32768; label="reverse"; }
    if (scenario == 2) { target[1]=32768; label="symmetric"; }
    if (scenario == 3) { target[1]=4096; label="asymmetric"; }
    if (scenario == 4) { target[1]=640; label="interactive"; }
    if (scenario == 5) { target[1]=8192; label="uart_pause"; }
    for (unsigned i=0;i<30000 && (sim.delivered[0]<target[0] || sim.delivered[1]<target[1] ||
                                 sim.transport[0].queuedTx() || sim.transport[1].queuedTx());++i)
    {
        for (uint8_t r=0;r<2;++r) if (sim.accepted[r]<target[r])
        {
            const bool interactive = scenario == 4 && r == 1;
            if (!interactive || i % 32 == 0) sim.feed(r,std::min(interactive ? (size_t)20 : (size_t)64,target[r]-sim.accepted[r]));
        }
        sim.step(scenario != 5 || i % 160 >= 40);
    }
    if (sim.delivered[0]!=target[0] || sim.delivered[1]!=target[1])
    {
        std::cerr << name << " scenario=" << label << " phy=" << (unsigned)p.phy_profile << " N=" << (unsigned)p.frame_size
            << " loss=" << loss << " accepted=" << sim.accepted[0] << ',' << sim.accepted[1]
            << " delivered=" << sim.delivered[0] << ',' << sim.delivered[1]
            << " queued=" << sim.transport[0].queuedTx() << ',' << sim.transport[1].queuedTx()
            << " expected=" << (unsigned)sim.transport[0].expectedRxSequence() << ',' << (unsigned)sim.transport[1].expectedRxSequence()
            << " backpressure=" << sim.transport[0].stats().backpressure << ',' << sim.transport[1].stats().backpressure
            << " base=" << (unsigned)sim.transport[0].nextTxSequence() << ',' << (unsigned)sim.transport[1].nextTxSequence()
            << " flight=" << (unsigned)sim.transport[0].inFlight() << ',' << (unsigned)sim.transport[1].inFlight()
            << " flow=" << (unsigned)sim.transport[0].flowControl() << ',' << (unsigned)sim.transport[1].flowControl()
            << " duplicates=" << sim.transport[0].stats().duplicates << ',' << sim.transport[1].stats().duplicates
            << " invalid=" << sim.transport[0].stats().invalid << ',' << sim.transport[1].stats().invalid
            << " retries=" << sim.transport[0].stats().retransmitted << ',' << sim.transport[1].stats().retransmitted << '\n';
        throw std::runtime_error("benchmark did not deliver all accepted bytes");
    }
    if (sim.transport[0].stats().session_resets || sim.transport[1].stats().session_resets) throw std::runtime_error("benchmark reset its session");
    const uint64_t duration = sim.nowUs()-startUs;
    if (!firstRow) std::cout << ",\n";
    firstRow = false;
    std::cout << "{\"phy\":\"" << (emlIsFsk(p)?"FSK250":"LoRa500SF6") << "\",\"frame_size\":" << (unsigned)p.frame_size
        << ",\"variant\":\"" << name << "\",\"window\":" << (unsigned)Window << ",\"scenario\":\"" << label
        << "\",\"loss_percent\":" << loss << ",\"duration_us\":" << duration
        << ",\"tx_airtime_us\":" << sim.airtime_us-startAir << ",\"collisions\":" << sim.collisions << ",\"directions\":[";
    for(uint8_t r=0;r<2;++r)
    {
        if(r) std::cout << ',';
        const auto &s=sim.transport[r].stats();
        std::cout << "{\"bytes\":" << sim.delivered[r] << ",\"bytes_per_second\":" << std::fixed << std::setprecision(2)
            << sim.delivered[r]*1000000.0/duration << ",\"latency_p50_us\":" << percentile(sim.latency_us[r],50)
            << ",\"latency_p95_us\":" << percentile(sim.latency_us[r],95) << ",\"latency_p99_us\":" << percentile(sim.latency_us[r],99)
            << ",\"retransmissions\":" << s.retransmitted << ",\"out_of_order\":" << s.out_of_order << '}';
    }
    std::cout << "]}";
}

int main()
{
    std::cout << "{\"measurement\":\"deterministic native model; no hardware measurements\",\"rows\":[\n";
    for(auto p : {EML_PROFILE_FSK64,EML_PROFILE_FSK32,EML_PROFILE_LORA64,EML_PROFILE_LORA32})
        for(unsigned loss : {0U,10U}) for(unsigned scenario=0;scenario<6;++scenario)
        {
            measure<1>(p,"symmetric_w1",false,scenario,loss);
            measure<1>(p,"adaptive_w1",true,scenario,loss);
            measure<4>(p,"symmetric_w4",false,scenario,loss);
            measure<4>(p,"adaptive_w4",true,scenario,loss);
        }
    std::cout << "\n]}\n";
}
