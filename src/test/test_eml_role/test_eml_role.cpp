#include <unity.h>
#include "eml_role.h"

// FSK64 on 60 channels: coordinator dwell 1017 ms per band, 4 bands
static const uint32_t kStep = 1017, kRendezvous = 5U * kStep;
static const uint32_t kListen = kRendezvous + kStep, kAdvertise = 2U * kRendezvous + kStep;
static const uint32_t kMaxExtra = EmlRoleElection::kMaxSteps * kStep;

void test_fixed_roles_never_change()
{
    EmlRoleElection e;
    e.begin(false, EML_RESPONDER, kListen, kAdvertise, kStep, 7, 0);
    TEST_ASSERT_FALSE(e.automatic());
    TEST_ASSERT_FALSE(e.listening());
    TEST_ASSERT_EQUAL(EML_RESPONDER, e.role());
    TEST_ASSERT_FALSE(e.update(1000000, false));
    e.begin(false, EML_COORDINATOR, kListen, kAdvertise, kStep, 7, 0);
    TEST_ASSERT_FALSE(e.update(1000000, false));
    TEST_ASSERT_EQUAL(EML_COORDINATOR, e.role());
}

void test_auto_listens_then_advertises_and_hold_restarts_phase()
{
    EmlRoleElection e;
    e.begin(true, EML_COORDINATOR, kListen, kAdvertise, kStep, 12345, 100);
    TEST_ASSERT_EQUAL(EML_RESPONDER, e.role());
    TEST_ASSERT_TRUE(e.listening());
    const uint32_t listen = e.phaseMs();
    TEST_ASSERT_TRUE(listen >= kListen && listen <= kListen + kMaxExtra);
    TEST_ASSERT_EQUAL(0, (listen - kListen) % kStep);

    // A handshake in progress keeps the role and restarts the phase
    TEST_ASSERT_FALSE(e.update(100 + listen - 1, false));
    TEST_ASSERT_FALSE(e.update(100 + listen + 50, true));
    TEST_ASSERT_FALSE(e.update(100 + 2 * listen + 49, false));
    TEST_ASSERT_TRUE(e.update(100 + 2 * listen + 50, false));
    TEST_ASSERT_EQUAL(EML_COORDINATOR, e.role());
    TEST_ASSERT_FALSE(e.listening());
    const uint32_t advertise = e.phaseMs();
    TEST_ASSERT_TRUE(advertise >= kAdvertise && advertise <= kAdvertise + kMaxExtra);

    TEST_ASSERT_TRUE(e.update(100 + 2 * listen + 50 + advertise, false));
    TEST_ASSERT_EQUAL(EML_RESPONDER, e.role());
}

void test_draws_cover_every_step_and_zero_seed_is_valid()
{
    unsigned seen = 0;
    for (uint32_t seed = 0; seed < 64; ++seed)
    {
        EmlRoleElection e;
        e.begin(true, EML_COORDINATOR, kListen, kAdvertise, kStep, seed * 2654435761UL, 0);
        seen |= 1U << ((e.phaseMs() - kListen) / kStep);
    }
    TEST_ASSERT_EQUAL_HEX(0xFF, seen);
}

// Discrete model of the rendezvous bounds documented in eml_role.h: a listener
// hears a coordinator that started while it listened within one step (default
// band), an older coordinator within R; a fixed responder within 2R.
struct SimNode
{
    EmlRoleElection e;
    bool automatic = false, started = false, linked = false;
    uint32_t boot = 0, roleSince = 0;
    EmlRole_e fixedRole = EML_COORDINATOR;
};

static void start(SimNode &n, uint32_t seed, uint32_t now)
{
    n.e.begin(n.automatic, n.fixedRole, kListen, kAdvertise, kStep, seed, now);
    n.started = true;
    n.roleSince = now;
}

static bool hears(const SimNode &listener, const SimNode &coordinator, uint32_t now)
{
    if (listener.e.role() != EML_RESPONDER || coordinator.e.role() != EML_COORDINATOR) return false;
    uint32_t due;
    if (!listener.automatic)
        due = (listener.roleSince > coordinator.roleSince ? listener.roleSince : coordinator.roleSince) + 2U * kRendezvous;
    else if (coordinator.roleSince >= listener.roleSince)
        due = coordinator.roleSince + kStep;
    else
        due = listener.roleSince + kRendezvous;
    return now >= due;
}

// Returns the link time, or 0 if no link within the limit
static uint32_t simulate(SimNode &a, SimNode &b, uint32_t seedA, uint32_t seedB, uint32_t limitMs)
{
    for (uint32_t now = 0; now <= limitMs; now += 10)
    {
        SimNode *nodes[2] = {&a, &b};
        const uint32_t seeds[2] = {seedA, seedB};
        for (int i = 0; i < 2; ++i)
        {
            SimNode &n = *nodes[i];
            if (!n.started && now >= n.boot) start(n, seeds[i], now);
            if (n.started && n.e.update(now, false)) n.roleSince = now;
        }
        if (a.started && b.started && (hears(a, b, now) || hears(b, a, now)))
        {
            a.linked = b.linked = true;
            return now;
        }
    }
    return 0;
}

void test_two_auto_nodes_converge_to_opposite_roles()
{
    uint32_t worst = 0, total = 0;
    const uint32_t runs = 400;
    for (uint32_t i = 0; i < runs; ++i)
    {
        SimNode a, b;
        a.automatic = b.automatic = true;
        b.boot = (i * 37U) % 3000U; // independent power-up
        const uint32_t linked = simulate(a, b, 0xA5A50000UL + i * 7919U, 0x5A5A0000UL + i * 104729U, 1200000);
        TEST_ASSERT_NOT_EQUAL(0, linked);
        TEST_ASSERT_NOT_EQUAL(a.e.role(), b.e.role());
        if (linked > worst) worst = linked;
        total += linked;
    }
    // Typical: within a few phases; equal draws only add further phases.
    TEST_ASSERT_LESS_THAN_UINT32(4U * (kListen + kAdvertise + 2U * kMaxExtra), worst);
    TEST_ASSERT_LESS_THAN_UINT32(kListen + kAdvertise + 2U * kMaxExtra, total / runs);
}

void test_auto_with_fixed_peer_links_in_its_first_matching_phase()
{
    for (uint32_t i = 0; i < 64; ++i)
    {
        SimNode a, master;
        a.automatic = true;
        master.fixedRole = EML_COORDINATOR;
        a.boot = (i * 53U) % 3000U;
        const uint32_t linked = simulate(a, master, 0x1000U + i, 0, 600000);
        TEST_ASSERT_NOT_EQUAL(0, linked);
        TEST_ASSERT_EQUAL(EML_RESPONDER, a.e.role());
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(a.boot + 20U + kRendezvous, linked); // first listen phase; start and detection on a 10 ms grid

        SimNode b, slave;
        b.automatic = true;
        slave.fixedRole = EML_RESPONDER;
        slave.boot = (i * 53U) % 3000U;
        const uint32_t linked2 = simulate(b, slave, 0x2000U + i, 0, 600000);
        TEST_ASSERT_NOT_EQUAL(0, linked2);
        TEST_ASSERT_EQUAL(EML_COORDINATOR, b.e.role());
        // first advertise phase, which is >= 2R
        TEST_ASSERT_LESS_OR_EQUAL_UINT32(kListen + kMaxExtra + 2U * kRendezvous + 3000U, linked2);
    }
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_fixed_roles_never_change);
    RUN_TEST(test_auto_listens_then_advertises_and_hold_restarts_phase);
    RUN_TEST(test_draws_cover_every_step_and_zero_seed_is_valid);
    RUN_TEST(test_two_auto_nodes_converge_to_opposite_roles);
    RUN_TEST(test_auto_with_fixed_peer_links_in_its_first_matching_phase);
    return UNITY_END();
}
