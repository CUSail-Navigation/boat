/**
 * SAIL TRIM TESTS -- SailTrimAlgo.
 * Tests in this file cover the goal sail angles that the deterministic algorithm writes for a given wind angle
 * (e.g. all-in inside the no-go zone, all-out dead downwind, symmetric otherwise), and that it only updates
 * once every UPDATE_PERIOD_MS.
 */
#include "test_support.h"
#include "suites.hpp"
#include "sailing/sail_trim_algo.hpp"


// Helper functions.
/** With the wind staged at @c wind_angle, let a full update period pass and run the algorithm once. */
static void trim_for(SailTrimAlgo& algorithm, const uint16_t wind_angle) {
    sfr::anemometer::wind_angle = wind_angle;
    mock_advance_millis(constants::algo::UPDATE_PERIOD_MS);
    algorithm.execute();
}

static void assert_sails_all_in(const char* message) {
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(constants::servo::MAINSAIL_MIN_ANGLE, sfr::algo::mainsail_angle, message);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(constants::servo::JIB_MIN_ANGLE, sfr::algo::jib_angle, message);
}


// Tests.
static void test_head_to_wind_is_in_irons() {
    SailTrimAlgo algorithm;
    trim_for(algorithm, 0);
    assert_sails_all_in("Head-to-wind, both sails should be pulled all the way in");
}

static void test_dead_downwind_lets_sails_all_out() {
    SailTrimAlgo algorithm;
    trim_for(algorithm, 180);

    TEST_ASSERT_EQUAL_UINT8(constants::servo::MAINSAIL_MAX_ANGLE, sfr::algo::mainsail_angle);
    TEST_ASSERT_EQUAL_UINT8(constants::servo::JIB_MAX_ANGLE, sfr::algo::jib_angle);
}

static void test_no_go_zone_edges() {
    SailTrimAlgo algorithm;

    for (const uint16_t wind : {constants::algo::NO_GO_HALF_WIDTH - 1, 360 - constants::algo::NO_GO_HALF_WIDTH + 1}) {
        trim_for(algorithm, wind);
        assert_sails_all_in("Just inside the no-go zone, both sails should be pulled all the way in");
    }

    trim_for(algorithm, constants::algo::NO_GO_HALF_WIDTH);
    assert_sails_all_in("At the edge of the no-go zone, the linear range should start from all-in");
}

static void test_sail_angles_rise_from_no_go_edge_to_dead_downwind() {
    SailTrimAlgo algorithm;
    std::vector<uint32_t> mainsail_sweep, jib_sweep;

    for (uint16_t wind = constants::algo::NO_GO_HALF_WIDTH; wind <= 180; ++wind) {
        trim_for(algorithm, wind);
        mainsail_sweep.push_back(sfr::algo::mainsail_angle);
        jib_sweep.push_back(sfr::algo::jib_angle);
    }

    assert_non_decreasing(mainsail_sweep, "mainsail goal angle");
    assert_non_decreasing(jib_sweep, "jib goal angle");
}

static void test_angles_are_symmetric_and_jib_side_follows_the_wind() {
    SailTrimAlgo algorithm;

    trim_for(algorithm, 90);
    const uint8_t stb_mainsail = sfr::algo::mainsail_angle;
    const uint8_t stb_jib = sfr::algo::jib_angle;
    TEST_ASSERT_EQUAL_UINT8(constants::servo::JIB_SIDE_STB, sfr::algo::jib_side_flag);

    trim_for(algorithm, 270);
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(stb_mainsail, sfr::algo::mainsail_angle, "Mainsail angle should be symmetric");
    TEST_ASSERT_EQUAL_UINT8_MESSAGE(stb_jib, sfr::algo::jib_angle, "Jib angle should be symmetric");
    TEST_ASSERT_EQUAL_UINT8(constants::servo::JIB_SIDE_PORT, sfr::algo::jib_side_flag);
}

static void test_only_updates_once_per_period() {
    SailTrimAlgo algorithm;
    trim_for(algorithm, 180);
    sfr::algo::update_servos_algo = false;

    sfr::anemometer::wind_angle = 0;
    mock_advance_millis(constants::algo::UPDATE_PERIOD_MS - 1);
    algorithm.execute();
    TEST_ASSERT_FALSE_MESSAGE(sfr::algo::update_servos_algo, "No update should happen before UPDATE_PERIOD_MS has passed");
    TEST_ASSERT_EQUAL_UINT8(constants::servo::MAINSAIL_MAX_ANGLE, sfr::algo::mainsail_angle);

    mock_advance_millis(1);
    algorithm.execute();
    TEST_ASSERT_TRUE_MESSAGE(sfr::algo::update_servos_algo, "An update should happen once UPDATE_PERIOD_MS has passed");
    assert_sails_all_in("The new wind angle should be picked up on the next update");
}


// Runner.
void run_sail_trim_tests() {
    Unity.TestFile = __FILE__; // Report failures against this file, not main.cpp.
    RUN_TEST(test_head_to_wind_is_in_irons);
    RUN_TEST(test_dead_downwind_lets_sails_all_out);
    RUN_TEST(test_no_go_zone_edges);
    RUN_TEST(test_sail_angles_rise_from_no_go_edge_to_dead_downwind);
    RUN_TEST(test_angles_are_symmetric_and_jib_side_follows_the_wind);
    RUN_TEST(test_only_updates_once_per_period);
}
