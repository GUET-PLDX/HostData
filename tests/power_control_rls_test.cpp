#include <array>
#include <cmath>
#include <iostream>
#include <limits>

#include "Modules/PowerControl/RLS.hpp"

namespace {

int failures = 0;

void check(bool condition, const char* expression, int line) {
  if (!condition) {
    std::cerr << "line " << line << ": CHECK(" << expression << ") failed\n";
    ++failures;
  }
}

void check_near(float actual, float expected, float tolerance,
                const char* expression, int line) {
  if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
    std::cerr << "line " << line << ": " << expression << " expected "
              << expected << " +/- " << tolerance << ", got " << actual
              << '\n';
    ++failures;
  }
}

#define CHECK(CONDITION) check((CONDITION), #CONDITION, __LINE__)
#define CHECK_NEAR(ACTUAL, EXPECTED, TOLERANCE)                           \
  check_near((ACTUAL), (EXPECTED), (TOLERANCE), #ACTUAL " ~= " #EXPECTED, \
             __LINE__)

void test_convergence_and_rejected_updates_are_transactional() {
  using Estimator = RLS<2>;
  const Estimator::ParamVector INITIAL = {0.22f, 1.2f};
  Estimator estimator(1000.0f, 0.995f, INITIAL);

  for (int index = 0; index < 800; ++index) {
    const float X0 = 5.0f + static_cast<float>(index % 17) * 2.0f;
    const float X1 = 0.2f + static_cast<float>(index % 13) * 0.4f;
    const Estimator::ParamVector SAMPLE = {X0, X1};
    CHECK(estimator.Update(SAMPLE, 0.35f * X0 + 1.7f * X1));
  }

  const Estimator::ParamVector TRUSTED = estimator.GetParamVector();
  CHECK_NEAR(TRUSTED[0], 0.35f, 0.01f);
  CHECK_NEAR(TRUSTED[1], 1.7f, 0.03f);

  CHECK(!estimator.Update({0.0f, 0.0f}, 10.0f));
  CHECK(!estimator.Update(
      {std::numeric_limits<float>::quiet_NaN(), 1.0f}, 10.0f));
  CHECK(!estimator.Update(
      {1.0f, 1.0f}, std::numeric_limits<float>::infinity()));
  CHECK_NEAR(estimator.GetParamVector()[0], TRUSTED[0], 1.0e-6f);
  CHECK_NEAR(estimator.GetParamVector()[1], TRUSTED[1], 1.0e-6f);
}

void test_reset_and_invalid_constructor_values_are_safe() {
  using Estimator = RLS<2>;
  Estimator estimator(-1.0f, 2.0f, {0.3f, 1.5f});
  CHECK(estimator.Update({10.0f, 2.0f}, 6.0f));

  estimator.Reset({0.4f, 1.6f});
  CHECK_NEAR(estimator.GetParamVector()[0], 0.4f, 1.0e-6f);
  CHECK_NEAR(estimator.GetParamVector()[1], 1.6f, 1.0e-6f);

  estimator.SetParamVector(
      {std::numeric_limits<float>::quiet_NaN(), 2.0f});
  CHECK_NEAR(estimator.GetParamVector()[0], 0.0f, 1.0e-6f);
  CHECK_NEAR(estimator.GetParamVector()[1], 2.0f, 1.0e-6f);
}

void test_parameter_bounds_apply_to_all_parameter_writes() {
  using Estimator = RLS<2>;
  Estimator estimator(1000.0f, 0.999f, {0.22f, 1.2f});
  estimator.SetParamBounds({0.0f, 0.0f}, {0.3f, 1.5f});

  estimator.SetParamVector({-1.0f, 4.0f});
  CHECK_NEAR(estimator.GetParamVector()[0], 0.0f, 1.0e-6f);
  CHECK_NEAR(estimator.GetParamVector()[1], 1.5f, 1.0e-6f);

  estimator.Reset({0.5f, -1.0f});
  CHECK_NEAR(estimator.GetParamVector()[0], 0.3f, 1.0e-6f);
  CHECK_NEAR(estimator.GetParamVector()[1], 0.0f, 1.0e-6f);

  CHECK(estimator.Update({10.0f, 10.0f}, 100.0f));
  CHECK_NEAR(estimator.GetParamVector()[0], 0.3f, 1.0e-6f);
  CHECK_NEAR(estimator.GetParamVector()[1], 0.075f, 1.0e-6f);

  estimator.SetParamBounds({3.0f, 2.0f}, {1.0f, -1.0f});
  estimator.SetParamVector({0.0f, 4.0f});
  CHECK_NEAR(estimator.GetParamVector()[0], 1.0f, 1.0e-6f);
  CHECK_NEAR(estimator.GetParamVector()[1], 2.0f, 1.0e-6f);
}

void test_bounded_updates_recover_after_saturating_at_a_limit() {
  RLS<1> estimator(1000.0f, 1.0f, {0.5f});
  estimator.SetParamBounds({0.0f}, {1.0f});

  CHECK(estimator.Update({1.0f}, 1000.0f));
  CHECK_NEAR(estimator.GetParamVector()[0], 0.55f, 1.0e-5f);
  for (int index = 0; index < 9; ++index) {
    CHECK(estimator.Update({1.0f}, 1000.0f));
  }
  CHECK_NEAR(estimator.GetParamVector()[0], 1.0f, 1.0e-5f);

  for (int index = 0; index < 100; ++index) {
    CHECK(!estimator.Update({1.0f}, 1000.0f));
  }
  CHECK_NEAR(estimator.GetParamVector()[0], 1.0f, 1.0e-5f);

  CHECK(estimator.Update({1.0f}, 0.2f));
  CHECK_NEAR(estimator.GetParamVector()[0], 0.95f, 1.0e-4f);
  for (int index = 0; index < 15; ++index) {
    CHECK(estimator.Update({1.0f}, 0.2f));
  }
  CHECK_NEAR(estimator.GetParamVector()[0], 0.2f, 0.02f);
}

void test_boundary_limited_update_advances_one_dimensional_covariance() {
  RLS<1> estimator(1.0f, 1.0f, {0.98f});
  estimator.SetParamBounds({0.0f}, {1.0f});

  CHECK(estimator.Update({1.0f}, 1.03f));
  CHECK_NEAR(estimator.GetParamVector()[0], 1.0f, 1.0e-6f);

  CHECK(estimator.Update({1.0f}, 0.98f));
  CHECK_NEAR(estimator.GetParamVector()[0], 0.9931579f, 1.0e-5f);
  CHECK(estimator.Update({1.0f}, 0.98f));
  CHECK_NEAR(estimator.GetParamVector()[0], 0.9898039f, 1.0e-5f);
}

void test_bounded_components_update_independently() {
  RLS<2> estimator(1000.0f, 1.0f, {1.0f, 0.0f});
  estimator.SetParamBounds({0.0f, 0.0f}, {1.0f, 1.0f});

  for (int index = 0; index < 40; ++index) {
    CHECK(estimator.Update({1.0f, 1.0f}, 2.0f));
    CHECK_NEAR(estimator.GetParamVector()[0], 1.0f, 1.0e-6f);
  }
  CHECK_NEAR(estimator.GetParamVector()[1], 1.0f, 0.02f);
}

void test_pinned_component_still_advances_shared_covariance() {
  RLS<2> estimator(1.0f, 1.0f, {0.98f, 0.0f});
  estimator.SetParamBounds({0.0f, 0.0f}, {1.0f, 10.0f});

  CHECK(estimator.Update({1.0f, 1.0f}, 1.06f));
  CHECK_NEAR(estimator.GetParamVector()[0], 1.0f, 1.0e-6f);
  CHECK_NEAR(estimator.GetParamVector()[1], 0.0266667f, 1.0e-5f);

  CHECK(estimator.Update({1.0f, 1.0f}, 1.16f));
  CHECK_NEAR(estimator.GetParamVector()[0], 1.0f, 1.0e-6f);
  CHECK_NEAR(estimator.GetParamVector()[1], 0.08f, 1.0e-5f);

  CHECK(estimator.Update({1.0f, 1.0f}, 1.20f));
  CHECK_NEAR(estimator.GetParamVector()[0], 1.0f, 1.0e-6f);
  CHECK_NEAR(estimator.GetParamVector()[1], 0.1142857f, 1.0e-5f);

  CHECK(estimator.Update({1.0f, 1.0f}, 1.24f));
  CHECK_NEAR(estimator.GetParamVector()[0], 1.0f, 1.0e-6f);
  CHECK_NEAR(estimator.GetParamVector()[1], 0.1422222f, 1.0e-5f);
}

void test_ill_conditioned_updates_remain_finite() {
  using Estimator = RLS<2>;
  Estimator estimator(1000.0f, 0.9999f, {0.22f, 1.2f});
  estimator.SetParamBounds({0.0f, 0.0f}, {5.0f, 20.0f});

  int accepted = 0;
  for (int index = 0; index < 20000; ++index) {
    const float X0 = 10000.0f + static_cast<float>(index % 7);
    const float X1 = 0.001f + static_cast<float>(index % 5) * 1.0e-6f;
    if (estimator.Update({X0, X1}, 0.35f * X0 + 1.7f * X1)) {
      ++accepted;
    }
    CHECK(std::isfinite(estimator.GetParamVector()[0]));
    CHECK(std::isfinite(estimator.GetParamVector()[1]));
  }
  CHECK(accepted > 0);

  const Estimator::ParamVector TRUSTED = estimator.GetParamVector();
  CHECK(!estimator.Update({std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max()},
                          std::numeric_limits<float>::max()));
  CHECK_NEAR(estimator.GetParamVector()[0], TRUSTED[0], 1.0e-6f);
  CHECK_NEAR(estimator.GetParamVector()[1], TRUSTED[1], 1.0e-6f);
  CHECK(estimator.Update({10.0f, 2.0f}, 6.9f));
  CHECK(std::isfinite(estimator.GetParamVector()[0]));
  CHECK(std::isfinite(estimator.GetParamVector()[1]));
}

}  // namespace

int main() {
  test_convergence_and_rejected_updates_are_transactional();
  test_reset_and_invalid_constructor_values_are_safe();
  test_parameter_bounds_apply_to_all_parameter_writes();
  test_bounded_updates_recover_after_saturating_at_a_limit();
  test_boundary_limited_update_advances_one_dimensional_covariance();
  test_bounded_components_update_independently();
  test_pinned_component_still_advances_shared_covariance();
  test_ill_conditioned_updates_remain_finite();
  if (failures != 0) {
    std::cerr << failures << " RLS checks failed\n";
    return 1;
  }
  std::cout << "all RLS checks passed\n";
  return 0;
}
