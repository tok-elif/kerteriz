/// \file
/// S2 duman testi — bagimliliklarin baglandigini dogrular.
/// Konvansiyon ve matematik testleri Faz 0 - S5..S8'de gelir.

#include "kerteriz/version.hpp"

#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <manif/SE_2_3.h>
#include <manif/SO3.h>

TEST(Dependencies, VersionHeaderIsReachable) {
  EXPECT_EQ(kerteriz::kVersionMajor, 0);
  EXPECT_EQ(kerteriz::kVersionMinor, 1);
}

TEST(Dependencies, EigenLinks) {
  const Eigen::Matrix3d m = Eigen::Matrix3d::Identity();
  EXPECT_DOUBLE_EQ(m.determinant(), 1.0);
}

TEST(Dependencies, ManifSe23IsAvailable) {
  // ADR-2: navigasyon cekirdegi SE_2(3) uzerindedir.
  // Burada yalnizca tipin var oldugu ve derlendigi dogrulanir.
  const manif::SE_2_3d x = manif::SE_2_3d::Identity();
  EXPECT_EQ(manif::SE_2_3d::DoF, 9);
  EXPECT_TRUE(x.translation().isZero());
}

TEST(Dependencies, CppStandardIsExactly17) {
  // R8 / ADR-12: C++20 kullanilmaz.
  EXPECT_EQ(__cplusplus, 201703L);
}
