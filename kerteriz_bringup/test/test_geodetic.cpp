/// \file
/// F1.8 — WGS84 LLH -> ECEF -> yerel ENU donusumu.

#include "kerteriz_bringup/geodetic.hpp"

#include <Eigen/LU> // yalnizca determinant(); acik ters alma KULLANILMAZ (ADR-17)
#include <cmath>
#include <gtest/gtest.h>

namespace {

using kerteriz::Scalar;
using kerteriz::Vec3;
using kerteriz_bringup::EnuProjector;
using kerteriz_bringup::Llh;
using kerteriz_bringup::llh_to_ecef;

constexpr Scalar kDeg = 3.14159265358979323846 / 180.0;

Llh orijin() { return Llh{49.0 * kDeg, 8.43 * kDeg, 116.0}; }

TEST(Geodetic, OriginMapsExactlyToZero) {
  const EnuProjector p(orijin());
  const Vec3 e = p.to_enu(orijin());
  EXPECT_LT(e.norm(), 1e-9) << "orijin [0,0,0] vermedi: " << e.transpose();
}

TEST(Geodetic, AxesHaveCorrectSignAndRoughMagnitude) {
  const EnuProjector p(orijin());

  // Kuzeye 1e-5 derece ~ 1.11 m; dogu bileseni ihmal edilebilir olmali.
  Llh kuzey = orijin();
  kuzey.lat_rad += 1e-5 * kDeg;
  const Vec3 n = p.to_enu(kuzey);
  EXPECT_GT(n.y(), 0.0) << "enlem artisi KUZEY'i artirmali";
  EXPECT_NEAR(n.y(), 1.11, 0.05);
  EXPECT_LT(std::abs(n.x()), 1e-3);
  EXPECT_LT(std::abs(n.z()), 1e-3);

  // Doguya 1e-5 derece; 49 derece enlemde ~ 0.73 m.
  Llh dogu = orijin();
  dogu.lon_rad += 1e-5 * kDeg;
  const Vec3 e = p.to_enu(dogu);
  EXPECT_GT(e.x(), 0.0) << "boylam artisi DOGU'yu artirmali";
  EXPECT_NEAR(e.x(), 0.73, 0.05);
  EXPECT_LT(std::abs(e.y()), 1e-3);

  // Yukselti dogrudan yukari.
  Llh yukari = orijin();
  yukari.alt_m += 10.0;
  const Vec3 u = p.to_enu(yukari);
  EXPECT_NEAR(u.z(), 10.0, 1e-6);
  EXPECT_LT(std::abs(u.x()), 1e-6);
  EXPECT_LT(std::abs(u.y()), 1e-6);
}

TEST(Geodetic, SouthAndWestAreNegative) {
  // Isaretlerin yalnizca bir yonde dogru olmadigini da gosterir.
  const EnuProjector p(orijin());
  Llh q = orijin();
  q.lat_rad -= 1e-5 * kDeg;
  q.lon_rad -= 1e-5 * kDeg;
  const Vec3 e = p.to_enu(q);
  EXPECT_LT(e.x(), 0.0);
  EXPECT_LT(e.y(), 0.0);
}

TEST(Geodetic, IsDeterministicAndFinite) {
  const EnuProjector p(orijin());
  Llh q = orijin();
  q.lat_rad += 1e-4 * kDeg;
  q.lon_rad += 2e-4 * kDeg;
  q.alt_m += 3.0;

  const Vec3 a = p.to_enu(q);
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(p.to_enu(q), a) << "ayni girdi ayni cikti vermedi";
  }
  EXPECT_TRUE(a.allFinite());
}

TEST(Geodetic, EcefMagnitudeIsEarthSized) {
  // Kaba akil sagligi: ECEF yaricapi yerin yaricapi mertebesinde olmali.
  const Vec3 e = llh_to_ecef(orijin());
  EXPECT_GT(e.norm(), 6.3e6);
  EXPECT_LT(e.norm(), 6.4e6);
}

TEST(Geodetic, EnuRotationIsOrthonormal) {
  // Acik ters alma yasaktir (ADR-17); bu matrisin tersi transpozudur ve kod
  // da buna dayanir. Ozellik burada dogrulanir.
  const auto r = kerteriz_bringup::ecef_to_enu_rotation(orijin());
  const auto birim = Eigen::Matrix<Scalar, 3, 3>::Identity();
  EXPECT_LT((r * r.transpose() - birim).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_NEAR(r.determinant(), 1.0, 1e-12);
}

} // namespace
