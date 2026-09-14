/// \file
/// S8 — CONVENTIONS §4 guncellemesi ve DENETIM (3).
///
/// ClosedFormScalarGaussian bu projenin en onemli testidir (PHASE0.md S8):
/// J_res yerine -J_res yazilirsa sayisal Jacobian testi GECMEYE DEVAM EDER,
/// bu test DUSER.

#include "kerteriz/util/linear_update.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <gtest/gtest.h>
#include <random>

namespace {

using kerteriz::innovation_nis;
using kerteriz::JacMat;
using kerteriz::kMaxResidualDim;
using kerteriz::linear_update;
using kerteriz::LinearUpdateResult;
using kerteriz::ResMat;
using kerteriz::ResVec;
using kerteriz::Scalar;
using kerteriz::StateMat;
using kerteriz::StateVec;

// --- DENETIM (3): kapali formlu lineer-Gauss testi --------------------------

TEST(LinearUpdate, ClosedFormScalarGaussian) {
  // Skaler problem: dof=1, dim=1, h(x)=x  =>  r = z - x,  J_res = -1.
  //   d  = (z - x) * P / (P + R)     <- ISARET POZITIF
  //   P+ = P * R / (P + R)
  const Scalar p0 = 2.0;
  const Scalar rr = 0.5;
  const Scalar x = 1.0;
  const Scalar z = 4.0;
  const Scalar artik = z - x; // +3

  JacMat j = JacMat::Zero();
  j(0, 0) = -1.0; // J_res = -1

  ResVec r = ResVec::Zero();
  r(0) = artik;

  ResMat r_cov = ResMat::Zero();
  r_cov(0, 0) = rr;

  StateMat p = StateMat::Zero();
  p(0, 0) = p0;

  StateVec d = StateVec::Zero();

  const auto sonuc = linear_update(j, r, r_cov, 1, 1, p, d);

  ASSERT_TRUE(sonuc.spd_ok);

  const Scalar d_beklenen = artik * p0 / (p0 + rr);
  const Scalar p_beklenen = p0 * rr / (p0 + rr);
  const Scalar nis_beklenen = artik * artik / (p0 + rr);

  EXPECT_NEAR(d(0), d_beklenen, 1e-12);
  EXPECT_NEAR(p(0, 0), p_beklenen, 1e-12);
  EXPECT_NEAR(sonuc.nis, nis_beklenen, 1e-12);

  // Anlami: olcum durumdan BUYUKSE duzeltme POZITIF olmalidir.
  EXPECT_GT(d(0), 0.0) << "olcum durumdan buyuk ama duzeltme negatif — isaret ters";
  EXPECT_LT(p(0, 0), p0) << "guncelleme belirsizligi azaltmadi";
}

TEST(LinearUpdate, SignFlipsWhenJacobianSignFlips) {
  // Denetim (3)'un varlik sebebi: -J_res verilirse duzeltmenin isareti doner.
  // Sayisal Jacobian testi bu hatayi GOREMEZ; bu test gorur.
  const Scalar p0 = 2.0;
  const Scalar rr = 0.5;
  const Scalar artik = 3.0;

  ResVec r = ResVec::Zero();
  r(0) = artik;
  ResMat r_cov = ResMat::Zero();
  r_cov(0, 0) = rr;

  auto calistir = [&](Scalar j00) {
    JacMat j = JacMat::Zero();
    j(0, 0) = j00;
    StateMat p = StateMat::Zero();
    p(0, 0) = p0;
    StateVec d = StateVec::Zero();
    const auto s = linear_update(j, r, r_cov, 1, 1, p, d);
    EXPECT_TRUE(s.spd_ok);
    return std::make_pair(d(0), p(0, 0));
  };

  const auto dogru = calistir(-1.0); // J_res konvansiyonu
  const auto ters = calistir(+1.0);  // yanlis isaret

  EXPECT_GT(dogru.first, 0.0);
  EXPECT_LT(ters.first, 0.0);
  EXPECT_NEAR(dogru.first, -ters.first, 1e-12);

  // Kovaryans her iki durumda da ayni — isaret hatasi P'de GORUNMEZ.
  // Hatanin yalnizca d'de gorunmesi, denetim (3)'un neden d'ye baktigini aciklar.
  EXPECT_NEAR(dogru.second, ters.second, 1e-12);
}

// --- Cok boyutlu davranis ---------------------------------------------------

/// Deterministik, simetrik pozitif tanimli matris.
StateMat spd_kovaryans(int dof, unsigned tohum) {
  std::mt19937 rng(tohum);
  std::uniform_real_distribution<Scalar> u(-1.0, 1.0);
  Eigen::MatrixXd m(dof, dof);
  for (int i = 0; i < dof; ++i) {
    for (int k = 0; k < dof; ++k) {
      m(i, k) = u(rng);
    }
  }
  StateMat p = StateMat::Zero();
  p.topLeftCorner(dof, dof) = m * m.transpose() + Eigen::MatrixXd::Identity(dof, dof) * 0.5;
  return p;
}

JacMat rastgele_jacobian(int dim, int dof, unsigned tohum) {
  std::mt19937 rng(tohum);
  std::uniform_real_distribution<Scalar> u(-1.5, 1.5);
  JacMat j = JacMat::Zero();
  for (int i = 0; i < dim; ++i) {
    for (int k = 0; k < dof; ++k) {
      j(i, k) = u(rng);
    }
  }
  return j;
}

TEST(LinearUpdate, MultiDimensionalCovarianceStaysSymmetricAndPositiveDefinite) {
  const int dof = 12;
  const int dim = 4;

  StateMat p = spd_kovaryans(dof, 7U);
  const JacMat j = rastgele_jacobian(dim, dof, 11U);

  ResMat r_cov = ResMat::Zero();
  r_cov.topLeftCorner(dim, dim) = Eigen::MatrixXd::Identity(dim, dim) * 0.25;

  ResVec r = ResVec::Zero();
  r.head(dim) << 0.3, -0.8, 1.2, 0.05;

  StateVec d = StateVec::Zero();
  const auto sonuc = linear_update(j, r, r_cov, dim, dof, p, d);

  ASSERT_TRUE(sonuc.spd_ok);

  const Eigen::MatrixXd pa = p.topLeftCorner(dof, dof);

  // Simetri — makine hassasiyetinde.
  EXPECT_LT((pa - pa.transpose()).cwiseAbs().maxCoeff(), 1e-14);

  // Pozitif tanimlilik.
  const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(pa);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0) << "P+ pozitif tanimli degil";

  EXPECT_GT(sonuc.nis, 0.0);
}

TEST(LinearUpdate, NisMatchesChiSquareMean) {
  // NIS'in beklenen degeri serbestlik derecesidir (= dim), artik N(0, S)'den
  // geldiginde. Tutarlilik iddiasinin en temel sayisal kontrolu budur.
  const int dof = 8;
  const int dim = 3;
  const int kosu = 4000;

  const StateMat p0 = spd_kovaryans(dof, 3U);
  const JacMat j = rastgele_jacobian(dim, dof, 5U);

  ResMat r_cov = ResMat::Zero();
  r_cov.topLeftCorner(dim, dim) = Eigen::MatrixXd::Identity(dim, dim) * 0.4;

  // S = J P J^T + R
  const Eigen::MatrixXd s = j.topLeftCorner(dim, dof) * p0.topLeftCorner(dof, dof) *
                                j.topLeftCorner(dim, dof).transpose() +
                            r_cov.topLeftCorner(dim, dim);
  const Eigen::MatrixXd l = Eigen::LLT<Eigen::MatrixXd>(s).matrixL();

  std::mt19937 rng(2024U);
  std::normal_distribution<Scalar> n(0.0, 1.0);

  Scalar toplam = 0.0;
  for (int i = 0; i < kosu; ++i) {
    Eigen::VectorXd w(dim);
    for (int k = 0; k < dim; ++k) {
      w(k) = n(rng);
    }
    ResVec r = ResVec::Zero();
    r.head(dim) = l * w; // r ~ N(0, S)

    StateMat p = p0;
    StateVec d = StateVec::Zero();
    const auto sonuc = linear_update(j, r, r_cov, dim, dof, p, d);
    ASSERT_TRUE(sonuc.spd_ok);
    toplam += sonuc.nis;
  }

  const Scalar ortalama = toplam / static_cast<Scalar>(kosu);
  // chi-kare(dim) ortalamasi = dim, varyansi = 2*dim.
  // 4000 kosuda standart hata = sqrt(2*dim/4000) ~ 0.039; 4 sigma band.
  const Scalar bant = 4.0 * std::sqrt(2.0 * dim / static_cast<Scalar>(kosu));
  EXPECT_NEAR(ortalama, static_cast<Scalar>(dim), bant)
      << "NIS ortalamasi " << ortalama << ", beklenen " << dim;
}

// --- Joseph vs naif: simetri sapmasi ----------------------------------------

TEST(LinearUpdate, JosephKeepsSymmetryWhereNaiveFormDrifts) {
  const int dof = 6;
  const int dim = 2;
  const int adim = 10000;

  const JacMat j = rastgele_jacobian(dim, dof, 13U);
  ResMat r_cov = ResMat::Zero();
  r_cov.topLeftCorner(dim, dim) = Eigen::MatrixXd::Identity(dim, dim) * 0.3;
  ResVec r = ResVec::Zero();
  r.head(dim) << 0.2, -0.1;

  // Joseph + simetrizasyon (uretim yolu).
  StateMat p_joseph = spd_kovaryans(dof, 17U);
  // Naif yol:  P+ = A P,  simetrizasyon YOK.
  Eigen::MatrixXd p_naif = p_joseph.topLeftCorner(dof, dof);

  StateVec d = StateVec::Zero();

  for (int i = 0; i < adim; ++i) {
    const auto sonuc = linear_update(j, r, r_cov, dim, dof, p_joseph, d);
    ASSERT_TRUE(sonuc.spd_ok) << "adim " << i << ": S pozitif tanimli cozulemedi";

    const Eigen::MatrixXd jj = j.topLeftCorner(dim, dof);
    const Eigen::MatrixXd s = jj * p_naif * jj.transpose() + r_cov.topLeftCorner(dim, dim);
    const Eigen::MatrixXd k =
        p_naif * jj.transpose() * s.ldlt().solve(Eigen::MatrixXd::Identity(dim, dim));
    p_naif = (Eigen::MatrixXd::Identity(dof, dof) - k * jj) * p_naif;
  }

  const Eigen::MatrixXd pj = p_joseph.topLeftCorner(dof, dof);
  const Scalar sapma_joseph = (pj - pj.transpose()).cwiseAbs().maxCoeff();
  const Scalar sapma_naif = (p_naif - p_naif.transpose()).cwiseAbs().maxCoeff();

  // Simetrizasyon zorunlu oldugu icin uretim yolu makine hassasiyetinde kalir.
  EXPECT_LT(sapma_joseph, 1e-15) << "simetrizasyon calismiyor";
  // Naif yol simetriyi korumaz — kuralin varlik sebebi budur.
  EXPECT_GT(sapma_naif, sapma_joseph) << "naif form bu ornekte ayrismiyor, test bekci degil";
}

// --- Bozuk giris ------------------------------------------------------------

TEST(LinearUpdate, RejectsSingularInnovationCovariance) {
  // REGRESYON. Eigen 3.4'te LDLT::isPositive() pozitif YARI-tanimli matris icin
  // de true doner. J, P ve R'nin hepsi sifirken S tekildir; yalnizca
  // isPositive()'e guvenilseydi bu durum kabul edilir ve anlamsiz bir cozum
  // uretilirdi. Kesin pozitiflik (vectorD > 0) bunu reddeder.
  const int dof = 3;
  const int dim = 2;

  const JacMat j = JacMat::Zero();     // J = 0
  const ResMat r_cov = ResMat::Zero(); // R = 0
  StateMat p = StateMat::Zero();       // P = 0
  const StateMat p_once = p;

  ResVec r = ResVec::Zero();
  r.head(dim).setOnes(); // r = 1

  StateVec d = StateVec::Ones();
  const auto sonuc = linear_update(j, r, r_cov, dim, dof, p, d);

  EXPECT_FALSE(sonuc.spd_ok) << "tekil S kabul edildi — pozitif yari-tanimli gecti";
  EXPECT_EQ(d.cwiseAbs().maxCoeff(), 0.0) << "tekil S'de delta sifirlanmadi";
  EXPECT_EQ((p - p_once).cwiseAbs().maxCoeff(), 0.0) << "tekil S'de P kirletildi";
}

TEST(LinearUpdate, LeavesCovarianceUntouchedWhenInnovationNotPositiveDefinite) {
  const int dof = 3;
  const int dim = 2;

  StateMat p = StateMat::Zero();
  p.topLeftCorner(dof, dof) = Eigen::MatrixXd::Identity(dof, dof);
  const StateMat p_once = p;

  JacMat j = JacMat::Zero();
  ResMat r_cov = ResMat::Zero();
  r_cov.topLeftCorner(dim, dim) = -Eigen::MatrixXd::Identity(dim, dim); // gecersiz
  ResVec r = ResVec::Zero();
  r.head(dim) << 1.0, 1.0;

  StateVec d = StateVec::Ones();
  const auto sonuc = linear_update(j, r, r_cov, dim, dof, p, d);

  EXPECT_FALSE(sonuc.spd_ok);
  EXPECT_EQ(d.cwiseAbs().maxCoeff(), 0.0) << "bozuk cozumde delta sifirlanmadi";
  EXPECT_EQ((p - p_once).cwiseAbs().maxCoeff(), 0.0) << "bozuk cozumde P kirletildi";
}

// -----------------------------------------------------------------------------
// F1.3 — NIS-only yardimci
//
// chi-kare kapisi MUTASYONDAN ONCE karar vermek zorunda; linear_update() ise
// NIS ile birlikte P'yi de gunceller. innovation_nis() ayni S'i kurar, ayni
// SPD olcutunu kullanir ve P'ye DOKUNMAZ.
// -----------------------------------------------------------------------------

TEST(InnovationNis, MatchesLinearUpdateNisForSameInputs) {
  std::mt19937 rng(4242U);
  std::normal_distribution<Scalar> n(0.0, 1.0);
  for (int deneme = 0; deneme < 200; ++deneme) {
    const int dim = 1 + (deneme % kMaxResidualDim);
    const int dof = 15;

    JacMat j = JacMat::Zero();
    ResVec r = ResVec::Zero();
    ResMat rc = ResMat::Zero();
    StateMat p = StateMat::Zero();

    for (int i = 0; i < dim; ++i) {
      r[i] = n(rng);
      for (int k = 0; k < dof; ++k) {
        j(i, k) = n(rng);
      }
    }
    // Simetrik pozitif tanimli P ve R.
    Eigen::MatrixXd a = Eigen::MatrixXd::Zero(dof, dof);
    for (int i = 0; i < dof; ++i) {
      for (int k = 0; k < dof; ++k) {
        a(i, k) = n(rng);
      }
    }
    p.topLeftCorner(dof, dof) = a * a.transpose() + Eigen::MatrixXd::Identity(dof, dof) * 0.5;
    rc.topLeftCorner(dim, dim) = Eigen::MatrixXd::Identity(dim, dim) * (0.2 + 0.1 * dim);

    const StateMat p_once = p;
    const LinearUpdateResult sadece_nis = innovation_nis(j, r, rc, dim, dof, p);

    EXPECT_LT((p - p_once).cwiseAbs().maxCoeff(), 0.0 + 1e-18) << "innovation_nis P'yi degistirdi";

    StateVec delta = StateVec::Zero();
    const LinearUpdateResult tam = linear_update(j, r, rc, dim, dof, p, delta);

    ASSERT_TRUE(sadece_nis.spd_ok);
    ASSERT_EQ(sadece_nis.spd_ok, tam.spd_ok);
    EXPECT_NEAR(sadece_nis.nis, tam.nis, 1e-12 * std::max(Scalar(1), std::abs(tam.nis)))
        << "dim=" << dim << " deneme=" << deneme;
  }
}

TEST(InnovationNis, UsesSameSpdCriterionAsLinearUpdate) {
  // Tekil S: J = 0, P = 0, R = 0, r != 0. linear_update bunu reddediyor;
  // innovation_nis de reddetmeli, aksi halde kapi cozulemeyen bir S'e NIS
  // uretmis olurdu.
  const JacMat j = JacMat::Zero();
  ResVec r = ResVec::Zero();
  r[0] = 1.0;
  const ResMat rc = ResMat::Zero();
  StateMat p = StateMat::Zero();

  const LinearUpdateResult sadece_nis = innovation_nis(j, r, rc, 1, 15, p);
  EXPECT_FALSE(sadece_nis.spd_ok) << "tekil S kabul edildi";

  StateVec delta = StateVec::Zero();
  const LinearUpdateResult tam = linear_update(j, r, rc, 1, 15, p, delta);
  EXPECT_EQ(sadece_nis.spd_ok, tam.spd_ok) << "iki yol farkli SPD karari verdi";
}

TEST(InnovationNis, ScalarGaussianClosedForm) {
  // Tek boyut: S = P + R, NIS = r^2 / (P + R).
  const Scalar p_deger = 2.0;
  const Scalar r_gurultu = 0.5;
  const Scalar artik = 1.3;

  JacMat j = JacMat::Zero();
  j(0, 0) = -1.0; // J_res = -H
  ResVec r = ResVec::Zero();
  r[0] = artik;
  ResMat rc = ResMat::Zero();
  rc(0, 0) = r_gurultu;
  StateMat p = StateMat::Zero();
  p(0, 0) = p_deger;

  const LinearUpdateResult out = innovation_nis(j, r, rc, 1, 1, p);
  ASSERT_TRUE(out.spd_ok);
  EXPECT_NEAR(out.nis, artik * artik / (p_deger + r_gurultu), 1e-12);
}

} // namespace
