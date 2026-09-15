/// \file
/// F1.3 — klon yolunun RELEASE'de de fail-fast oldugunu dogrular.
///
/// Bu hedef KASTEN -DNDEBUG ile derlenir ve -UNDEBUG ALMAZ. Amaci tam olarak
/// assert'lerin KAPALI oldugu durumu sinamaktir: `assert(false)` NDEBUG altinda
/// kalkar, ardindan gelen `return *current_` calisir ve guncel durum istenen
/// KLON gibi sunulurdu. Diger backend testleri assert'ler acikken kostugu icin
/// bu senaryoyu goremez.

#include "kerteriz/backends/eskf_backend.hpp"

#include <gtest/gtest.h>

namespace {

using kerteriz::ArrayView;
using kerteriz::CloneId;
using kerteriz::EskfBackend;
using kerteriz::EskfConfig;
using kerteriz::ImuNoiseParams;
using kerteriz::kCoreDof;
using kerteriz::Measurement;
using kerteriz::MeasurementWorkspace;
using kerteriz::NavCovariance;
using kerteriz::NavState;
using kerteriz::Scalar;
using kerteriz::StateBundle;
using kerteriz::TimeNs;

// Assert'lerin gercekten KAPALI oldugunu derleme zamaninda kanitla; aksi halde
// asagidaki testler yanlis sebeple gecerdi.
#ifndef NDEBUG
#error "bu hedef NDEBUG ile derlenmelidir; aksi halde Release davranisi sinanmis olmaz"
#endif

class KlonIsteyen : public Measurement {
 public:
  TimeNs stamp_ns() const override { return 1; }
  int residual_dim() const override { return 1; }
  std::string_view name() const override { return "klon_isteyen"; }
  ArrayView<const CloneId> required_clones() const override {
    return ArrayView<const CloneId>(&istenen_, 1);
  }
  void evaluate(const StateBundle&, MeasurementWorkspace& w) const override {
    w.dim = 1;
    w.r[0] = Scalar(1);
    w.J_res(0, 9) = Scalar(-1);
    w.R(0, 0) = Scalar(1e-4);
  }

 private:
  CloneId istenen_{0};
};

EskfConfig config() {
  NavCovariance p = NavCovariance::Zero();
  p.topLeftCorner(kCoreDof, kCoreDof) =
      Eigen::MatrixXd::Identity(kCoreDof, kCoreDof).cast<Scalar>() * 0.1;
  return EskfConfig{NavState{}, p, 1000, ImuNoiseParams{1e-3, 1e-5, 2e-2, 3e-4}, 0.997};
}

TEST(CloneFailFastDeathTest, StateBundleCloneTerminatesWithAssertsDisabled) {
  EXPECT_DEATH(
      {
        const NavState x;
        const StateBundle bundle(x);
        // Donus degeri KULLANILIR; sessizce current_ dondurulseydi burada
        // gecerli bir NavState elde edilir ve test DUSERDI.
        const NavState& sahte = bundle.clone(CloneId{0});
        if (sahte.active_dof() == kCoreDof) {
          std::exit(0); // sessiz fallback: olum testi basarisiz olur
        }
      },
      "");
}

TEST(CloneFailFastDeathTest, UpdateRejectsMeasurementRequiringClonesWithAssertsDisabled) {
  EXPECT_DEATH(
      {
        EskfBackend backend(config());
        const KlonIsteyen z;
        backend.update(z);
        std::exit(0); // buraya ulasilirsa sessizce kabul edilmis demektir
      },
      "");
}

} // namespace
