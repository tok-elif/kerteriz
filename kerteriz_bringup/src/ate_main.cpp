/// \file
/// Iki yorunge CSV'si arasinda oteleme ATE'si — F1.8 Checkpoint B.
///
/// Hem Kerteriz hem de HARICI taban cizgisi ciktisi ayni araca ve ayni
/// referansa verilir; olcut tarafinda asimetri olmasin diye.

#include "kerteriz_bringup/ate.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using kerteriz_bringup::TrajectorySample;

/// "timestamp_ns,px,py,pz[,...]" — basliksatiri atlanir.
bool csv_oku(const std::string& yol, std::vector<TrajectorySample>& out) {
  std::ifstream f(yol);
  if (!f) {
    return false;
  }
  out.clear();
  std::string satir;
  bool ilk = true;
  while (std::getline(f, satir)) {
    if (satir.empty()) {
      continue;
    }
    if (ilk) {
      ilk = false;
      if (satir.rfind("timestamp_ns", 0) == 0) {
        continue;
      }
    }
    std::istringstream akis(satir);
    std::string alan;
    std::vector<std::string> parcalar;
    while (std::getline(akis, alan, ',')) {
      parcalar.push_back(alan);
    }
    if (parcalar.size() < 4) {
      std::fprintf(stderr, "bozuk satir: %s\n", satir.c_str());
      return false;
    }
    TrajectorySample s;
    s.stamp_ns = std::atoll(parcalar[0].c_str());
    s.position_w = kerteriz::Vec3(std::atof(parcalar[1].c_str()), std::atof(parcalar[2].c_str()),
                                  std::atof(parcalar[3].c_str()));
    out.push_back(s);
  }
  std::stable_sort(
      out.begin(), out.end(),
      [](const TrajectorySample& a, const TrajectorySample& b) { return a.stamp_ns < b.stamp_ns; });
  return true;
}

} // namespace

int main(int argc, char** argv) {
  std::string ref_yolu;
  std::string est_yolu;
  kerteriz::TimeNs max_dt = 20000000; // 20 ms
  std::string etiket = "kestirim";

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const auto sonraki = [&]() -> std::string {
      return (i + 1 < argc) ? argv[++i] : std::string();
    };
    if (a == "--reference") {
      ref_yolu = sonraki();
    } else if (a == "--estimate") {
      est_yolu = sonraki();
    } else if (a == "--label") {
      etiket = sonraki();
    } else if (a == "--max-dt-ms") {
      max_dt = static_cast<kerteriz::TimeNs>(std::atoll(sonraki().c_str())) * 1000000;
    } else {
      std::fprintf(stderr, "bilinmeyen secenek: %s\n", a.c_str());
      return 2;
    }
  }
  if (ref_yolu.empty() || est_yolu.empty()) {
    std::fprintf(stderr, "kullanim: kerteriz_ate --reference <csv> --estimate <csv> "
                         "[--label ad] [--max-dt-ms N]\n");
    return 2;
  }

  std::vector<TrajectorySample> ref;
  std::vector<TrajectorySample> est;
  if (!csv_oku(ref_yolu, ref) || !csv_oku(est_yolu, est)) {
    std::fprintf(stderr, "CSV okunamadi\n");
    return 1;
  }

  const auto r = kerteriz_bringup::translational_ate(ref, est, max_dt);
  std::printf("%-22s referans %zu  kestirim %zu  ", etiket.c_str(), ref.size(), est.size());
  if (!r.ok) {
    std::printf("ESLESME YOK\n");
    return 1;
  }
  std::printf("RMSE %.6f m  maks %.6f m  eslesen %d  eslesmeyen %d\n", r.rmse_m, r.max_error_m,
              r.matched, r.unmatched);
  return 0;
}
