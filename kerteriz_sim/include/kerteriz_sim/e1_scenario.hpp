#pragma once

/// \file
/// E1 — Faz 2 ESKF tutarlilik deneyinin DONDURULMUS senaryosu.
///
/// ======================== NEDEN AYRI DOSYA ==================================
///
/// Bu sayilar bir deneyin ONKOSULUDUR. Sonucu gordukten SONRA degistirilirse
/// deney kanit olmaktan cikar. Tek bir yerde, denetlenebilir bicimde dururlar;
/// manifest dosyasi da buradan uretilir.
///
/// ============================ KAPSAM ========================================
///
/// SPEC §8'deki nihai E1 IKI BACKEND'in kiyasidir. Bugun yalnizca ESKF vardir,
/// dolayisiyla buradan cikan sey FAZ 2 ESKF TABAN CIZGISIDIR — E1'in backend
/// kiyasi sorusu KAPANMAMISTIR. InEKF Faz 3'te AYNI deney sozlesmesinden
/// gecirilecektir.
///
/// ====================== BASLANGIC BELIRSIZLIGI ==============================
///
/// F1.5 duman senaryosu her realizasyonda AYNI sabit baslangic hatasini
/// kullanir. Topluluk tabanli NEES yorumu icin bu YANLISTIR: 500 kosu ayni
/// onyargiyi tasir ve ANEES kovaryansin dogrulugunu degil, tek bir sapmanin
/// suzulusunu olcer. E1 bu yuzden delta0 ~ N(0, P0) ornekler.
///
/// BUYUK YAW BELIRSIZLIGI. E1'in sordugu soru SPEC §8'de "buyuk yaw
/// belirsizliginde bant icinde kalinip kalinmadigi"dir. Depoda dondurulmus bir
/// deger YOKTU; burada 30 derece olarak dondurulur. Yuvarlanma/pitch 0.1 rad
/// olarak KALIR — stres yalnizca yaw uzerindedir, cunku linearizasyon kaynakli
/// tutarsizligin en gorunur oldugu yon odur.

#include "kerteriz_sim/experiment.hpp"
#include "kerteriz_sim/monte_carlo.hpp"

#include <cstdint>

namespace kerteriz_sim {

/// E1 dondurulmus sabitleri. Sonuc gorulduKTEN SONRA DEGISTIRILMEZ.
namespace e1 {

inline constexpr int kRuns = 500;
inline constexpr std::uint64_t kMasterSeed = 20260916ULL;
inline constexpr int kStateSampleStride = 20; ///< 100 Hz -> 5 Hz, GNSS ile hizali
inline constexpr Scalar kConsistencyConfidence = Scalar(0.95);

/// 30 derece. Teget sirasi [dtheta, dv, dp, db_g, db_a] ve dtheta'nin ucuncu
/// bileseni govde z eksenidir — yani yaw.
inline constexpr Scalar kYawSigmaRad = Scalar(0.52359877559829887308); // pi/6
inline constexpr int kYawTangentIndex = 2;

/// Yuvarlanma/pitch varsayilanda kalir.
inline constexpr Scalar kRollPitchSigmaRad = Scalar(0.1);

} // namespace e1

/// E1'in baslangic kovaryansi: varsayilan P0, yalnizca YAW varyansi buyutulmus.
inline NavCovariance e1_initial_covariance() {
  NavCovariance p = default_initial_covariance();
  p(e1::kYawTangentIndex, e1::kYawTangentIndex) = e1::kYawSigmaRad * e1::kYawSigmaRad;
  return p;
}

/// E1 senaryosu. Gurultu, yorunge, frekanslar ve kapi guveni `noisy_scenario`
/// ile AYNIDIR; yalnizca baslangic belirsizligi ve ornekleme politikasi farkli.
inline ScenarioConfig e1_eskf_scenario() {
  ScenarioConfig c = noisy_scenario();
  c.initial_covariance = e1_initial_covariance();
  c.initial_error_policy = ScenarioConfig::InitialErrorPolicy::kGaussianFromP0;
  // kGaussianFromP0 modunda `initial_error` KULLANILMAZ; sifirlanmasi
  // niyetin acik olmasi icindir.
  c.initial_error = StateVec::Zero();
  c.record_state_samples = true;
  c.state_sample_stride = e1::kStateSampleStride;
  return c;
}

inline MonteCarloConfig e1_eskf_monte_carlo() {
  MonteCarloConfig m;
  m.scenario = e1_eskf_scenario();
  m.runs = e1::kRuns;
  m.master_seed = e1::kMasterSeed;
  m.record_state_samples = true;
  m.state_sample_stride = e1::kStateSampleStride;
  return m;
}

} // namespace kerteriz_sim
