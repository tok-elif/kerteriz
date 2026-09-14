# Changelog

Biçim: [Keep a Changelog](https://keepachangelog.com/), sürümleme: [SemVer](https://semver.org/).

## [Unreleased]

### Changed
- ADR-24: `UpdateResult` sayısal başarısızlığı temsil eder — `UpdateStatus` üç durumlu,
  `nis` opsiyonel; `RejectReason` `kNumericalFailure` ile genişler (INTERFACES §4, §5)

## [0.1.0] - 2026-09-13

Faz 0 — temel ve iskelet. Filtre henüz yoktur; bu sürüm, mimarisi dondurulmuş
ve beş denetimi CI'da zorlanan bir iskeleti işaretler.

### Added
- Mimari dondurma dokümantasyonu (ADR-1…23)
- Faz 0 · S0: repo iskeleti, lisans, lint ve pre-commit yapılandırması
- Faz 0 · S1: iki yollu CMake yapısı (`kerteriz_core` ROS'suz bağımsız derlenir)
- Faz 0 · S2: Eigen 3.4, manif 0.0.5, GoogleTest 1.14.0 bağımlılıkları
- Faz 0 · S3: Docker imajı (`ros:humble-ros-base`) ve devcontainer
- Faz 0 · S4: GitHub Actions matrisi (Humble + Jazzy), `core-standalone`, lint; **Denetim ①**
- Faz 0 · S5: temel tipler (`types.hpp`), `ArrayView`, sabit kapasiteli tipler; **Denetim ②**
- Faz 0 · S6: manif sarmalayıcı (`state/lie.hpp`), SE₂(3) exp/log/adjoint ve konvansiyon testleri
- Faz 0 · S7: sayısal residual Jacobian (`util/numeric_residual_jacobian.hpp`)
- Faz 0 · S8: `linear_update` — Joseph formu, simetrizasyon, LDLT; **Denetim ③** (kapalı formlu lineer-Gauss testi)
- Faz 0 · S9: minimal deterministik yörünge/sensör üreteci (`kerteriz_sim`), tohumlu ve tekrarlanabilir
- Faz 0 · S10: ölçüm registry'si ve Jacobian testi zorunluluğu; **Denetim ④**
- Faz 0 · S11: `tools/check_docs.py` — doküman ve kod tutarlılığı; **Denetim ⑤**
- Faz 0 · S12: ADR-1 ve ADR-2 ayrı dosyalara; `tools/check_adr.py` ile bölünme bütünlüğü

### Fixed
- `colcon` yolunda `kerteriz_core` paketinin dışa aktardığı hedef çözülemiyordu (`find_dependency(manif)` eksikti)
- `kerteriz_eval` testleri hiç koşmadan yeşil görünüyordu; `colcon`'un gerçekten çalıştırdığı biçime çevrildi
- `BUILD_TESTING` cache sızıntısı test hedeflerini sessizce kapatabiliyordu
- Kurulu pakette hedef adı `kerteriz::core` yerine `kerteriz::kerteriz_core` olarak dışa aktarılıyordu
