# Changelog

Biçim: [Keep a Changelog](https://keepachangelog.com/), sürümleme: [SemVer](https://semver.org/).

## [Unreleased]

Faz 1 — ESKF. Kestirimci uçtan uca çalışır durumda: sentetik veride doğrulanmış,
gerçek KITTI üzerinde koşturulmuş ve harici bir taban çizgisiyle karşılaştırılmıştır.
Sürüm etiketi bu girdiyle **oluşturulmamıştır**.

### Added
- Faz 1 · F1.1: `NavState` — 15-DoF sabit çekirdek, sınırlı kapasiteli augmentation,
  klon değerleri, sağ pertürbasyon `plus`/`minus` (INTERFACES §1)
- Faz 1 · F1.2: `ImuPropagator` — birinci mertebe strapdown (rotasyon tam), ayrık haritanın
  tam Jacobian'ı `F`, `Q = G Qc Gᵀ dt` (INTERFACES §2)
- Faz 1 · F1.3: `EskfBackend` — Joseph formu, simetrizasyon, LDLT, χ² kapılama, klon
  kovaryansı; `chi_square.hpp` ters χ² niceliği
- Faz 1 · F1.4: `GnssPosition` — anten kolu destekli, analitik/sayısal Jacobian eşleşmesi
- Faz 1 · F1.5: sentetik ESKF yakınsama smoke testi (`kerteriz_sim`) — determinizm ve
  NIS özeti dahil; **R6 gereği gerçek veriden önce**
- Faz 1 · F1.6: `GnssVelocity`, `WheelVelocity`, `NonHolonomic`, `ZeroVelocity`;
  Denetim ④ artık beş **gerçek** Faz 1 ölçümünü kapsıyor
- Faz 1 · F1.7: `MeasurementBuffer` (sırasız ölçüm, geri sarma, ölçüm damgasında tam
  yerleştirme) + `Estimator` (tam `PipelineSnapshot` sahibi) + `RejectReason` eşlemesi;
  `NisMonitor` / `FaultDetector` **pasif** snapshot iskeleti (Faz 4)
- Faz 1 · F1.8 (A): KITTI raw OXTS ve NCLT adaptörleri, kanonik `DatasetEvent` sözleşmesi,
  WGS84 LLH → ECEF → yerel ENU dönüşümü, öteleme ATE aracı, sentetik biçim fixture'ları
- Faz 1 · F1.8 (B): gerçek KITTI çevrimdışı koşucusu (`kerteriz_kitti_runner`), ATE CLI
  (`kerteriz_ate`), **harici** `robot_localization` taban çizgisi yolu (yayıncı, kaydedici,
  launch, yapılandırma)
- Faz 1 · F1.8 (C): Faz 1 kapanış dokümantasyonu ve gerçek KITTI iş akışı

### Changed
- ADR-24: `UpdateResult` sayısal başarısızlığı temsil eder — `UpdateStatus` üç durumlu,
  `nis` opsiyonel; `RejectReason` `kNumericalFailure` ile genişler (INTERFACES §4, §5)
- Faz 1 tamamlanma ölçütü **açıkça değiştirildi**: özgün *"KITTI'de `robot_localization`
  ile denk ATE"* ölçütü sağlanmadı (ölçülen oran 4.01). Yerine *"tekrarlanabilir ATE
  karşılaştırması kurulmuş ve ham sonuçlar raporlanmış"* kondu; performans denkliği,
  ayar, çoklu dizi ve daha güçlü değerlendirme Faz 2'ye devredildi. Gerekçe ve ölçülen
  sayılar `docs/SPEC.md` §9.1'de.
- `CLAUDE.md` aktif faz Faz 2'ye geçti; README artık uygulanmış olanı anlatıyor
  ("filtre henüz yoktur" ifadesi kaldırıldı)

### Fixed
- `ImuPropagator` rotasyonu ortonormal kalmıyordu; kuaterniyon yayılımdan sonra
  normalleştiriliyor (assert açık yolda `SE_2_3` doğrulaması düşüyordu)
- `.gitignore` içindeki `data/` kuralı KITTI'nin `oxts/data/` düzenini yutuyor ve sentetik
  fixture'ı sessizce commit dışı bırakıyordu

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
