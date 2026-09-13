# Arayüz sözleşmesi

> `kerteriz_core`'un **sözleşmesidir**. Ajanlar buradaki imzalara göre uygular.
> İmza değiştirmek gerekiyorsa önce bu dosyayı güncelle, sonra kodu.
>
> **Durum: dondurulmuş (architecture freeze).** Değişiklik yeni ADR gerektirir.
>
> Ön koşul: `CONVENTIONS.md` §3 (`J_res`), §4 (Joseph), §5 (durum), §6 (zaman), §8.1 (tahsis).
> C++ standardı **C++17**'dir; `std::span` gibi C++20 tipleri kullanılmaz, `ArrayView` kullanılır. <!-- denetim5:muaf-satir std::span — C++17 kurali; yasagi anlatan uyari; sembolu yazmadan kural ifade edilemez -->

---

## 0. Ortak tipler

<!-- denetim5:muaf std::span — C++17 kuralini anlatan kod yorumu; yasagi anlatan uyari; sembolu yazmadan kural ifade edilemez -->
```cpp
namespace kerteriz {

using Scalar = double;
using Vec3   = Eigen::Matrix<Scalar, 3, 1>;

/// Mutlak zaman. CONVENTIONS §6: timestamp asla double saniye değildir.
using TimeNs = std::int64_t;

/// Durum boyutu sınırları — CONVENTIONS §5.3
inline constexpr int kCoreDof        = 15;
inline constexpr int kMaxAugmentDof  = 48;
inline constexpr int kMaxStateDof    = kCoreDof + kMaxAugmentDof;
inline constexpr int kMaxResidualDim = 12;

/// Sabit kapasiteli, çalışma anında aktif boyutu değişen tipler.
using StateVec  = Eigen::Matrix<Scalar, kMaxStateDof, 1>;
using StateMat  = Eigen::Matrix<Scalar, kMaxStateDof, kMaxStateDof>;
using ResVec    = Eigen::Matrix<Scalar, kMaxResidualDim, 1>;
using ResMat    = Eigen::Matrix<Scalar, kMaxResidualDim, kMaxResidualDim>;
using JacMat    = Eigen::Matrix<Scalar, kMaxResidualDim, kMaxStateDof>;


/// C++17 uyumlu tahsissiz görünüm. std::span C++20 olduğu için kullanılmaz.
template <typename T>
class ArrayView {
 public:
  constexpr ArrayView() = default;
  constexpr ArrayView(const T* data, std::size_t size) : data_(data), size_(size) {}

  constexpr const T* data()  const { return data_; }
  constexpr std::size_t size() const { return size_; }
  constexpr bool empty() const { return size_ == 0; }
  constexpr const T& operator[](std::size_t i) const { return data_[i]; }
  constexpr const T* begin() const { return data_; }
  constexpr const T* end()   const { return data_ + size_; }

 private:
  const T*    data_ = nullptr;
  std::size_t size_ = 0;
};

/// Zayıf gözlemlenebilir yön teşhisi (ADR-18).
struct WeakDirection {
  std::string_view label;    ///< "yaw", "wheel_scale", ...
  Scalar           sigma;    ///< bu yöndeki standart sapma
};

struct ImuSample {
  TimeNs stamp_ns;
  Vec3   gyro;      // rad/s, ölçülen (bias dahil)
  Vec3   accel;     // m/s², ölçülen (bias dahil)
};

/// Klon tanıtıcısı. kInvalidClone: klon yok.
enum class CloneId : std::int32_t {};
inline constexpr CloneId kInvalidClone{-1};

/// Kestirimci seviyesi çalışma modu. SensorHealth (§6) ile KARIŞTIRILMAZ:
/// bu sistemin durumu, o tek bir sensörün durumu.
enum class EstimatorMode {
  kUninitialized,   ///< orijin veya başlangıç durumu henüz yok
  kInitializing,    ///< toplanıyor (statik hizalama, ilk fix bekleniyor)
  kNominal,         ///< tüm beklenen sensörler sağlıklı
  kDegraded,        ///< çalışıyor ama bir veya daha fazla yardım kaynağı yok
  kFaulted          ///< kestirim güvenilir değil
};

}  // namespace kerteriz
```
<!-- denetim5:muaf-son -->

---

## 1. Durum — `state/`

Sabit çekirdek + sınırlı kapasiteli augmentation. İki **ayrı yaşam döngüsü**, tek depolama
(ADR-14, CONVENTIONS §5.2).

```cpp
/// Augmentation blok türü — yaşam döngüsünü belirler.
enum class AugmentKind {
  kPersistentCalibration,  ///< yapılandırmada açılır, oturum boyunca kalır, marjinalleştirilmez
  kClone                   ///< çalışma anında push/pop, kullanıldıktan sonra marjinalleştirilir
};

class NavState {
 public:
  // --- çekirdek: derleme zamanı, her zaman var ---
  manif::SE_2_3d& extended_pose();          ///< R_WB, v_WB, p_WB
  Vec3&           gyro_bias();
  Vec3&           accel_bias();

  // --- augmentation: çalışma anında ---
  int  active_dof() const;                  ///< kCoreDof + aktif augmentation
  int  augment_dof() const;

  /// Yapılandırma sırasında çağrılır. Kapasite aşılırsa std::nullopt.
  std::optional<int> register_calibration(std::string_view name, int dof);
  int                calibration_offset(std::string_view name) const;

  /// Çalışma anında. Kapasite yoksa kInvalidClone.
  CloneId push_clone();
  void    drop_clone(CloneId);              ///< marjinalleştirir, boyutu küçültür
  int     clone_offset(CloneId) const;
  bool    has_clone(CloneId) const;

  /// StateVec her zaman kMaxStateDof kapasitesindedir. Yalnız head(active_dof()) geçerlidir;
  /// plus() kuyruğu yok sayar, minus() kuyruğu sıfırlar. Heap resize yoktur.
  NavState plus (const Eigen::Ref<const StateVec>& delta) const;
  StateVec minus(const NavState& other) const;
};

/// Kapasite sabit, aktif blok active_dof() × active_dof().
using NavCovariance = StateMat;
```

**`register_calibration` neden yeniden derleme gerektirmez.** Depolama `kMaxStateDof`
kapasitesinde sabittir; kayıt yalnızca *aktif* boyutu ve ofset tablosunu değiştirir.
`estimate_scale_online: true` böylece salt YAML değişikliğiyle çalışır (ADR-14).

---

## 2. Süreç modeli — `process/`

```cpp
struct ImuNoiseParams {
  Scalar gyro_noise_density;    // rad/s/√Hz
  Scalar gyro_random_walk;      // rad/s²/√Hz
  Scalar accel_noise_density;   // m/s²/√Hz
  Scalar accel_random_walk;     // m/s³/√Hz
};

class ImuPropagator {
 public:
  explicit ImuPropagator(ImuNoiseParams params, Vec3 gravity_W = {0, 0, -9.80665});

  /// dt: SANİYE cinsinden süre, iki TimeNs farkından hesaplanmış (CONVENTIONS §6).
  /// Mutlak zamanı backend tutar; propagator zaman durumu taşımaz.
  void propagate(NavState& x, NavCovariance& P, const ImuSample& u, Scalar dt,
                 StateMat* F = nullptr, StateMat* Q = nullptr) const;
};
```

---

## 3. Ölçüm modeli — `measurements/`

Genişleme noktası burasıdır. Yeni sensör = bu arayüzün yeni bir uygulaması.

```cpp
/// Ölçümün göreceği durumlar: güncel durum + talep ettiği klonlar (ADR-9, ADR-14).
class StateBundle {
 public:
  explicit StateBundle(const NavState& current);
  const NavState& current() const;
  /// Ölçümün required_clones() ile talep ettiği klon. Yoksa çağrı hatadır.
  const NavState& clone(CloneId) const;
};

/// Tahsissiz çalışma alanı (CONVENTIONS §8.1). Çağıran sahiptir, ölçüm doldurur.
struct MeasurementWorkspace {
  ResVec r;        ///< artık,   ilk dim satırı geçerli
  JacMat J_res;    ///< residual Jacobian, dim × state.active_dof() bloğu geçerli
  ResMat R;        ///< ölçüm gürültüsü, dim × dim bloğu geçerli
  int    dim = 0;  ///< bu ölçümün gerçek boyutu
};

class Measurement {
 public:
  virtual ~Measurement() = default;

  virtual TimeNs           stamp_ns()     const = 0;
  virtual int              residual_dim() const = 0;   ///< ≤ kMaxResidualDim
  /// YAML'daki sensör adı. Ömrü ölçümden uzundur (yapılandırmada sahiplenilir).
  virtual std::string_view name()         const = 0;

  /// Bu ölçümün ihtiyaç duyduğu klonlar. Boş ise mutlak ölçümdür.
  /// Backend bunları StateBundle'a koyar ve J_res'in ilgili sütun bloklarını bekler.
  virtual ArrayView<const CloneId> required_clones() const { return {}; }

  /// w.r = z ⊖ h(X),  w.J_res = ∂r(X ⊞ δ)/∂δ,  w.R = gürültü,  w.dim = residual_dim()
  ///
  /// J_res sütunları durumun TAM aktif düzenine göredir: çekirdek, kalibrasyon
  /// blokları ve talep edilen klon blokları kendi ofsetlerinde doldurulur; geri kalanı
  /// sıfır bırakılır. Ofsetler NavState::calibration_offset / clone_offset ile alınır.
  ///
  /// Tahsis yapmaz. Yalnızca w'ye yazar.
  virtual void evaluate(const StateBundle& states, MeasurementWorkspace& w) const = 0;
};
```

### Uygulanacak ölçümler

| Sınıf | Faz | Klon | Not |
|---|---|---|---|
| `GnssPosition` | 1 | — | ENU konum |
| `GnssVelocity` | 1 | — | Doppler |
| `WheelVelocity` | 1 | — | Ham enkoder → gövde hızı. Klon gerekmez (ADR-9). |
| `NonHolonomic` | 1 | — | v_y ≈ 0, v_z ≈ 0 gövde çerçevesinde |
| `ZeroVelocity` | 1 | — | ZUPT |
| `RelativePose` | 3 | **1 klon** | Entegre `/odom`. `required_clones()` başlangıç anını döner. |
| `Magnetometer` | 4 | — | opsiyonel |
| `Barometer` | 4 | — | opsiyonel |

---

## 4. Filtre backend'i — `backends/`

```cpp
/// Filtre güncellemesinin sonucu. YALNIZCA filtre seviyesi bilgisi taşır —
/// "bayat ölçüm" veya "FDI dışladı" gibi sebepler buraya ait DEĞİLDİR (§5, ADR-19).
struct UpdateResult {
  bool   accepted;    ///< χ² kapısını geçti mi
  Scalar nis;         ///< rᵀ S⁻¹ r
  int    dof;         ///< = residual_dim()
  Scalar threshold;   ///< kullanılan χ² eşiği
};

/// Backend'in KENDİ durumu. Bütünlük katmanını İÇERMEZ (ADR-20).
struct BackendSnapshot {
  /// NavState kopyası aktif augmentation düzenini, kalibrasyon ofsetlerini, klon kimliklerini
  /// ve klon değerlerini de taşır. Klon cross-covariance blokları covariance içindedir.
  NavState      state;
  NavCovariance covariance;
  TimeNs        stamp_ns;         ///< zaman snapshot'ın parçasıdır (ADR-15)
};

class FilterBackend {
 public:
  virtual ~FilterBackend() = default;

  /// dt saniye; backend mutlak zamanı u.stamp_ns'ten alır.
  virtual void         predict(const ImuSample& u, Scalar dt) = 0;
  /// CONVENTIONS §4'teki Joseph + simetrizasyon uygulanır. Explicit inverse yasak.
  virtual UpdateResult update(const Measurement& z)           = 0;

  virtual const NavState&      state()      const = 0;
  virtual const NavCovariance& covariance() const = 0;
  virtual TimeNs               stamp_ns()   const = 0;
  virtual EstimatorMode        mode()       const = 0;

  /// Geri sarma sözleşmesi. reset(x, P) YOKTUR — zaman ve düzen kaybolduğu için
  /// sessiz semantik hata üretiyordu (ADR-15).
  /// Backend YALNIZCA kendi durumunu kaydeder; bütünlük katmanını bilmez (ADR-20).
  virtual BackendSnapshot save_snapshot() const                   = 0;
  virtual void            restore_snapshot(const BackendSnapshot&) = 0;

  virtual CloneId push_clone()          = 0;
  virtual void    drop_clone(CloneId)   = 0;

  /// Zayıf gözlemlenebilir alt uzaylar — teşhis için (ADR-18, Faz 4).
  virtual ArrayView<const WeakDirection> weak_directions() const { return {}; }
};
```

**Uygulamalar:** `EskfBackend` (Faz 1) · `InekfBackend` (Faz 3) · `IteratedEskfBackend` (ops.)

Üçü de aynı `Measurement` nesnelerini tüketir. Ölçüm sınıfları backend'i bilmez.

---

## 5. Ardışık düzen — `buffer/`

Red sebebi **ardışık düzen seviyesindedir**, backend'in değil (ADR-19).

```cpp
class Estimator;
class NisMonitor;
class FaultDetector;

/// Alt bileşen snapshot'ları somut, değer-semantikli ve sabit kapasiteli tiplerdir.
/// Aşağıdaki alanlar şematik gösterimdir; gerçek header'da fixed-capacity üyeler açıkça tanımlanır.
/// Heap sahipliği taşımazlar.
struct NisState   { /* fixed-capacity per-sensor NIS accumulators */ };
struct FaultState { /* fixed-capacity sensor health/exclusion state */ };

enum class RejectReason {
  kNone,
  kTooOld,            ///< tampon penceresinin dışında           → buffer kararı
  kOutOfOrderDrop,    ///< geri sarma mümkün değil                → buffer kararı
  kSensorDisabled,    ///< YAML'da kapalı                         → yapılandırma
  kSensorExcluded,    ///< FDI dışladı                            → integrity kararı
  kOriginNotSet,      ///< coğrafi ölçüm, orijin henüz yok        → yapılandırma
  kChiSquareGate,     ///< filtre kapısı elendi                   → backend kararı
  kCloneUnavailable   ///< gerekli klon yok veya düşürülmüş       → durum kararı
};

/// Bir ölçümün ardışık düzendeki tam sonucu.
struct ProcessingResult {
  std::string_view            sensor;
  TimeNs                      stamp_ns;
  RejectReason                reason;      ///< kNone ise kabul edildi
  std::optional<UpdateResult> update;      ///< yalnızca backend'e ulaştıysa dolu
};

class MeasurementBuffer {
 public:
  explicit MeasurementBuffer(TimeNs window_ns);

  void add_imu(const ImuSample& u);
  bool add_measurement(std::unique_ptr<Measurement> z);

  /// Zaman sırasına göre işler. Geciken ölçüm için Estimator'ün snapshot'ına döner.
  /// Dönen view bir sonraki process() çağrısına kadar geçerlidir; buffer içi sabit kapasiteli
  /// sonuç deposuna bakar, sahiplik taşımaz.
  ArrayView<const ProcessingResult> process(Estimator& est);
};
```

`MeasurementBuffer` ölçüm nesnelerinin sahipliğini `std::unique_ptr` ile alabilir. Bu tahsis,
ADR-22'de tanımlanan **sayısal hot path** (`predict/update/evaluate`) dışındadır. Hard-real-time
bir taşıma hedefinde bu sahiplik katmanı ayrıca object pool/fixed-capacity variant ile değiştirilebilir;
filtre matematiği ve ölçüm çalışma alanı tahsissiz kalır.

### 5.1 `Estimator` — orkestratör ve snapshot sahibi

Backend bütünlük katmanını bilmez, bütünlük katmanı backend'i bilmez. İkisini birleştiren
ve **tam snapshot'ın sahibi olan** katman budur (ADR-20).

```cpp
/// Replay'i etkileyen TÜM mutable state. Alt bileşenlerin snapshot'larının bileşimi.
/// KURAL (ADR-15): yeni stateful bileşen eklenirse buraya da eklenir — istisna yoktur.
struct PipelineSnapshot {
  BackendSnapshot backend;
  NisState        nis;      ///< NisMonitor::capture()
  FaultState      fault;    ///< FaultDetector::capture()
};

class Estimator {
 public:
  Estimator(std::unique_ptr<FilterBackend>, NisMonitor, FaultDetector);

  void             predict(const ImuSample& u, Scalar dt);
  /// Kapılama, bütünlük dışlaması ve filtre güncellemesini sırayla uygular.
  ProcessingResult apply(const Measurement& z);

  /// Tam snapshot — alt bileşenlerden toplanır, onlara dağıtılır.
  PipelineSnapshot save_snapshot() const;
  void             restore_snapshot(const PipelineSnapshot&);

  const FilterBackend& backend() const;
  EstimatorMode        mode()    const;
};
```

**Neden backend'de değil.** `NisMonitor` ve `FaultDetector` `integrity/` katmanındadır;
tam snapshot'ı `FilterBackend::save_snapshot()`'a koymak backend'in kendisine ait olmayan
durumu bilmesini gerektirirdi — ADR-19'un red sebebi için reddettiği katman karışmasının
aynısı. Her bileşen kendi durumunu kaydeder, `Estimator` birleştirir, `MeasurementBuffer`
`PipelineSnapshot` saklar.

---

## 6. Bütünlük — `integrity/` (Faz 4)

```cpp
/// TEK SENSÖRÜN sağlığı. EstimatorMode (§0) sistem seviyesidir — karıştırma.
enum class SensorHealth { kNominal, kSuspect, kFaulted };

class NisMonitor {
 public:
  void   record(std::string_view sensor, const UpdateResult& r);
  Scalar mean_nis(std::string_view sensor)        const;
  Scalar acceptance_rate(std::string_view sensor) const;
  /// CONVENTIONS §10: beklenen bant c ± 3√(c(1−c)/N)
  bool   acceptance_within_band(std::string_view sensor, Scalar chi2_confidence) const;

  /// PipelineSnapshot'a dahil (ADR-15, ADR-20). Bileşen kendi durumunu kaydeder.
  NisState capture() const;
  void     restore(const NisState&);
};

class FaultDetector {
 public:
  SensorHealth evaluate(std::string_view sensor, const NisMonitor&);
  bool         is_excluded(std::string_view sensor) const;

  FaultState capture() const;
  void       restore(const FaultState&);
};

struct ProtectionLevel { Scalar horizontal; Scalar vertical; Scalar integrity_risk; };

ProtectionLevel compute_protection_level(const NavCovariance& P, int active_dof,
                                         const FaultDetector& fd,
                                         Scalar target_integrity_risk);

```

---

## 7. Doğrulama araçları — `util/`

```cpp
/// Manifold üzerinde merkezi farkla sayısal residual Jacobian.
/// residual_fn, X ⊞ δ noktasındaki artığı döndürür → çıktı doğrudan J_res'tir.
template <typename Fn>
JacMat numeric_residual_jacobian(const NavState& x, Fn&& residual_fn,
                                 int residual_dim, Scalar eps = 1e-6);
```

```cpp
TEST(GnssPosition, JacobianMatchesNumeric) {
  const auto x = random_nav_state();
  GnssPosition z{/* ... */};
  MeasurementWorkspace w;
  StateBundle bundle{x};
  z.evaluate(bundle, w);

  const JacMat J_num = numeric_residual_jacobian(
      x, [&](const NavState& xp) {
           MeasurementWorkspace ww; StateBundle b{xp};
           z.evaluate(b, ww); return ww.r.head(ww.dim).eval();
         },
      z.residual_dim());

  EXPECT_TRUE(w.J_res.topLeftCorner(w.dim, x.active_dof())
              .isApprox(J_num.topLeftCorner(w.dim, x.active_dof()), 1e-5));
}
```

**Kapalı formlu işaret testi — zorunlu (CONVENTIONS §9).** Analitik çözümü bilinen skaler
lineer-Gauss problemde güncellemenin *işaretini ve değerini* doğrular. `J_res`
konvansiyonunu koruyan tek gerçek savunma budur; `J_res` yerine `−J_res` yazılırsa
sayısal Jacobian testi geçer ama bu test düşer.

---

## 8. ROS katmanı — `kerteriz_ros/`

İş mantığı yok. Sorumlulukları:

1. Parametre yükleme → core yapılandırma nesneleri (kalibrasyon bloğu kaydı dahil)
2. ROS mesajı → `Measurement` (fabrika, YAML'daki `type` anahtarına göre)
3. `MeasurementBuffer`'ı beslemek, `Estimator` orkestratörünü sürmek
4. `NavState` → `nav_msgs/Odometry`, TF2; `ProcessingResult` + `EstimatorMode` +
   `ProtectionLevel` → `kerteriz_msgs/*` teşhis
5. Lifecycle geçişleri

```cpp
class EstimatorNode : public rclcpp_lifecycle::LifecycleNode {
  CallbackReturn on_configure(const State&) override;   // YAML, kalibrasyon kaydı, core kurulumu
  CallbackReturn on_activate(const State&) override;
  CallbackReturn on_deactivate(const State&) override;
  CallbackReturn on_cleanup(const State&) override;
};
```

**Denetim:** `kerteriz_ros`'ta Eigen matematiği görüyorsan, o kod `kerteriz_core`'a aittir.
