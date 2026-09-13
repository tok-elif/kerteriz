# Faz 0 — Uygulama Planı

> **Süre:** 2 hafta (10 iş günü) · **Ön koşul:** mimari dondurulmuş (ADR-13…22)
> **Bu faz kod yazma fazıdır.** Mimari tartışması kapandı; burada karar değil, uygulama var.

---

## 1. Faz 0 ne için var

Üç iş yapar, üçü de sonraki fazların ön koşuludur:

1. **Altyapı** — build, test, CI, Docker. Repo ilk günden profesyonel görünür.
2. **Matematik temeli** — manif sarmalayıcı, sayısal Jacobian aracı, ve **§4'teki güncelleme
   denklemi tek bir test edilmiş fonksiyonda**.
3. **Sentetik veri** — minimal deterministik üreteç. R6 ("önce simülasyon") Faz 1'de ancak
   bu varsa tutulabilir.

**Faz 0'da filtre yoktur.** ESKF, ölçüm modelleri, `Estimator`, tampon — hepsi Faz 1.

---

## 2. Uygulama sırası

Bağımlılık yönü: yukarıdan aşağı. Bir adım bitmeden sonrakine geçme.

```
S0 repo/git ──► S1 CMake ──► S2 bagimliliklar ──► S3 Docker ──► S4 CI + denetim①
                                   │
                                   ├──► S5 types.hpp + denetim②
                                   │        │
                                   │        ├──► S6 manif sarmalayici + SE_2(3) testleri
                                   │        │
                                   │        ├──► S7 numeric_residual_jacobian
                                   │        │        │
                                   │        │        └──► S8 linear_update + denetim③  ★
                                   │        │
                                   │        └──► S9 minimal uretec (kerteriz_sim)
                                   │
                                   ├──► S10 test registry + denetim④
                                   └──► S11 dokuman denetimi + denetim⑤
                                            │
                                            └──► S12 ADR-1/ADR-2 dosyalari
```

★ = fazın en kritik çıktısı.

---

## 3. Adımlar

### S0 · Repo ve hijyen — ½ gün

```
kerteriz/
├── .gitignore  .clang-format  .clang-tidy  .pre-commit-config.yaml
├── LICENSE (MIT)  README.md (taslak)  CHANGELOG.md
├── CLAUDE.md  docs/…                      ← mevcut, taşınacak
```

- `.clang-format`: LLVM tabanlı, `ColumnLimit: 100`
- `.clang-tidy`: `bugprone-*`, `performance-*`, `readability-*`, `cppcoreguidelines-*` (seçmeli)
- İlk commit: `chore: initial repo skeleton with frozen architecture docs`

**DoD:** repo public, `pre-commit run --all-files` geçiyor.

---

### S1 · CMake iskeleti — ½ gün

İki ayrı build yolu olmalı; bu R1'in *yapısal* garantisidir:

```
kerteriz_core/CMakeLists.txt      ← saf CMake. ament YOK. Tek başına derlenir.
kerteriz_ros/CMakeLists.txt       ← ament_cmake. kerteriz_core'u find_package ile bulur.
kerteriz_msgs/CMakeLists.txt      ← rosidl
kerteriz_sim/CMakeLists.txt       ← saf CMake (core'a bağlı, ROS'a değil)
kerteriz_eval/                    ← Python, setup.py
kerteriz_bringup/CMakeLists.txt   ← ament, yalnız launch/config
```

`kerteriz_core` dışa `kerteriz::core` hedefi export eder.

**DoD:** `cmake -S kerteriz_core -B build/core` **ROS kurulu olmayan bir kabukta** derleniyor;
`colcon build` altı paketi de derliyor.

---

### S2 · Bağımlılıklar — ½ gün

| Bağımlılık | Nasıl |
|---|---|
| Eigen 3.4 | `find_package(Eigen3 3.4 REQUIRED)` — 22.04'te sistemde var |
| manif | `FetchContent` + **sabit tag** (`GIT_TAG` ile pinle, `master` kullanma) |
| GoogleTest | `FetchContent`, pinli |

`CMAKE_CXX_STANDARD 17`, `CXX_STANDARD_REQUIRED ON`, `CXX_EXTENSIONS OFF`.

**DoD:** temiz bir konteynerde sıfırdan build geçiyor; manif sürümü kilitli.

---

### S3 · Docker ve devcontainer — ½ gün

```dockerfile
FROM ros:humble-ros-base
# build-essential, cmake, git, libeigen3-dev, python3-colcon-common-extensions,
# clang-format, clang-tidy, pre-commit
```

`.devcontainer/devcontainer.json` aynı imajı kullanır.

**DoD:** `docker build` + `docker run` içinde `colcon build && colcon test` geçiyor.

---

### S4 · CI matrisi + **Denetim ①** — ½ gün

`.github/workflows/ci.yml`:

```yaml
strategy:
  matrix:
    ros_distro: [humble, jazzy]
```

İşler: `build` · `test` · `lint` (clang-format + clang-tidy) · `core-standalone`
(ROS'suz derleme) · `checks` (denetimler).

**Denetim ①** burada devreye girer:

```bash
! grep -rE "rclcpp|_msgs/|ament_" kerteriz_core/ --include=*.hpp --include=*.cpp --include=CMakeLists.txt
```

**DoD:** iki distro'da da yeşil rozet.

---

### S5 · Temel tipler + **Denetim ②** — 1 gün

`kerteriz_core/include/kerteriz/types.hpp` — INTERFACES §0'ın birebir uygulaması:

- `Scalar`, `Vec3`, `TimeNs`
- `kCoreDof`, `kMaxAugmentDof`, `kMaxStateDof`, `kMaxResidualDim`
- `StateVec`, `StateMat`, `ResVec`, `ResMat`, `JacMat`
- `ArrayView<T>` (C++17, R8)
- `ImuSample`, `CloneId`, `kInvalidClone`, `EstimatorMode`, `WeakDirection`

**Denetim ②:**

```cpp
static_assert(kCoreDof + kMaxAugmentDof == kMaxStateDof, "state capacity mismatch");
static_assert(kMaxResidualDim > 0 && kMaxResidualDim <= kMaxStateDof);
```

**DoD:** `types.hpp` tek başına derleniyor; `ArrayView` için birim testleri (boş, tek eleman,
aralık tabanlı for) geçiyor.

---

### S6 · manif sarmalayıcı ve SE₂(3) testleri — 1½ gün

`kerteriz_core/include/kerteriz/state/lie.hpp` — ince sarmalayıcı; amacı konvansiyonu
tek yerde sabitlemek (sağ pertürbasyon, Hamilton).

Testler (`test/test_lie.cpp`):

| Test | İddia |
|---|---|
| `ExpLogRoundTrip` | `Log(Exp(τ)) ≈ τ` küçük ve orta τ için |
| `PlusMinusRoundTrip` | `X ⊞ (Y ⊟ X) ≈ Y` |
| `RightPerturbationConvention` | `X ⊞ δ == X ∘ Exp(δ)` — sol değil |
| `AdjointIdentity` | `Ad_X · τ` ile `X ∘ Exp(τ) ∘ X⁻¹` tutarlı |
| `QuaternionStorageOrder` | `Quaterniond(w,x,y,z).coeffs() == (x,y,z,w)` — tuzağı kayda geçirir |

**DoD:** beş test de geçiyor. `RightPerturbationConvention` ve `QuaternionStorageOrder`
konvansiyon kaymasına karşı kalıcı bekçidir.

---

### S7 · Sayısal residual Jacobian — 1 gün

`kerteriz_core/include/kerteriz/util/numeric_residual_jacobian.hpp` — INTERFACES §7.

Merkezi fark, manifold üzerinde: `x ⊞ (+εeᵢ)` ve `x ⊞ (−εeᵢ)`.

**DoD:** analitik türevi bilinen oyuncak bir residual fonksiyonunda (örn. `r(x) = A·Log(x)`)
sayısal Jacobian analitiğe `1e-5` içinde eşit.

---

### S8 · `linear_update` + **Denetim ③** — 1½ gün ★

**Bu fazın en önemli çıktısı.**

CONVENTIONS §4'teki güncelleme denklemi **tek bir yerde** uygulanır. Faz 1'deki ESKF ve
Faz 3'teki InEKF bu fonksiyonu çağırır; işaret ve Joseph mantığı iki kere yazılmaz.

`kerteriz_core/include/kerteriz/util/linear_update.hpp`:

```cpp
/// CONVENTIONS §4'ün birebir uygulaması. J_res konvansiyonu (ADR-13),
/// Joseph formu + simetrizasyon (ADR-17), explicit inverse yok.
/// Tahsis yapmaz (ADR-22): tüm ara matrisler sabit kapasiteli.
struct LinearUpdateResult { Scalar nis; bool spd_ok; };

LinearUpdateResult linear_update(
    const Eigen::Ref<const JacMat>& J_res,   // dim × dof
    const Eigen::Ref<const ResVec>& r,       // dim
    const Eigen::Ref<const ResMat>& R,       // dim × dim
    int dim, int dof,
    Eigen::Ref<StateMat> P,                  // giriş/çıkış
    Eigen::Ref<StateVec> delta_out);         // δ = −P J_resᵀ S⁻¹ r
```

İçeride sırasıyla: `S = J P Jᵀ + R` → `S.ldlt()` → `δ = −P Jᵀ S⁻¹ r` →
`A = I − P Jᵀ S⁻¹ J` → `P⁺ = A P Aᵀ + P Jᵀ S⁻¹ R S⁻¹ J P` → `P ← ½(P+Pᵀ)` →
`nis = rᵀ S⁻¹ r`.

> **Not:** Bu, dondurulmuş bir arayüzü değiştirmez. `FilterBackend::update()` imzası
> INTERFACES §4'teki hâliyle durur; `linear_update` yalnızca onun *içinde* kullanılacak
> bir `util/` yardımcısıdır. Faz 0'da backend olmadığı için denetim ③'ün burada
> uygulanabilmesinin tek yolu budur.

**Denetim ③ — kapalı formlu lineer-Gauss testi** (`test/test_linear_update.cpp`):

Skaler problem: `dof=1`, `dim=1`, `h(x)=x`, dolayısıyla `r = z − x` ve `J_res = −1`.
Analitik sonuç:

```
δ  = (z − x)·P/(P+R)          ← işaret POZİTİF olmalı: ölçüm büyükse durum büyür
P⁺ = P·R/(P+R)
```

Test bunu `1e-12` içinde doğrular. **`J_res` yerine `−J_res` yazılırsa `δ`'nın işareti
ters döner ve bu test düşer** — sayısal Jacobian testi ise geçmeye devam eder. Denetim ③'ün
varlık sebebi tam olarak budur.

Ek testler: çok boyutlu durumda `P⁺` simetrik ve pozitif tanımlı; `nis` beklenen χ²
ortalamasına yakın; Joseph ile naif formun 10⁴ adım sonrası simetri sapması karşılaştırması.

**DoD:** kapalı formlu test geçiyor ve CI'da zorunlu işaretli.

---

### S9 · Minimal deterministik üreteç — 1½ gün

`kerteriz_sim/` — Faz 2'deki Monte Carlo'nun tohumu, ama şimdilik **minimal**.

| Bileşen | Kapsam |
|---|---|
| `TrajectoryGenerator` | Analitik yörünge: sabit hız, daire, sekiz. Ground truth `R_WB, v_WB, p_WB` kapalı formda. |
| `ImuSynthesizer` | Ground truth'tan `ω, a` üretir; bias + beyaz gürültü + rastgele yürüyüş ekler |
| `GnssSynthesizer` | Konum + gürültü, yapılandırılabilir hız |
| `WheelSynthesizer` | Gövde hızı + ölçek hatası + gürültü |
| `SeededRng` | `std::mt19937_64`, açık tohum |

**Kapsam dışı (Faz 2):** Monte Carlo koşucusu, NEES analizi, arıza enjeksiyonu, evo.

**DoD:** aynı tohum → bit-bit aynı çıktı (test bunu doğrular); üretilen IMU'yu ground truth'a
entegre ettiğinde sapma gürültü modeliyle tutarlı.

---

### S10 · Test registry + **Denetim ④** — ½ gün

Ölçüm fabrikası ve test registry'si birlikte kurulur. Faz 0'da ölçüm yok, ama **mekanizma**
kurulur ki Faz 1'de ilk ölçüm yazıldığında denetim otomatik işlesin.

```cpp
// Ölçümün yanında, kendi çeviri biriminde:
KERTERIZ_REGISTER_MEASUREMENT(GnssPosition, "gnss_position");

// Test ağacında. Makro GÖVDE bekler — kayıt ile testin kendisi tek yapıdır.
KERTERIZ_REGISTER_JACOBIAN_TEST(GnssPosition) {
  // analitik Jacobian'ı numeric_residual_jacobian ile karşılaştır
}
```

Gövde zorunluluğu bu adımın uygulanmasında eklendi. Kaydın gövdesiz hâli
(`KERTERIZ_REGISTER_JACOBIAN_TEST(GnssPosition);`) "testi olmayan test kaydı"na
izin verirdi; gövde beklendiğinde böyle bir kayıt **linklenmez**, çünkü makro
iç-bağlantılı bir fonksiyonun adresini alır ama tanımı bulunmaz.

Test paketi: registry'deki her ölçüm için Jacobian testi kaydı var mı → yoksa **düşer**.
Kayıtlı testler ayrıca **çağrılır**; yalnızca varlıklarını saymak kaydı boş bir beyana çevirirdi.

**DoD:** sahte bir ölçüm tipiyle mekanizma doğrulanmış — testi kaydedilmediğinde paket düşüyor.

---

### S11 · Doküman denetimi + **Denetim ⑤** — ½ gün

`tools/check_docs.py` — bu oturumda iki kez elle yakalanan hatayı otomatikleştirir.

Yasaklı semboller: `MatX H`, `struct Residual`, `MatX noise`, `CompositeState<`, <!-- denetim5:muaf-satir MatX H struct Residual MatX noise CompositeState< — S11 in kendi yasakli sembol listesi; yasagi anlatan uyari; sembolu yazmadan kural ifade edilemez -->
`EuclideanBlock`, `reset(const NavState`, `IntegrityState integrity`, `.inverse()`, `std::span` <!-- denetim5:muaf-satir EuclideanBlock reset(const NavState IntegrityState integrity .inverse() std::span — S11 in kendi yasakli sembol listesi; yasagi anlatan uyari; sembolu yazmadan kural ifade edilemez -->
— `.md` ve `.hpp`/`.cpp` dosyalarında.

Hariç tutulanlar — "bunu yapma" uyarıları ve tarihsel ADR metni — **açık işaretlemeyle**
verilir. Uygulamada anahtar kelime sezgiseli ("satırda *yasak* geçiyorsa geç") kasten
kullanılmadı: gerçek bir drift o kelimeyi taşıyan bir satıra düşebilir ve sessizce geçerdi.

```
<!-- denetim5:muaf <semboller> — <sebep> -->   ... blok ...   <!-- denetim5:muaf-son -->
| tablo satırı | ... | <!-- denetim5:muaf-satir <semboller> — <sebep> -->
```

Satır içi biçim tablolar içindir: kendi satırında bir HTML yorumu tabloyu bölerdi.

Muafiyet bir **allowlist**'tir, blanket değil. İki kural bunu korur: hiçbir yasaklı sembol
yakalamayan muafiyet **hatadır** (metin taşınmış, muafiyet unutulmuş), ve sebep metni
yakaladığı **her sembolü adıyla saymak zorundadır**. Kod bloğu gibi işaretçinin ancak
dışına konabildiği yerlerde muafiyeti dar tutan tek şey budur.

**DoD:** script CI'da; kasıtlı bir ihlal eklendiğinde düşüyor.

---

### S12 · ADR-1 ve ADR-2 dosyaları — ½ gün

`DECISIONS.md` büyüdü (463 satır, 22 karar). Ayrı dosyalara bölmeye başla:

```
docs/design/ADR-0001-ros-free-core.md
docs/design/ADR-0002-se23-state.md
docs/design/DECISIONS.md        ← dizin + henüz bölünmemiş kararlar
```

**Metin değişmez** — yalnızca taşınır. ADR'ler kapalıdır.

**DoD:** iki dosya ayrılmış, `DECISIONS.md` onlara link veriyor, içerik birebir aynı.

---

## 4. Günlük dağılım

| Gün | İş |
|---|---|
| 1 | S0 + S1 |
| 2 | S2 + S3 |
| 3 | S4 (CI yeşil rozet) |
| 4 | S5 |
| 5–6 | S6 |
| 7 | S7 |
| 8–9 | **S8** ★ |
| 10 | S9 başlangıç |
| (taşma) | S9 bitiş + S10 + S11 + S12 |

S9–S12 hafifçe taşarsa sorun değil; **S8 taşmamalı.** Taşıyorsa S12'yi Faz 1'e ertele.

---

## 5. Faz 0 bitiş koşulu

`CLAUDE.md` §3'teki liste:

- [ ] `colcon build` temiz
- [ ] CI Humble + Jazzy yeşil
- [ ] `docker build` + `docker run` tek komut
- [ ] SE₂(3) testleri geçiyor
- [ ] **Kapalı formlu lineer-Gauss testi geçiyor**
- [ ] Üreteç aynı tohumla bit-bit aynı
- [ ] Beş denetim CI'da zorunlu
- [ ] ADR-1, ADR-2 ayrı dosyada

Hepsi işaretlendiğinde: `CLAUDE.md` §3'ü **Faz 1** olarak güncelle, `v0.1.0` etiketle.

---

## 6. Faz 1'e devrederken

Faz 1'in ilk işi `EskfBackend`'dir ve Faz 0'ın üç çıktısına dayanır:

- `linear_update` → güncelleme matematiği hazır ve test edilmiş
- `numeric_residual_jacobian` → ilk `Measurement`'ın Jacobian'ı gün içinde doğrulanır
- `kerteriz_sim` → R6 gereği ESKF **önce** sentetik veride koşar, KITTI/NCLT sonra

Faz 1 sırası: `NavState` → `ImuPropagator` → `EskfBackend` → `GnssPosition` →
**sentetik smoke test** → `MeasurementBuffer` + `Estimator` → veri seti adaptörleri →
`robot_localization` kıyası.
