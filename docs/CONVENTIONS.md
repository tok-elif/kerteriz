# Konvansiyonlar

> **Bu dosya bağlayıcıdır.** Kestirim kodunda hataların çoğu yanlış matematikten değil,
> *karışık konvansiyondan* doğar — ve sessizce, bulunması çok zor biçimde.
> Matematik veya kod yazmadan önce ilgili bölümü oku.
>
> **Durum: dondurulmuş (architecture freeze).** Bu dosyadaki bir kuralı değiştirmek
> `docs/design/DECISIONS.md`'ye yeni bir ADR yazmayı gerektirir.

---

## 1. Çerçeveler (frames)

| Ad | Tanım |
|---|---|
| `W` (world) | Yerel teğet düzlem, **ENU**: x=Doğu, y=Kuzey, z=Yukarı. Orijini §1.1'deki politika belirler. |
| `B` (body) | IMU gövde çerçevesi. **REP-103**: x=ileri, y=sol, z=yukarı. |
| `S` (sensor) | Her sensörün kendi çerçevesi. `B`'ye göre extrinsic YAML'dan gelir. |

`base_link` ↔ `B` ilişkisi: varsayılan olarak `B = base_link`. Farklıysa YAML'da `imu_extrinsic` verilir.

### Notasyon

```
R_WB    : B çerçevesinden W çerçevesine döndüren rotasyon
p_WB    : B'nin orijininin W içinde ifade edilmiş konumu
v_WB    : B'nin W'ye göre hızı, W içinde ifade edilmiş
T_WB    : R_WB ve p_WB'yi taşıyan rijit dönüşüm
```

**Kural:** `X_AB` her zaman "B'den A'ya" demektir. Kodda da aynı: `R_WB`, `p_WB`, `v_WB`.

### 1.1 Orijin politikası

`W`'nin orijini GNSS'e bağlı **değildir**; YAML'daki politika belirler. GNSS'siz iç mekân
kullanımı birinci sınıf desteklenir.

```yaml
world:
  origin_policy: gnss_first_fix   # gnss_first_fix | manual_llh | first_pose | identity
  manual_llh: {lat: 41.0, lon: 29.0, alt: 100.0}   # yalnız manual_llh için
```

| Politika | Davranış | Kullanım |
|---|---|---|
| `gnss_first_fix` | İlk geçerli GNSS sabitlemesi orijin olur; o ana kadar kestirimci `kInitializing` | Dış mekân, GNSS var |
| `manual_llh` | Orijin YAML'dan sabit LLH | Tekrarlanabilir deney, harita hizalama |
| `first_pose` | İlk `relative_pose` ölçümünün başlangıcı orijin | İç mekân, odometri tabanlı |
| `identity` | Orijin = başlangıç durumu, coğrafi bağ yok | Simülasyon, birim testi |

**Coğrafi bağın olmadığı politikalarda (`first_pose`, `identity`) `W` yalnızca yerel
tutarlıdır.** Manyetometre ve GNSS ölçümleri bu modlarda ya devre dışıdır ya da orijin
kurulana kadar reddedilir.

---

## 2. Kuaterniyonlar

- **Hamilton konvansiyonu.** JPL **değil**. (manif Hamilton kullanır; tutarlıyız.)
- Matematiksel yazımda sıra: `q = [w, x, y, z]`, `w` skaler kısım.
- Rotasyon aktif yorumlanır: `v_W = q_WB ⊗ v_B ⊗ q_WB*`

### Eigen tuzağı

```cpp
Eigen::Quaterniond q(w, x, y, z);   // YAPICI: (w, x, y, z)
q.coeffs();                          // DÖNDÜRÜR: (x, y, z, w)  ← farklı!
```

`geometry_msgs/Quaternion` alan sırası `(x, y, z, w)`'dir.

**Kural:** ROS ↔ core kuaterniyon dönüşümü **yalnızca** `kerteriz_ros/src/conversions.cpp`
içinde yapılır. Başka hiçbir yerde `msg.orientation.w` gibi bir erişim olmaz.

---

## 3. Pertürbasyon ve Jacobian

### 3.1 Pertürbasyon

**Varsayılan: sağ pertürbasyon (right-plus).** manif'in varsayılanıyla aynı.

```
X = X̂ ⊞ δ  =  X̂ ∘ Exp(δ)          (⊞ = right-plus)
δ = X ⊟ X̂  =  Log(X̂⁻¹ ∘ X)        (⊟ = right-minus)
```

### 3.2 Jacobian: `J_res` — tek tanım

Bu proje **residual Jacobian'ı** kullanır. Ölçüm Jacobian'ı `H = ∂h/∂δ` **kullanılmaz**;
kodda `H` adı geçmez.

```
r(X)   = z ⊖ h(X)                                  (artık)
J_res  = ∂r(X ⊞ δ) / ∂δ  |_{δ=0}                   (residual Jacobian)
```

**Neden `H` değil.** Üç gerekçe:

1. Sayısal Jacobian testi residual'ı pertürbe eder (`util/numeric_jacobian.hpp`), yani
   doğrudan `J_res` üretir. `H` konvansiyonu seçilse test ile üretim arasında sessiz bir
   işaret uyuşmazlığı riski doğar.
2. Manifold değerli ölçümlerde artık `Log(ẑ⁻¹ ∘ h(X))` biçimindedir; ayrı bir Öklidyen
   `h` tanımlamak iyi tanımlı değildir. Residual türevi her durumda tanımlıdır.
3. Öklidyen özel durumda `J_res = −H`, ve §4'teki güncelleme standart EKF'e indirgenir.

**Neden `J_r` değil.** `Jr`, Lie gruplarının **sağ Jacobian**'ının standart gösterimidir
(`manif::SO3::Jr`) ve bu kod tabanında yoğun kullanılacaktır. İsim çakışması yasaktır.

### 3.3 InEKF istisnası

InEKF backend'i hata tanımı olarak sol-değişmez veya sağ-değişmez kullanır:

```
η_R = X X̂⁻¹     (sağ-değişmez hata)
η_L = X̂⁻¹ X     (sol-değişmez hata)
```

**Kural:** InEKF içindeki her fonksiyon isminde hangisini kullandığı geçer
(`propagate_right_invariant`). Her dosyanın başında hangi hatanın kullanıldığı yazılır.

---

## 4. Kovaryans güncelleme — sözleşmenin parçası

Projenin tezi kovaryansın anlamlı olmasıdır. Bu nedenle **Joseph formu ve simetrizasyon
opsiyonel optimizasyon değil, zorunludur.** Naif `(I−KH)P` formu kullanılmaz.

Her şey `J_res` cinsinden yazılır; `H` hiç geçmez, dolayısıyla işaret tuzağı ortadan kalkar:

```
S    = J_res P J_resᵀ + R
δ    = −P J_resᵀ S⁻¹ r
A    = I − P J_resᵀ S⁻¹ J_res
P⁺   = A P Aᵀ + P J_resᵀ S⁻¹ R S⁻¹ J_res P          (Joseph)
P⁺  ← ½ (P⁺ + P⁺ᵀ)                                   (simetrizasyon)
X⁺   = X ⊞ δ
```

NIS: `ν = rᵀ S⁻¹ r`, serbestlik derecesi = `residual_dim()`.

### 4.1 Explicit inverse yasağı

`S⁻¹` dokümantasyonda yazılır, **kodda hesaplanmaz**. `S.inverse()` yasaktır. <!-- denetim5:muaf-satir .inverse() — §4.1 explicit inverse yasagi; yasagi anlatan uyari; sembolu yazmadan kural ifade edilemez -->

```cpp
const auto S_ldlt = S.ldlt();          // S simetrik pozitif tanımlı
const MatType S_inv_r     = S_ldlt.solve(r);
const MatType S_inv_Jres  = S_ldlt.solve(J_res);
```

Gerekçe: açık tersleme sayısal olarak kötü koşullu ve gereksiz pahalıdır. Kovaryans
doğruluğunu merkeze koyan bir projede savunulamaz.

---

## 5. Durum vektörü

```
X = [ çekirdek (15 dof, sabit) | augmentation (çalışma anında, sınırlı) ]
```

### 5.1 Çekirdek — derleme zamanı, sabit

```
T ∈ SE₂(3)  (R_WB, v_WB, p_WB),  b_g ∈ R³,  b_a ∈ R³
```

Teğet uzay sırası — **değiştirme**:

```
δ_core = [ δθ(3)   δv(3)   δp(3)   δb_g(3)   δb_a(3) ]
          0..2     3..5    6..8    9..11     12..14
```

### 5.2 Augmentation — çalışma anında, iki ayrı yaşam döngüsü

Tek depolama, **iki farklı blok türü**. Karıştırılmaz:

| Tür | Yaşam döngüsü | Örnek |
|---|---|---|
| `PersistentCalibration` | Yapılandırmada etkinleştirilir, oturum boyunca kalır. Marjinalleştirilmez. | teker ölçek faktörü, iz genişliği, sensör extrinsic, zaman gecikmesi |
| `Clone` | Çalışma anında push/pop edilir, cross-covariance taşır, kullanıldıktan sonra marjinalleştirilir. | göreli poz ölçümünün başlangıç anı |

Sıralama: `[ çekirdek | persistent (kayıt sırasına göre) | clone (push sırasına göre) ]`

### 5.3 Sınırlı kapasite

Depolama başta `kMaxStateDof` kapasitesiyle ayrılır; `active_dof()` çalışma anında değişir.
**Heap'te çalışma anında yeniden boyutlandırma yapılmaz.** Bu, §8.1'deki tahsis kuralıyla
uyumludur: aktif boyutun değişmesi serbest, yeniden tahsis yasaktır.

```cpp
static constexpr int kCoreDof       = 15;
static constexpr int kMaxAugmentDof = 48;                     // YAML'la değil, derlemeyle
static constexpr int kMaxStateDof   = kCoreDof + kMaxAugmentDof;
```

Kapasite aşılırsa yapılandırma **başarısız olur** — sessizce kırpılmaz.

---

## 6. Zaman

İki ayrı kavram, iki ayrı tip. Bunları karıştırmak yasaktır:

| Kavram | Tip | Kural |
|---|---|---|
| **Mutlak zaman** (timestamp) | `TimeNs = std::int64_t`, nanosaniye | `double` saniye olarak **asla** tutulmaz veya taşınmaz — precision kaybı sapma üretir |
| **Süre / aralık** (`dt`) | `Scalar` (double), **saniye** | Yalnızca iki `TimeNs`'in farkından hesaplanır, biriktirilmez |

```cpp
const Scalar dt = static_cast<Scalar>(u.stamp_ns - last_stamp_ns_) * 1e-9;  // doğru
```

Yasak olan, zaman damgasını `double` tutmaktır. Yerel `dt`'nin `double` saniye olması
doğru ve beklenen davranıştır.

**Süre biriktirme yasağı:** `t += dt` biçiminde zaman ilerletilmez. Mutlak zaman her zaman
kaynaktan gelen `TimeNs`'tir.

---

## 7. Birimler

| Büyüklük | Birim |
|---|---|
| Uzunluk | m |
| Hız | m/s |
| Açı | **rad** (derece yalnızca YAML'da, yüklemede çevrilir) |
| Açısal hız | rad/s |
| İvme | m/s² |

**Yer çekimi:** `g = 9.80665 m/s²`. ENU'da `g_W = [0, 0, −g]`.

### IMU ölçüm modeli

```
ω_m = ω + b_g + n_g       ḃ_g = n_bg
a_m = a + b_a + n_a       ḃ_a = n_ba
```

**Ölçüm = gerçek + bias + gürültü.** Ters işaret konvansiyonu kullanılmaz.

```yaml
imu:
  gyro_noise_density:    1.0e-4   # rad/s/√Hz
  gyro_random_walk:      1.0e-6   # rad/s²/√Hz
  accel_noise_density:   1.0e-3   # m/s²/√Hz
  accel_random_walk:     1.0e-5   # m/s³/√Hz
```

---

## 8. Kod stili ve tahsis

- **C++17.** (Humble/GCC 11 hedefi; C++20 kullanma.)
- Tipler `PascalCase`, fonksiyon/değişken `snake_case`, üye `trailing_underscore_`.
- Ham `new`/`delete` yok.
- Her public sembol Doxygen yorumu alır; Jacobian döndüren her fonksiyon `J_res`
  konvansiyonuna atıf yapar.
- `clang-format` (LLVM tabanlı, 100 sütun) ve `clang-tidy` CI'da zorunlu.

### 8.1 Tahsis kuralı — sayısal hot path

**`predict`, `update` ve `Measurement::evaluate` içinde heap tahsisi yoktur** (ADR-16, ADR-22).
Bu yalnızca residual için değil, sayısal çalışma alanının tamamı için geçerlidir: artık,
`J_res`, gürültü matrisi ve ara lineer-cebir sonuçları.

Uygulama: sabit kapasiteli tipler (`Eigen::Matrix<Scalar, kMaxResidualDim, ...>`) veya
çağıran tarafından sağlanan çalışma alanı (`MeasurementWorkspace&`). Runtime resize yapılmaz.

**Değerle dinamik matris döndürmek yasaktır.** `MatX noise() const` gibi bir imza her <!-- denetim5:muaf-satir MatX noise — §8.1 tahsis yasagi; yasagi anlatan uyari; sembolu yazmadan kural ifade edilemez -->
çağrıda tahsis edebilir; yerine çalışma alanına yazılır.

`MeasurementBuffer`'ın ölçüm/olay **sahipliği** bu hot-path sınırının dışındadır;
`std::unique_ptr<Measurement>` genel ROS/masaüstü kullanımında kabul edilir. Hard-real-time
bir port gerekirse sahiplik katmanı object pool/fixed-capacity variant ile ayrıca tahsissizleştirilir.

---

## 9. Test konvansiyonları

- Her `J_res` → merkezi farkla sayısal Jacobian testi. Tolerans `1e-6` mutlak / `1e-5` göreli.
- Her Lie grubu işlemi → `Exp(Log(X)) == X` ve `X ⊞ (Y ⊟ X) == Y`.
- **Kapalı formlu lineer-Gauss testi zorunludur.** Bilinen analitik çözümü olan skaler bir
  problemde §4'teki güncellemenin *işaretini ve değerini* doğrular. `J_res` işaret
  konvansiyonunu koruyan tek gerçek savunma budur.
- Her backend → sentetik veride tutarlılık dumanı testi.
- **Core testleri ROS olmadan çalışmak zorundadır.**
- Test isimleri okunabilir: `TEST(EskfBackend, RejectsMeasurementBeyondChiSquareGate)`

---

## 10. NIS kabul oranı kriteri

Kabul oranı sabit bir banda değil, **yapılandırılan güven seviyesine** göre denetlenir.

`gate.chi2_confidence = c` ise, model doğruyken beklenen kabul oranı `c`'dir.
Sonlu örneklemde gözlenen oran `Binom(N, c)/N` dağılır:

```
σ = √( c(1−c) / N )
kabul bandı ≈ c ± 3σ
```

Örnek: `c = 0.997`, `N = 1000` → `σ ≈ 0.17%` → beklenen bant `[99.2%, 100%]`.

**Sabit "%95–99 olmalı" kriteri kullanılmaz** — `c = 0.997` için doğru çalışan bir filtreyi
hatalı işaretler. `kerteriz_eval` bandı `c` ve `N`'den hesaplar.

---

## 11. YAML yapılandırma

Platforma özel her sayı YAML'dadır. Kodda gömülü platform sabiti yoktur.

```yaml
world:
  origin_policy: gnss_first_fix

sensors:
  - name: gnss_main
    type: gnss_position
    topic: /fix
    extrinsic: {xyz: [0.0, 0.0, 0.35], rpy_deg: [0, 0, 0]}
    noise: {horizontal_std: 1.5, vertical_std: 3.0}
    gate: {chi2_confidence: 0.997}
    enabled: true

  - name: wheel
    type: wheel_velocity
    topic: /wheel_speeds
    params: {radius: 0.165, track_width: 0.52, ticks_per_rev: 4096}
    estimate_scale_online: true      # PersistentCalibration bloğu ekler
    enabled: true
```

`estimate_scale_online: true` gibi bayraklar **çalışma anında** `PersistentCalibration`
bloğu kaydeder; yeniden derleme gerekmez. Toplam `kMaxAugmentDof`'u aşarsa yapılandırma
hata verir.
