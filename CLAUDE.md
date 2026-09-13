# CLAUDE.md — Kerteriz

> Bu dosya, bu repoda çalışan her AI ajanının **ilk okuyacağı** dosyadır.
> Kurallar tavsiye değil, kısıttır. Bir kuralı çiğnemen gerekiyorsa önce sor.
>
> **Mimari dondurulmuştur: ADR-13…23 bağlayıcıdır.** Yeniden tasarım önerme.
> Bir karara katılmıyorsan uygula ve ayrıca not düş; tartışma kod yazmayı geciktirir.

---

## 1. Proje nedir

**Kerteriz**, ROS 2 için çok-sensörlü durum kestirim çerçevesidir.

**Tez:** Çoğu kestirimci nerede olduğunu söyler. Kerteriz *ne kadar güvenebileceğini* de söyler.

Üç katman:

1. **Yapısal olarak daha iyi koşullu** — navigasyon çekirdeği SE₂(3) üzerinde
2. **Tutarlılığı ölçülmüş** — Monte Carlo NEES / çevrimiçi NIS, χ² bantlarıyla
3. **Çalışma anında farkında** — FDI ve protection level

Kendi kendine yeten tam tanım: [`docs/SPEC.md`](docs/SPEC.md)

---

## 2. Bozulmaz kurallar

| # | Kural | Neden |
|---|---|---|
| R1 | **`kerteriz_core` asla ROS'a bağımlı olmaz.** `#include <rclcpp/...>`, ROS mesaj tipi, `rclcpp` CMake bağımlılığı core içinde geçemez. | Taşınabilirlik, ROS'suz test, gömülü hedef. |
| R2 | **Her `J_res`'in sayısal türev testi olur.** Testsiz Jacobian merge edilmez. | Filtre hatalarının bir numaralı kaynağı. |
| R3 | **Konvansiyonlar `docs/CONVENTIONS.md`'de sabittir.** | Karışık konvansiyon = sessiz, bulunması imkânsız hata. |
| R4 | **Yeni sensör = yeni `Measurement` sınıfı + YAML.** Backend kodu değişmez. | Genişletilebilirlik iddiasının tek kanıtı. |
| R5 | **Platforma özel hiçbir sayı koda gömülmez.** | "Aynı ikili dosya, farklı YAML" hedefi. |
| R6 | **Önce simülasyon, sonra gerçek veri.** Her filtre gerçek-veri adaptöründen *önce* sentetik veride doğrulanır. | Gerçek veride hata ayıklamak zaman öldürür. |
| R7 | **Faz dışına çıkılmaz.** §3 kapsamı belirler. | Bu projelerin ölüm sebebi kapsam kayması. |
| R8 | **C++17.** `std::span`, `std::optional<T&>`, konsept, `<ranges>` gibi C++20 API'leri kullanılmaz. Görünüm gerekiyorsa `ArrayView` (INTERFACES §0). | Humble / GCC 11 hedefi (ADR-12). <!-- denetim5:muaf-satir std::span — R8 C++17 kurali; yasagi anlatan uyari; sembolu yazmadan kural ifade edilemez --> |

---

## 3. Şu an ne yapıyoruz

<!-- Her faz geçişinde GÜNCELLE. Ajan buraya bakıp kapsamı belirliyor. -->

**Aktif faz:** FAZ 0 — Temel ve iskelet · **Hafta:** 1–2

**Kapsam içi:**
- Repo iskeleti, CMake hedefleri, colcon yapısı
- Docker imajı (`ros:humble-ros-base`) + devcontainer
- GitHub Actions: Humble + Jazzy matrisi, build/test/lint
- manif entegrasyonu, SE₂(3) exp/log/adjoint birim testleri
- `ArrayView`, sabit kapasiteli tipler, `numeric_residual_jacobian`
- **Minimal deterministik yörünge/sensör üreteci** — tohumlu, tekrarlanabilir (R6'nın ön koşulu)
- **Beş otomatik denetim** (§6.1)
- ADR-1 ve ADR-2'nin ayrı dosyaya yazılması

**Kapsam dışı (şimdi yazma):**
- Herhangi bir filtre backend'i (ESKF dahil) — Faz 1
- Ölçüm modelleri — Faz 1
- `Estimator` orkestratörü, `MeasurementBuffer` — Faz 1
- Monte Carlo, NEES analizi, evo — Faz 2
- InEKF — Faz 3 · Klonlama, teker kalibrasyonu — Faz 3
- FDI, protection level ve `WeakDirection`'ın **çalışma anındaki kullanımı** — Faz 4.
  `WeakDirection` ortak veri tipi olarak S5'te tanımlanır (INTERFACES §0).

**Faz tamamlandı sayılır:**
- [ ] `colcon build` temiz geçiyor
- [ ] CI Humble ve Jazzy'de yeşil
- [ ] `docker build` + `docker run` tek komutta çalışıyor
- [ ] SE₂(3) testleri geçiyor (`Exp(Log(X))==X`, `X ⊞ (Y ⊟ X)==Y`)
- [ ] **Kapalı formlu lineer-Gauss testi geçiyor** — `J_res` işaretini koruyan test
- [ ] Üreteç aynı tohumla bit-bit aynı çıktıyı veriyor
- [ ] Beş otomatik denetim CI'da zorunlu
- [ ] ADR-1, ADR-2 yazılmış

---

## 4. Nereye ne yazılır

```
kerteriz_core/     saf C++17. Eigen + manif dışında bağımlılık YOK.
kerteriz_ros/      ince ROS 2 adaptörü. İş mantığı BURAYA YAZILMAZ.
kerteriz_msgs/     özel mesaj tanımları
kerteriz_sim/      yörünge üreteci, gürültü ve arıza enjeksiyonu
kerteriz_eval/     Python değerlendirme: evo, NEES/NIS, raporlar
kerteriz_bringup/  launch dosyaları, veri seti adaptörleri
docs/theory/       matematik türetmeleri
docs/design/       ADR'ler
results/           üretilmiş şekil ve tablolar (commit'lenir)
```

**Bir kod nereye ait?** Testi ROS kurulmadan çalışabiliyorsa → `kerteriz_core`. Aksi halde → `kerteriz_ros`.

---

## 5. Referans dosyalar

| Dosya | Ne zaman oku |
|---|---|
| [`docs/SPEC.md`](docs/SPEC.md) | Projeye ilk kez bakıyorsan |
| [`docs/CONVENTIONS.md`](docs/CONVENTIONS.md) | **Matematik veya kod yazmadan önce, her seferinde** |
| [`docs/INTERFACES.md`](docs/INTERFACES.md) | Bir sınıf ekliyor veya değiştiriyorsan |
| [`docs/INTEGRATION.md`](docs/INTEGRATION.md) | Yeni platform/sensör desteği ekliyorsan |
| [`docs/design/DECISIONS.md`](docs/design/DECISIONS.md) | "Neden böyle yapılmış?" |
| [`docs/AI-HANDOFF.md`](docs/AI-HANDOFF.md) | İşi başka bir ajana devrediyorsan |
| [`docs/PHASE0.md`](docs/PHASE0.md) | **Aktif faz planı** — adım sırası, dosya listesi, DoD |

---

## 6. Doğrulama komutları

```bash
colcon build --symlink-install
colcon test --packages-select kerteriz_core && colcon test-result --verbose
cmake -S kerteriz_core -B build/core && cmake --build build/core && ctest --test-dir build/core
pre-commit run --all-files
```

**Bir değişikliği "bitti" saymadan önce:** dördü de geçmeli.

### 6.1 Beş otomatik denetim

Mimari freeze ancak zorlanabiliyorsa gerçektir. Bunlar CI'da zorunludur:

| # | Denetim | Nasıl |
|---|---|---|
| 1 | **R1 — core ROS'suz** | `grep -rE "rclcpp\|_msgs/" kerteriz_core/` boş dönmeli |
| 2 | **Durum kapasitesi** | `static_assert(kCoreDof + kMaxAugmentDof == kMaxStateDof)` + kayıtta kapasite kontrolü |
| 3 | **`J_res` işareti** | Kapalı formlu lineer-Gauss testi. `J_res` yerine `−J_res` yazılırsa sayısal Jacobian testi geçer, **bu test düşer**. |
| 4 | **Jacobian testi zorunluluğu** | Test registry üzerinden: `Measurement` fabrikaya kaydolurken Jacobian testini de kaydeder; testsiz ölçüm varsa test paketi düşer. *Dosya tarayan kırılgan script yazma.* |
| 5 | **Doküman tutarlılığı** | Yasaklı sembol taraması (`MatX H`, `struct Residual`, `MatX noise`, `CompositeState<`, `reset(const NavState`, `std::span`, `.inverse()`) tüm `.md` ve `.hpp` dosyalarında. Hariç tutmalar `tools/check_docs.py` tarafından **açık işaretlemeyle** verilir (`denetim5:muaf`); işaretleme yakaladığı her sembolü adıyla saymak zorundadır, yani muafiyet blanket değil allowlist'tir. Kod tarafında muafiyet yoktur. <!-- denetim5:muaf-satir MatX H struct Residual MatX noise CompositeState< reset(const NavState std::span .inverse() — Denetim 5 in kendi yasakli sembol listesi; yasagi anlatan uyari; sembolu yazmadan kural ifade edilemez --> |

Denetim 3 ve 4 bu projenin en önemli iki testidir.

---

## 7. Commit ve dal

- Conventional commits: `feat(core): add SE_2(3) state`, `fix:`, `docs:`, `test:`, `refactor:`, `chore:`
- Doğrudan `main`'e commit yok; dal aç, PR aç
- Bir PR bir konu

---

## 8. Sık yapılan hatalar

- **Eigen kuaterniyon sırası.** `Eigen::Quaterniond(w,x,y,z)` yapıcısı ile `.coeffs()` → `(x,y,z,w)` farklıdır. ROS mesajı `(x,y,z,w)`. Dönüşüm **yalnızca** `conversions.cpp`'de.
- **Zaman damgasını `double` saniye tutmak.** Mutlak zaman her zaman `TimeNs` (int64 ns). `dt`'nin `double` saniye olması **doğrudur** — iki `TimeNs` farkından hesaplanır (CONVENTIONS §6).
- **`J_res` işaretini ters yazmak.** `J_res = ∂r/∂δ`, `r = z ⊖ h(X)` — Öklidyen durumda `J_res = −H`. Düzeltmede eksi vardır: `δ = −P J_resᵀ S⁻¹ r`. Sayısal Jacobian testi bu hatayı **yakalamaz**; kapalı formlu test yakalar.
- **`H` adını kullanmak.** Bu kod tabanında ölçüm Jacobian'ı yoktur. `Jr` ise Lie sağ Jacobian'ıdır (`manif::…::Jr`) — karıştırma.
- **`S.inverse()` yazmak.** Yasak (ADR-17). `S.ldlt().solve(...)` kullan. <!-- denetim5:muaf-satir .inverse() — ADR-17 yasagi; yasagi anlatan uyari; sembolu yazmadan kural ifade edilemez -->
- **Naif kovaryans güncellemesi.** Joseph formu ve `P ← ½(P+Pᵀ)` zorunludur, opsiyonel optimizasyon değil (ADR-17).
- **Sayısal hot path'te tahsis.** `predict`, `update`, `evaluate` ve bunların çağırdığı matris işlemlerinde heap tahsisi, runtime resize ve değerle dinamik matris döndürme yasaktır (ADR-22). Ölçüm `MeasurementWorkspace&` doldurur. *Kuyruk sahipliği (`std::unique_ptr<Measurement>`) bu sınırın dışındadır — orada tahsis serbesttir.*
- **`std::span` yazmak.** C++20. `ArrayView` kullan (R8). <!-- denetim5:muaf-satir std::span — R8 yasagi; yasagi anlatan uyari; sembolu yazmadan kural ifade edilemez -->
- **`reset(x, P)` aramak.** Yoktur. Backend `save_snapshot()` → `BackendSnapshot`; **tam snapshot'ın sahibi `Estimator`'dür** ve `PipelineSnapshot` döner (backend + NIS + FDI). Backend bütünlük katmanını bilmez (ADR-15, ADR-20).
- **Göreli ölçümü mutlak gibi işlemek.** Odometri iki anı bağlar; klonlama olmadan korelasyon kaybolur (ADR-9, ADR-14).
- **InEKF avantajını fazla güçlü ifade etmek.** Exact group-affine / log-linear özellik **bias'sız** SE₂(3) çekirdeğine aittir. Bias, kalibrasyon ve klonlarla yaklaşıklaşır (*imperfect InEKF*). "Tanım gereği tutarlı" **yazma**; genişletilmiş durumun tutarlılığı E1/NEES ile ölçülür (ADR-21).
- **"Yakınsıyor" diye geçmek.** Kabul kriteri NEES/NIS'tir. Kabul oranı sabit %95–99 değil, yapılandırılan güvene bağlı binom bandıdır (CONVENTIONS §10).
