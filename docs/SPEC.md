# Kerteriz — Proje Spesifikasyonu

> **Kendi kendine yeten doküman.** Bu projeyi hiç görmemiş bir mühendis veya AI ajanı
> yalnızca bu dosyayı okuyarak ne yapılacağını ve neden yapıldığını anlayabilmelidir.
> Konuşma geçmişine, dış bağlama veya "daha önce konuştuğumuz gibi" ifadelerine atıf yoktur.

---

## 1. Tek cümlede

ROS 2 için, kestirim hatasına çalışma anında sayısal üst sınır (*protection level*) üreten,
tutarlılığı Monte Carlo NEES analiziyle doğrulanmış, SE₂(3) üzerinde değişmez (invariant) EKF
tabanlı çok-sensörlü füzyon çerçevesi.

---

## 2. Problem

Açık kaynak dünyasında IMU + GNSS füzyonu yapan yüzlerce repo var. Neredeyse hepsi aynı üç şeyi yapıyor:
bir yörünge çiziyor, grafik koyuyor, "çalışıyor" diyor.

Hiçbiri şu soruyu cevaplamıyor: **filtrenin ürettiği kovaryans doğru mu?**

Bu önemsiz bir ayrıntı değil. Kestirimciyi tüketen her şey — yol planlama, kontrol, emniyet
mantığı — kovaryansa güvenerek karar verir. Aşırı-iyimser bir kovaryans, sapmış bir konumdan
daha tehlikelidir: sistem yanlış olduğunu bilmez.

ROS'un fiili standardı `robot_localization` da bu boşluğu kapatmıyor: tutarlılık aracı sunmuyor,
sensör arızasını tespit etmiyor, hatasına sınır üretmiyor.

## 3. Yaklaşım

Üç katman, üçü birlikte tezi oluşturur:

| Katman | Ne | Nasıl |
|---|---|---|
| **Yapısal olarak daha iyi koşullu** | Linearizasyon kaynaklı tutarsızlığı azalt | Navigasyon çekirdeği SE₂(3) üzerinde. **Bias'sız** durumda yer çekimli IMU kinematiği bu grupta *group-affine*'dir; hata log-lineer ilerler ve linearizasyon noktasından bağımsızdır. Bias ve diğer augmentasyonlar bu özelliği tam korumaz (*imperfect InEKF*) — bu yüzden avantaj **varsayılmaz, ölçülür**. |
| **Tutarlılığı ispatlanmış** | İddiayı ölç | Monte Carlo NEES (durum) ve çevrimiçi NIS (yenilik), χ² güven bantlarıyla. Simülasyonda ground truth ve tekrarlanabilir gürültü. |
| **Çalışma anında farkında** | Sahada kendini izlesin | NIS tabanlı χ² kapılama, arıza tespiti ve izolasyonu, protection level, kademeli bozulma ve otomatik toparlanma. |

---

## 4. Kapsam

### Kapsam içi

- IMU merkezli yayılım, çoklu yardımcı sensörle güncelleme
- İki filtre backend'i: ESKF ve InEKF, aynı arayüz arkasında, çalışma anında seçilebilir
- Ölçümler: GNSS konum/hız, teker hızı, göreli poz, non-holonomik kısıt, ZUPT, manyetometre, barometre
- Çevrimiçi kalibrasyon: IMU biasları, teker ölçek faktörü ve iz genişliği, opsiyonel extrinsic
- Sırasız ölçüm yönetimi (geri sar + yeniden yay)
- Bütünlük izleme: NIS, FDI, protection level
- Değerlendirme altyapısı: simülatör, Monte Carlo, NEES/NIS, ATE/RPE, arıza enjeksiyonu
- ROS 2 lifecycle düğümü, composable component, teşhis yayını

### Kapsam dışı — bunları yazma

| Kapsam dışı | Neden |
|---|---|
| Kendi LiDAR/görsel odometri ön-ucu | Altı hafta alır, sonuç KISS-ICP'den kötü olur. Katkımız ön-uç değil kestirimci. Hazır olanı `relative_pose` arayüzünden tüket. |
| SLAM, döngü kapama, harita | Farklı problem. Bu bir odometri/navigasyon kestirimcisidir. |
| Öğrenme tabanlı bileşenler | Tez model-tabanlı tutarlılık üzerine. ML eklemek hikâyeyi bulandırır. |
| Çoklu robot / dağıtık kestirim | Ayrı bir proje. |
| Gerçek zamanlı gömülü optimizasyon | Taşınabilir olması hedef, gömülüye *port edilmiş* olması değil. |
| GNSS ham gözlem işleme (RTK, PPP) | GNSS alıcısının çözümünü tüketiyoruz, kendimiz çözmüyoruz. |

---

## 5. Mimari

Tek belirleyici kural: **kestirim çekirdeği ROS bilmez.**

```
┌─ kerteriz_ros ────────── LifecycleNode · component · conversions · TF2 · diagnostics
│                          (ince adaptör, iş mantığı yok)
├─ kerteriz_core ───────── saf C++17, bağımlılık: Eigen + manif
│   ├─ state/              NavState (15-DoF çekirdek + sınırlı augmentation), StateBundle
│   ├─ process/            ImuPropagator
│   ├─ backends/           FilterBackend → Eskf · Inekf
│   ├─ measurements/       Measurement → Gnss · Wheel · RelativePose · NHC · ZUPT
│   ├─ buffer/             MeasurementBuffer (sırasız ölçüm)
│   ├─ integrity/          NisMonitor · FaultDetector · ProtectionLevel
│   └─ util/               numeric_jacobian
├─ kerteriz_sim ────────── yörünge üreteci, gürültü ve arıza enjeksiyonu
└─ kerteriz_eval ───────── Python: evo, NEES/NIS, Stanford diyagramı, rapor
```

Arayüz sözleşmesi: [`INTERFACES.md`](INTERFACES.md) · Kararların gerekçeleri: [`design/DECISIONS.md`](design/DECISIONS.md)

### Dondurulmuş mimari kararlar

Kod yazılmadan önce ADR-13…22 ile kapatılan noktalar. Bunlar sözleşmenin parçasıdır,
tercih değil:

| Konu | Karar |
|---|---|
| Jacobian | `J_res = ∂r/∂δ`; `H` kullanılmaz. Düzeltme `δ = −P J_resᵀ S⁻¹ r` |
| Durum | 15-DoF sabit çekirdek + sınırlı kapasiteli augmentation (`PersistentCalibration` \| `Clone`) |
| Geri sarma | `PipelineSnapshot` sahibi `Estimator`; backend yalnız `BackendSnapshot` taşır. Replay'i etkileyen *tüm* mutable state dahil (ADR-15,20) |
| Tahsis | Sayısal hot path (`predict/update/evaluate`) tahsissiz; değerle dinamik matris dönülmez. Kuyruk sahipliği bu sınırın dışındadır (ADR-22) |
| Kovaryans | Joseph formu + simetrizasyon **zorunlu**; `S.inverse()` yasak, `LDLT` ile çözülür <!-- denetim5:muaf-satir .inverse() — kovaryans kurali; yasagi anlatan uyari; sembolu yazmadan kural ifade edilemez --> |
| Başlatma | `EstimatorMode` (sistem) ≠ `SensorHealth` (tek sensör) |
| Red sebebi | `UpdateResult` (filtre) ≠ `ProcessingResult` + `RejectReason` (ardışık düzen) |
| InEKF iddiası | Exact group-affine/log-linear avantaj bias'sız SE₂(3) çekirdeğine aittir; genişletilmiş durumun tutarlılığı varsayılmaz, NEES/NIS ile ölçülür (ADR-21) |
| Zaman | mutlak = `TimeNs` (int64 ns); süre = `Scalar` saniye, ns farkından |

---

## 6. Hedef ortam

| | |
|---|---|
| Birincil | Ubuntu 22.04 + **ROS 2 Humble** |
| CI matrisi | Humble **ve** Jazzy |
| Dil | C++17 (GCC 11 uyumlu — C++20 kullanma) |
| Bağımlılık | Eigen 3.4, [manif](https://github.com/artivis/manif) (FetchContent), GoogleTest |
| Opsiyonel | GTSAM (uzatma), KISS-ICP (göreli poz kaynağı) |
| Python | 3.10, `evo`, `numpy`, `matplotlib`, `rosbags` |

---

## 7. Veri setleri

| Set | Rol | Özellik |
|---|---|---|
| **Simülasyon** (`kerteriz_sim`) | E1, E5 | Ground truth ve tekrarlanabilir gürültü. Monte Carlo buradan. |
| **NCLT** (Michigan) | E4, E7 — **teker odometrisi ana seti** | Segway yer robotu. 10 Hz enkoder, 50 Hz IMU, RTK GPS, Velodyne. 147 km, 27 oturum. İç **ve** dış mekân → GNSS gerçekten kesiliyor. |
| **KITTI raw** | E2 | Taban çizgisi kıyası. Yaygın bilinir, güvenilir. *Teker enkoderi yok.* |
| **UrbanNav** (Hong Kong) | E3 | Kent kanyonu, multipath, ham RINEX, SPAN-CPT ground truth. *Teker enkoderi yok.* |
| KAIST Complex Urban | opsiyonel | Araba ölçeğinde teker enkoderi (RLS LM13). |

UrbanNav ROS 1 bag formatındadır: `pip install rosbags && rosbags-convert <dosya>.bag`

---

## 8. Deneyler

Her deney bir şekil üretir, her şekil bir iddiayı kanıtlar. Çıktılar `results/` altında versiyonlanır.

| # | Deney | Veri | Kanıtladığı |
|---|---|---|---|
| E1 | Monte Carlo tutarlılık: 500 koşu, ortalama NEES + %95 χ² bandı | sim | **Projenin en değerli grafiği.** Büyük yaw belirsizliğinde ESKF banttan çıkar, InEKF çıkmaz. |
| E2 | Doğruluk kıyası: ATE/RPE, 3 kestirimci × 6 dizi | KITTI | `robot_localization`'a karşı sayısal üstünlük |
| E3 | Kent kanyonu: multipath altında kapılama | UrbanNav | Naif vs. χ² kapılı güncelleme |
| E4 | GNSS kaybı: iç mekâna girişte sapma eğrisi | NCLT + UrbanNav tünel | Teker odometrisi devralıyor; NHC/ZUPT/teker katkıları ayrıştırılmış |
| E5 | Arıza enjeksiyonu: ramp/step/multipath → ROC | sim + KITTI | Tespit gecikmesi, yanlış alarm, kaçırılan tespit |
| E6 | Bütünlük: Stanford diyagramı (hata vs. PL) | tüm setler | PL hedef bütünlük riskinde gerçek hatayı sınırlıyor |
| E7 | Çevrimiçi teker kalibrasyonu: ölçek faktörü yakınsaması | NCLT | Kalibrasyonu duruma koymanın somut karşılığı |

**Kritik kural — faz planıyla uyumlu hale getirildi.** Filtreyi asla önce gerçek veride hata
ayıklama. Bu kuralın tutulabilmesi için minimal üreteç **Faz 0'da** hazır olur ve her filtre
Faz 1'de **gerçek-veri adaptörlerinden önce** sentetik veride doğrulanır. Simülasyonda ground
truth ve tekrarlanabilir gürültü var; hatayı orada bulursun. Gerçek veride yalnızca "sapıyor"
diyebilirsin. Full Monte Carlo ve deney harness'ı Faz 2'de kalır — Faz 0 şişmez.

---

## 9. Fazlar

Her faz sonunda repo **tamamlanmış görünür**. Yarım kalan faz yoktur.

| Faz | Hafta | İçerik | Tamamlandı sayılır |
|---|---|---|---|
| **0** Temel | 1–2 | İskelet, CMake, Docker, CI matrisi, manif, sayısal Jacobian aracı, **minimal deterministik yörünge/sensör üreteci**, 4 otomatik denetim | Build + CI yeşil; SE₂(3) testleri geçiyor; kapalı formlu lineer-Gauss testi geçiyor; üreteç tohumlu ve tekrarlanabilir; ADR-1,2 yazılı |
| **1** ESKF | 3–5 | IMU yayılımı, GNSS, teker hızı, NHC, ZUPT, ölçüm tamponu → **sentetik smoke test** → sonra KITTI+NCLT adaptörü | Sentetik veride ESKF yakınsıyor ve NIS makul (**gerçek veriye geçiş ön koşulu**); KITTI'de `robot_localization` ile denk ATE; tüm Jacobian testleri geçiyor; **repo paylaşılabilir (MVP)** |
| **2** Kanıt | 6–8 | Üretecin Monte Carlo'ya genişletilmesi, NEES/NIS analizi, evo, otomatik rapor | E1 grafiği üretiliyor; `make results` her şeyi sıfırdan üretiyor; filtrenin iyimser olduğu bir senaryo dürüstçe raporlanmış |
| **3** InEKF + odometri | 9–12 | InEKF backend, karşılaştırma, gözlemlenebilirlik; son hafta: göreli poz + stochastic cloning + çevrimiçi teker kalibrasyonu | İki backend aynı arayüzde; E1'de NEES farkı görünüyor; E7 yakınsıyor; blog yazısı yayında |
| **4** Bütünlük | 13–15 | χ² kapılama, FDI, protection level, arıza enjeksiyonu | E5 ROC eğrisi; E6 Stanford diyagramında HMI bölgesi boş; `ProtectionLevel` rviz'de |
| **5** Yayın | 16–17 | Doxygen, README, demo videosu, teknik rapor, v1.0.0 | README ilk ekranda sonuç tablosu; video yayında; release çekilmiş |

### Geri kalınırsa kısma sırası

**Faz 5 kısalt → Faz 4 tamamen at → Faz 3 at.** **Faz 2 asla atılmaz** — projeyi oyuncak
olmaktan çıkaran tek şey odur. Yarım bırakılmış bir Faz 4, hiç başlanmamış olandan kötüdür.

---

## 10. Başarı ölçütü

Proje şu üç soruya "evet" diyebiliyorsa başarılıdır:

1. **Doğru mu?** KITTI'de `robot_localization`'a karşı ATE eşit veya daha iyi.
2. **Tutarlı mı?** Monte Carlo NEES χ² bandı içinde; NIS ortalaması serbestlik derecesine yakın.
3. **Dürüst mü?** Filtrenin kötü olduğu senaryolar bulunmuş ve raporlanmış; protection level
   gerçek hatayı hedeflenen risk seviyesinde sınırlıyor.

Yalnızca "güzel bir yörünge grafiği" başarı sayılmaz.

---

## 11. Risk ve savunma

| Risk | Savunma |
|---|---|
| Kapsam kayması | Faz kapıları. Yeni fikirler `ideas.md`'ye yazılır, koda değil. |
| Veri seti bataklığı | Adaptöre faz başına en fazla 4 gün. `rosbags-convert` ile bir kere çevir. |
| Filtre sapıyor, sebep bulunamıyor | Sayısal Jacobian testleri + önce simülasyon. İkisi olmadan gerçek veriye dokunma. |
| "Yine bir EKF reposu" algısı | README'nin ilk ekranı tez + sonuç tablosu olsun, kurulum talimatı değil. |
| Bitmeden bırakma | Her faz sonunda release çek ve paylaş. Dışarıya verilmiş söz bitirme baskısı yaratır. |
