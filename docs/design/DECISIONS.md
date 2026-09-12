# Mimari Kararlar (ADR)

> Her kayıt: **karar**, **gerekçe**, **alternatifler**, **sonuçlar**.
> Kararlar kapalıdır — yeniden açmak için yeni bir ADR yaz, mevcut olanı düzenleme.
> Büyüdükçe her biri ayrı dosyaya (`ADR-0001-ros-free-core.md`) bölünebilir.

---

## ADR-1 · ROS'suz kestirim çekirdeği

**Karar.** `kerteriz_core` bağımsız bir CMake hedefidir. Bağımlılığı yalnızca Eigen ve manif'tir.
ROS başlık dosyası, ROS mesaj tipi veya `rclcpp` bağımlılığı içeremez. ROS 2 paketi çekirdeği sarar.

**Gerekçe.** Çekirdek ROS kurulmamış bir makinede derlenir, test edilir, profillenir. Testler
saniyeler içinde çalışır. Kod micro-ROS'a, gömülü hedefe, ROS 1 köprüsüne veya toplu işleme
hattına taşınabilir. OpenVINS, GTSAM ve Ceres aynı ayrımı kullanır.

**Alternatifler.** Her şeyi ROS düğümünde toplamak — daha hızlı başlar, test edilemez ve taşınamaz hale gelir.

**Sonuçlar.** Bir dönüşüm katmanı maliyeti doğar (`conversions.cpp`). Karşılığında CI hızlanır
ve taşınabilirlik iddiası kanıtlanabilir olur. CI'da çekirdeği ROS'suz derleyen ayrı bir iş vardır.

---

## ADR-2 · Durum SE₂(3) × R⁶ üzerinde

**Karar.** Navigasyon durumu genişletilmiş poz grubu SE₂(3) (rotasyon, hız, konum) artı
IMU biasları için R⁶'dır.

**Gerekçe.** Yer çekimli IMU kinematiği SE₂(3) üzerinde *group-affine*'dir. Bu, InEKF'in
hata dinamiğinin durumdan bağımsız olması için gereken ve yeten koşuldur: hata log-lineer
ilerler, linearizasyon noktası kaynaklı tutarsızlık ortadan kalkar. Tutarlılık ayarlamayla
değil, yapıdan gelir.

**Alternatifler.** SO(3) × R³ × R³ — yaygın, ama hata dinamiği duruma bağlı; büyük yaw
belirsizliğinde EKF sahte gözlemlenebilirlik kazanır ve aşırı-iyimser olur (E1'de görünür).

**Sonuçlar.** manif'in `SE_2_3` desteği zorunlu. Ekip için öğrenme eğrisi var; `docs/theory/`
altında türetme yazılır. Karşılığında InEKF backend'i doğal biçimde eklenebilir hale gelir.

---

## ADR-3 · Genel ⊞/⊟ bileşik manifold

**Karar.** Durum `CompositeState<Blocks...>` şablonudur; teğet uzay indeksleri derleme
zamanında hesaplanır.

**Gerekçe.** Yeni durum eklemek (teker ölçek faktörü, sensör extrinsic'i, GNSS anten kolu,
zaman gecikmesi) filtre kodunu değiştirmemelidir. Elle indeks yönetimi hata kaynağıdır ve
kalibrasyon parametrelerini duruma eklemeyi caydırır.

**Alternatifler.** Sabit boyutlu elle indekslenmiş durum — basit başlar, her genişlemede
onlarca yerde sihirli sayı düzeltmek gerekir.

**Sonuçlar.** Şablon karmaşıklığı ve derleme süresi artar. Karşılığında ADR-10'daki çevrimiçi
kalibrasyon neredeyse bedava gelir.

---

## ADR-4 · Ölçüm modelleri eklenti arayüzü

**Karar.** Tek soyut sınıf `Measurement`: artık, Jacobian, gürültü, zaman damgası.
Yeni sensör = bir sınıf + fabrikaya bir kayıt + bir YAML bloğu.

**Gerekçe.** Kerteriz tek bir sensör kümesi için değil, bir platform ailesi için yazılıyor.
Backend'in hangi sensörlerin var olduğunu bilmesi gerekmez.

**Alternatifler.** Her sensör için backend'de özel dal — sensör sayısıyla birlikte
kombinatoryal karmaşıklık.

**Sonuçlar.** Sanal fonksiyon çağrısı maliyeti (ölçüm hızlarında ihmal edilebilir).
Genişletilebilirlik iddiası ölçülebilir hale gelir: yeni sensör kaç dosya değiştiriyor?

---

## ADR-5 · Sırasız ölçüm tamponu ve yeniden yayılım

**Karar.** Kısa bir pencerede (varsayılan 200 ms) durum geçmişi tutulur. Geciken ölçüm
gelince o ana geri sarılır, ölçüm uygulanır, ileri yeniden yayılır.

**Gerekçe.** Gerçek sensörler geç ve sırasız gelir: GNSS çözümü hesaplama gecikmesi taşır,
görsel odometri kare işleme süresi kadar geride gelir, sürücü tamponları sırayı bozar.
Geciken ölçümü güncel duruma uygulamak sistematik hata üretir.

**Alternatifler.** Geç ölçümü atmak (bilgi kaybı) veya güncel duruma uygulamak (sessiz sapma).

**Sonuçlar.** Bellek ve hesap maliyeti pencere boyutuyla artar. Bu, hobi repolarının
neredeyse hiçbirinde bulunmayan bir saha gereksinimidir.

---

## ADR-6 · Her analitik Jacobian sayısal türevle test edilir

**Karar.** Manifold üzerinde merkezi farkla hesaplanan Jacobian'a karşı gtest. Testi olmayan
Jacobian merge edilmez.

**Gerekçe.** Filtre hatalarının bir numaralı kaynağı yanlış Jacobian'dır ve belirtisi
"filtre biraz kötü çalışıyor"dur — yani sessiz. Sayısal karşılaştırma bunu dakikalar içinde yakalar.

**Alternatifler.** Elle türetmeye güvenmek; yalnızca uçtan uca testler — hangi Jacobian'ın
yanlış olduğunu söylemez.

**Sonuçlar.** Her ölçüm için ek test yazma yükü. Karşılığında hata ayıklama süresinde
büyük tasarruf ve kod incelemesinde güçlü sinyal.

---

## ADR-7 · Lifecycle node ve bileşen kompozisyonu

**Karar.** ROS katmanı `rclcpp_lifecycle::LifecycleNode` olarak yazılır ve composable
component olarak da derlenir.

**Gerekçe.** Yapılandırma ile etkinleştirme ayrılır: parametreler yüklenip doğrulanabilir,
sensörler hazır olmadan abonelik açılmaz, kestirimci çalışma sırasında durdurulup yeniden
başlatılabilir. Kompozisyon, sürücülerle aynı süreçte çalışmayı ve kopyasız aktarımı sağlar.

**Alternatifler.** Düz `rclcpp::Node` — ROS 1 alışkanlığı; yapılandırma hataları çalışma
anında ortaya çıkar.

**Sonuçlar.** Geçiş mantığı yazma yükü. Karşılığında ROS 2'yi kendi modeliyle kullandığın görünür.

---

## ADR-8 · Teşhis birinci sınıf çıktıdır

**Karar.** Düğüm yalnızca `nav_msgs/Odometry` yayınlamaz. Ayrı bir teşhis mesajı sensör
başına NIS, ölçüm kabul oranı, bias kestirimleri, kovaryans koşul sayısı ve protection
level taşır; `diagnostic_msgs` ile entegre çalışır.

**Gerekçe.** Kestirimci sahada hata ayıklanacak bir sistemdir. "Sapıyor" raporunun
eyleme dönüşebilmesi için hangi sensörün, ne zaman, neden reddedildiği görünmelidir.

**Alternatifler.** Yalnızca log yazmak — makine tarafından izlenemez, kaydedilemez, çizilemez.

**Sonuçlar.** Ek mesaj tanımı ve yayın maliyeti. Karşılığında entegrasyon doğrulaması
(`INTEGRATION.md` §1.6) mümkün hale gelir.

---

## ADR-9 · Odometri iki ayrı yoldan girer

**Karar.** İki farklı ölçüm modeli:

- **`WheelVelocity`** — ham enkoder hızları. Tek bir ana ait *hız* ölçümüdür, SE₂(3)'ün hız
  bileşenine doğrudan bağlanır. Klonlama gerekmez. Faz 1.
- **`RelativePose`** — entegre `/odom` pozu. İki anı bağlayan *göreli* ölçümdür. Başlangıç
  anının durumu klonlanır ve korelasyon taşınır (stochastic cloning, Roumeliotis & Burdick 2002). Faz 3.

**Gerekçe.** Bu ayrım gözden kaçarsa göreli ölçüm mutlak gibi işlenir, korelasyon yok sayılır
ve filtre aşırı-iyimser olur — E1'de anında görünür. Ayrıca çoğu ROS robotu ham enkoder tiki
değil `/odom` yayınladığı için ikinci yol pratikte zorunludur.

**Alternatifler.** Yalnızca göreli poz desteklemek (ham enkoder için gereksiz karmaşıklık);
yalnızca hız desteklemek (`/odom` yayınlayan robotları dışarıda bırakır).

**Sonuçlar.** Backend'e klonlama arayüzü eklenir. Karşılığında görsel ve LiDAR odometrisi
de aynı `RelativePose` yolundan bedava gelir.

---

## ADR-10 · Teker kalibrasyonu durum vektöründe

**Karar.** Teker ölçek faktörü ve iz genişliği duruma eklenir, çevrimiçi kestirilir
(`estimate_scale_online: true`).

**Gerekçe.** Tekerin etkin yarıçapı sabit değildir: lastik basıncı, yük ve aşınma değiştirir.
Sabit varsayılan ölçek hatası doğrudan konum sapmasına dönüşür. Parametreler GNSS varken
gözlemlenebilir; robot iç mekâna girdiğinde *öğrenilmiş* değerle ölü hesap yapılır.

**Alternatifler.** Offline kalibrasyon — zamanla geçersizleşir, her platformda tekrar gerekir.

**Sonuçlar.** Durum boyutu büyür, gözlemlenebilirlik analizi gerekir (hareket yeterli mi?).
ADR-3'teki bileşik manifold bunu ucuzlatır. E7 deneyi bu kararın karşılığını ölçer.

---

## ADR-11 · Odometri ön-ucu yazılmaz, tüketilir

**Karar.** Kendi LiDAR veya görsel odometrimizi yazmıyoruz. Hazır olanların çıktısı
(`nav_msgs/Odometry`) `RelativePose` arayüzünden tüketilir. Referans uygulama: KISS-ICP.

**Gerekçe.** Kendi ön-ucunu yazmak altı hafta alır ve sonuç olgun bir kütüphaneden kötü olur.
Bu projenin katkısı ön-uç değil kestirimcidir. Ayrıca kaynak-agnostik olmak, mimari
iddiasının (ADR-4) doğrudan kanıtıdır.

**Alternatifler.** Kendi ICP'sini yazmak — kapsamı ikiye katlar, tezi bulandırır.

**Sonuçlar.** Demo kurulumunda dış bağımlılık. Karşılığında kapsam disiplini korunur.

---

## ADR-12 · Birincil hedef Humble, CI matrisi Humble + Jazzy

**Karar.** Geliştirme Ubuntu 22.04 + ROS 2 Humble üzerinde yapılır. CI aynı kodu Humble ve
Jazzy üzerinde derler ve test eder.

**Gerekçe.** Humble sektörde en yaygın kurulu sürümdür, ancak desteği Mayıs 2027'de biter.
Tek distro'ya kilitlenmek projeyi kısa sürede eskitir. Çift matris neredeyse bedavadır çünkü
çekirdek zaten ROS'suzdur (ADR-1) — distro'ya değen tek yer ince adaptör katmanıdır.

**Alternatifler.** Yalnızca Humble (kısa ömür); yalnızca Jazzy (mevcut geliştirme ortamıyla uyumsuz).

**Sonuçlar.** C++17 ile sınırlı kalınır (GCC 11). CI süresi iki katına çıkar. Karşılığında
proje ömrü uzar ve bakım bilinci görünür olur.

---
---

# Architecture Freeze — ADR-13…22

> Kod yazılmadan önce yapılan mimari inceleme sonucunda alınan kararlar.
> **ADR-1…12 dokunulmadan durur.** Aşağıdakiler gerektiğinde onları *supersede* eder.

## Supersede dizini

| Yeni | Etki | Eski |
|---|---|---|
| ADR-13 | kurar | — (CONVENTIONS §3 ile INTERFACES arasındaki çelişkiyi kapatır) |
| ADR-14 | **supersede** | ADR-3 (derleme zamanı `CompositeState`); ADR-9'un klon mekanizmasını somutlaştırır |
| ADR-15 | genişletir | ADR-5 (geri sarma) |
| ADR-16 | kurar | — (CONVENTIONS §8.1'i sözleşmeye bağlar) |
| ADR-17 | kurar | — |
| ADR-18 | genişletir | ADR-8 (teşhis) |
| ADR-19 | genişletir | ADR-8 (red sebebi katmanlaması) |
| ADR-20 | düzeltir | ADR-15 (tam snapshot sahipliği backend değil orkestratör) |
| ADR-21 | sınırlar | ADR-2'nin “tanım gereği tutarlı” iddiasının kapsamı |
| ADR-22 | netleştirir | ADR-16 (allocation-free sınırı sayısal hot path) |
| ADR-23 | gereksinim koyar | INTERFACES §6 `compute_protection_level` / `FaultDetector` (Faz 4'te yeniden ele alınır) |

---

## ADR-13 · Jacobian konvansiyonu: `J_res`

**Karar.** Tek Jacobian tanımı residual Jacobian'dır:
`J_res = ∂r(X ⊞ δ)/∂δ |₀`, `r = z ⊖ h(X)`. Ölçüm Jacobian'ı `H = ∂h/∂δ` kullanılmaz ve
kodda `H` adı geçmez. Backend düzeltmesindeki eksi işareti açıkça yazılır:
`δ = −P J_resᵀ S⁻¹ r`.

**Gerekçe.** Üç ayrı sebep aynı yöne işaret ediyor:
(1) Sayısal Jacobian testi residual'ı pertürbe eder, yani zaten `J_res` üretir — `H`
konvansiyonunda test ile üretim arasında sessiz işaret uyuşmazlığı riski doğar.
(2) Manifold değerli ölçümlerde artık `Log(ẑ⁻¹ ∘ h(X))` biçimindedir; ayrı Öklidyen `h`
iyi tanımlı değildir, residual türevi her zaman tanımlıdır.
(3) Öklidyen özel durumda `J_res = −H` ve güncelleme standart EKF'e indirgenir
(`J_res P J_resᵀ = H P Hᵀ`, kuadratik formda işaret sadeleşir).

**Alternatifler.** `H = ∂h/∂δ` — daha yaygın gösterim, ama mevcut test mimarisini bozar ve
manifold ölçümlerde ek tanım gerektirir. `J_r` adı — `manif::…::Jr` (Lie sağ Jacobian'ı) ile
çakışır; bu kod tabanında ikisi de yoğun geçeceği için yasaklandı.

**Sonuçlar.** İşaret hatası riskini kapalı formlu lineer-Gauss testi taşır (CONVENTIONS §9);
bu test zorunludur ve `J_res` yerine `−J_res` yazıldığında düşen tek testtir.

---

## ADR-14 · Sınırlı kapasiteli augmentation deposu · **ADR-3'ü supersede eder**

**Karar.** Durum: sabit 15-DoF çekirdek + `kMaxAugmentDof` ile sınırlı çalışma-anı
augmentation deposu. Depoda **iki ayrı yaşam döngüsü** vardır:

- `PersistentCalibration` — yapılandırmada kaydedilir, oturum boyunca kalır, marjinalleştirilmez
- `Clone` — çalışma anında push/pop, cross-covariance taşır, kullanımdan sonra marjinalleştirilir

Depolama başta `kMaxStateDof` kapasitesiyle ayrılır; `active_dof()` çalışma anında değişir,
heap'te yeniden boyutlandırma yapılmaz.

**Gerekçe.** ADR-3'teki derleme zamanı `CompositeState<Blocks...>` iki sözü aynı anda
tutamıyordu: `INTERFACES` "yeni blok = alias'ı düzenle" (yani recompile) derken
`INTEGRATION` "aynı ikili dosya, farklı YAML" diyordu. Ayrıca klon sayısı doğası gereği
derleme zamanında bilinemez. Ortak depo bu ikisini çözer.

**Neden tek tür değil.** Klonlar geçici ve marjinalleştirilebilir, kalibrasyon durumları
kalıcıdır. Tek tür yapılırsa ya klonlar yapılandırma zamanına hapsolur ya da kalibrasyon
durumları yanlışlıkla marjinalleştirilebilir hale gelir. Ortak *depolama*, ayrı *yaşam döngüsü*.

**Alternatifler.** Tamamen dinamik durum (`MatrixXd`) — CONVENTIONS §8.1'deki tahsis
yasağını ihlal eder. Derleme zamanı varyantlar — "aynı binary" sözünü tutamaz.

**Sonuçlar.** `kMaxAugmentDof` derleme zamanı bir seçimdir; aşılırsa yapılandırma sessizce
kırpmaz, hata verir. ADR-3'ün şablon indeksleme yaklaşımı yerini çalışma-anı ofset tablosuna
bırakır.

---

## ADR-15 · Geri sarma `Snapshot` ile · ADR-5'i genişletir

**Karar.** `reset(x, P)` kaldırıldı. Yerine `save_snapshot()` / `restore_snapshot()`.

**Kapsam kuralı:** *replay sonucunu etkileyen tüm mutable estimator/integrity state
snapshot'a dahildir.* Bugün bu: durum, tam kovaryans, `stamp_ns`, aktif augmentation düzeni
ve ofsetleri, klon kimlikleri ve değerleri, NIS birikimleri, FDI dışlama kararları. İleride
stateful bir bileşen eklenirse **aynı sözleşmeye dahil edilir** — istisna yoktur.

**Gerekçe.** `reset(x, P)` geçerli bir imzaydı ve derlenirdi, ama zamanı geri almadığı için
backend eski durumla yeni zaman damgasında kalıyordu; sonraki `dt` yanlış veya negatif
hesaplanıyordu. Sessiz bozulma. Ayrıca ADR-14 ile klon kümesi de duruma dahil olduğundan
geri sarmanın onu da kapsaması gerekir. FDI dışlama kararı geri sarılmazsa replay
deterministik olmaz.

**Sonuçlar.** Snapshot maliyeti pencere boyutuyla artar. Kapsam kuralı genel yazıldığı için
yeni bileşen eklendiğinde aynı hata tekrar doğmaz.

---

## ADR-16 · Tahsissiz ölçüm çalışma alanı

**Karar.** Sayısal yayılım/güncelleme hot path'i heap tahsisi yapmaz — `predict`, `update`,
`Measurement::evaluate` ve bunların matris/çalışma alanı işlemleri dahil. Kuyruk sahipliği
sınırı ADR-22 ile netleştirilir. Uygulama: sabit kapasiteli Eigen tipleri veya
çağıran tarafından sağlanan `MeasurementWorkspace&`. Değerle dinamik matris döndürmek
(`MatX noise() const` gibi) yasaktır.

**Gerekçe.** CONVENTIONS §8.1 tahsisi zaten yasaklıyordu, ama arayüz bunu garanti etmiyordu.
`MatX noise() const` her çağrıda tahsis eder. Dinamik tipler önceden boyutlandırılıp yeniden
kullanılabilir, fakat sözleşme bunu zorunlu kılmadıkça uygulamaya bırakılmış olur.

**Alternatifler.** Yalnızca `Residual`'ı sabit boyuta geçirmek — daha zayıf örneği çözer,
asıl ihlali (`noise()`) bırakırdı.

**Sonuçlar.** `kMaxResidualDim` üst sınırı gerekir. Ölçüm uygulamaları biraz daha ayrıntılı
olur; karşılığında gerçek zamanlı davranış öngörülebilir ve gömülü hedef açık kalır.

---

## ADR-17 · Joseph formu, simetrizasyon ve explicit inverse yasağı

**Karar.** Kovaryans güncellemesi Joseph formudur, ardından `P ← ½(P + Pᵀ)`. Naif
`(I−KH)P` kullanılmaz. `S⁻¹` dokümanda yazılır, kodda hesaplanmaz — `LDLT` ile çözülür,
`S.inverse()` yasaktır.

**Gerekçe.** Projenin tezi kovaryansın anlamlı olmasıdır; ADR-8 kovaryans koşul sayısını
teşhis çıktısı sayar. Naif form simetriyi ve pozitif tanımlılığı yuvarlama hatasına karşı
korumaz — bu, tezle doğrudan çelişir. Explicit tersleme ise hem sayısal olarak kötü koşullu
hem de gereksiz pahalıdır.

**Alternatifler.** Joseph'i opsiyonel bir bayrak yapmak — "hızlı mod"da tez çöker, savunulamaz.

**Sonuçlar.** Güncelleme başına hesap maliyeti artar. Uzun koşularda kovaryans sağlığı korunur.

---

## ADR-18 · `EstimatorMode` ve gözlemlenebilirlik teşhisi · ADR-8'i genişletir

**Karar.** Backend bir çalışma modu yayınlar:
`kUninitialized → kInitializing → kNominal → kDegraded → kFaulted`.
Ayrıca zayıf gözlemlenebilir yönler (`WeakDirection{label, sigma}`) teşhis olarak bildirilir.

**İsimlendirme:** `EstimatorMode` **sistem** seviyesidir; mevcut `SensorHealth`
(`kNominal/kSuspect/kFaulted`) **tek sensör** seviyesidir. İkisi ayrı tiplerdir ve
birbirinin yerine kullanılmaz.

**Gerekçe.** Tezi "ne kadar güvenebileceğini söyler" olan bir kestirimcinin "henüz
başlatılmadım" veya "yaw şu anda zayıf gözlemlenebilir" diyebilecek bir kanalı olmalıdır.
`bool is_initialized()` bu bilgiyi taşıyamaz: başlatılıyor olmak ile bozulmuş çalışmak
farklı durumlardır.

**Alternatifler.** Tek bir `bool` — yetersiz. Gözlemlenebilirlik Gramian'ının tam analizi —
çalışma anında pahalı; kovaryans öz yapısı üzerinden ucuz vekil yeterlidir.

**Sonuçlar.** `EstimatorMode` Faz 1'de girer (başlatma için gerekli); `WeakDirection`
teşhisi Faz 4'te bütünlük çalışmasıyla birlikte gelir.

---

## ADR-19 · Red sebebi ardışık düzen seviyesindedir · ADR-8'i genişletir

**Karar.** İki ayrı sonuç tipi:
`UpdateResult` — yalnızca filtre güncellemesi (kabul, NIS, dof, eşik).
`ProcessingResult` — ardışık düzenin tamamı + `RejectReason`.

**Gerekçe.** Red sebepleri farklı katmanlara aittir: `kChiSquareGate` backend kararıdır,
`kTooOld` tampon kararıdır, `kSensorExcluded` bütünlük katmanının kararıdır,
`kOriginNotSet` yapılandırma kaynaklıdır. Hepsini backend'in `UpdateResult`'ına yüklemek,
backend'in bilemeyeceği şeyleri biliyormuş gibi göstermek olur ve katman ayrımını bulandırır.

**Alternatifler.** Tek birleşik sonuç tipi — ADR-8'in teşhis ihtiyacını karşılar ama
mimariyi bozar.

**Sonuçlar.** ROS katmanı teşhis mesajını `ProcessingResult`'tan üretir. Backend arayüzü
sensör yönetiminden habersiz kalır.

---

## ADR-20 · Snapshot sahipliği orkestratördedir · ADR-15'i düzeltir

**Karar.** Tam snapshot backend'de değil, `Estimator` orkestratöründedir.

```
Estimator                      ← PipelineSnapshot sahibi
├── FilterBackend              → BackendSnapshot  (NavState[düzen+klonlar], kovaryans, zaman)
├── NisMonitor                 → NisState
└── FaultDetector              → FaultState
```

`FilterBackend::save_snapshot()` yalnızca `BackendSnapshot` döner.
`Estimator::save_snapshot()` üçünü birleştirip `PipelineSnapshot` üretir.
`MeasurementBuffer` `PipelineSnapshot` saklar.

**Gerekçe.** ADR-15'in *kapsamı* doğruydu (replay'i etkileyen tüm mutable state), fakat
*sahipliği* yanlış yerleştirilmişti: `Snapshot` içine `IntegrityState` konarak
`FilterBackend`'in `integrity/` katmanını bilmesi gerekiyordu. Bu, ADR-19'un red sebebi
için tam olarak reddettiği katman karışmasının aynısıdır — aynı hatanın snapshot
tarafındaki tekrarı.

**Alternatifler.** Backend'in integrity'yi sahiplenmesi — katman ayrımını bozar, InEKF/ESKF
backend'lerini sensör yönetimine bağımlı kılar. Snapshot'ı yalnız backend'le sınırlamak —
FDI dışlaması geri sarılmaz, replay deterministik olmaz.

**Sonuçlar.** Yeni bir `Estimator` katmanı doğar; `MeasurementBuffer::process()` artık
backend değil `Estimator` alır. Her stateful bileşen kendi `capture()`/`restore()` çiftini
sağlar; ADR-15'in "istisna yoktur" kuralı bileşen bazında uygulanabilir hale gelir.

---

## ADR-21 · Tutarlılık iddiasının kapsamı

**Karar.** Proje, InEKF'in **tüm genişletilmiş durum için** tanım gereği tutarlı olduğunu
iddia **etmez**. İddia şudur:

> SE₂(3) navigasyon çekirdeğinin değişmez hata yapısı, linearizasyon kaynaklı tutarsızlığı
> azaltmayı hedefler. Genişletilmiş durumun tutarlılığı E1/NEES deneyleriyle **doğrulanır**.

**Gerekçe.** Group-affine özelliği, yer çekimli IMU kinematiği için **bias'sız** SE₂(3)
durumunda geçerlidir. IMU bias'ları eklendiği anda özellik tam olarak korunmaz — literatürde
*imperfect InEKF* denen durum budur. Kalıcı kalibrasyon durumları (ADR-14) ve stochastic
clone'lar (ADR-9) bu yaklaşıklığı daha da genişletir.

Dolayısıyla ADR-2'deki "tutarlılık ayarlamayla değil tanım gereği gelir" ifadesi, bias'ları
zaten içeren bir durum vektörü için fazla güçlüydü — augmentasyon eklenmeden önce de öyleydi.

**Bu projeyi zayıflatmaz, güçlendirir.** Tez zaten "tutarlılığı varsaymıyoruz, ölçüyoruz"dur;
E1 deneyi tam olarak bunun için vardır. Yapısal avantajı varsaymak yerine ölçmek, projenin
kendi metodolojisiyle tutarlı olan tek duruştur.

**Sonuçlar.** `SPEC.md` §3'teki katman adı "Tasarım gereği tutarlı" yerine "Yapısal olarak
daha iyi koşullu" oldu. `docs/theory/03-eskf-vs-inekf.md` group-affine özelliğinin nerede
tam, nerede yaklaşık olduğunu türetmeyle gösterecek. ADR-2'nin kararı (durumu SE₂(3)
üzerinde kurmak) geçerliliğini korur; değişen yalnızca ondan çıkarılan iddianın gücüdür.

---

## ADR-22 · Allocation-free sınırı sayısal hot path'tir · ADR-16'yı netleştirir

**Karar.** Tahsis yasağının zorunlu sınırı, kestirimin sayısal hot path'idir:
`FilterBackend::predict`, `FilterBackend::update`, `Measurement::evaluate` ve bunların çağırdığı
matris/çalışma alanı işlemleri. Bu bölgede heap allocation, runtime resize ve değerle dinamik
matris döndürme yasaktır.

`MeasurementBuffer`'ın olay/ölçüm **sahipliği** (`std::unique_ptr<Measurement>` gibi) bu sınırın
dışındadır. Dolayısıyla genel amaçlı masaüstü/ROS kullanımında kuyruk katmanında tahsis kabul
edilir. Hard-real-time bir taşıma hedeflenirse bu katman object pool veya fixed-capacity variant ile
ayrıca tahsissiz hale getirilebilir; filtre matematiğinin arayüzü değişmez.

**Gerekçe.** ADR-16'nın amacı lineer cebir ve ölçüm değerlendirmesinde öngörülemez tahsisleri
engellemekti. `std::unique_ptr` ile kuyruk sahipliğini aynı yasak içine almak, kapsamı gereksiz yere
type-erasure/pool tasarımına büyütür ve mevcut gömülü-*taşınabilir* hedefini gömülü hard-real-time
zorunluluğuna çevirirdi.

**Alternatifler.** Tüm pipeline'ı tahsissiz yapmak — mümkün ama bugünkü kapsam için gereksiz
karmaşıklık. Tahsis kuralını tamamen kaldırmak — sayısal hot path'in deterministik davranışını bozar.

**Sonuçlar.** `MeasurementWorkspace`, state/covariance ve update ara matrisleri sabit kapasiteli
kalır. Kuyruk sahipliği ayrı performans katmanıdır ve ileride ölçülerek optimize edilir.


---

## ADR-23 · Protection level arıza-farkında olmalıdır · Faz 4 gereksinimi

> **Freeze sonrası eklenmiştir.** ADR-13…22 kod yazılmadan önce donduruldu; bu ADR Faz 0
> sırasında yapılan dış literatür taramasında bulunan bir açığı kayda geçirir. Bağlayıcılığı
> diğerleriyle aynıdır.

**Karar.** Protection level yalnız nominal kovaryanstan türetilemez. Faz 4'te `ProtectionLevel`,
hedef bütünlük riskinde kestirim hatasını **tespit edilmemiş arıza hipotezleri altında da**
sınırlamak zorundadır.

`SPEC.md` §7'deki E6 başarı ölçütü — Stanford diyagramında HMI bölgesinin boş olması —
**daraltılmaz**. Bağlayıcı kalır; uygulamanın ona yükselmesi beklenir.

Bu ADR bir **gereksinim kaydıdır, tasarım değildir.** Arıza hipotezi API'si, solution
separation formülasyonu, alt-çözüm yönetimi ve Faz 4 uygulaması **burada tasarlanmaz.**
Faz 4 açıldığında ayrı bir ADR ile yazılır.

**Gerekçe.** `INTERFACES.md` §6'daki dondurulmuş imza şudur:

```cpp
ProtectionLevel compute_protection_level(const NavCovariance& P, int active_dof,
                                         const FaultDetector& fd,
                                         Scalar target_integrity_risk);
```

`FaultDetector`'ın dışarı verdiği bilgi `SensorHealth{kNominal, kSuspect, kFaulted}` ve
`is_excluded(sensor)` ile sınırlıdır. Yani PL, kovaryans artı "hangi sensör dışlandı"
bilgisiyle hesaplanabilir. Bu, bütünlük literatüründeki **nominal** PL ailesidir: sıfır
ortalamalı Gauss hatası varsayımı altında hatayı sınırlar.

Açık buradadır. Tespit eşiğinin **altında** kalan bir arıza kestirimi kaydırır ama `P`'yi
şişirmez ve sensörü dışlatmaz. Kovaryans tabanlı PL bu durumda hatayı sınırlamaz; gerçek
hata PL'i aşar ve E6'nın HMI bölgesinde nokta belirir. Deney kendi başarı ölçütünü düşürür.

Yerleşik çözüm ailesi bütünlük riskini **her arıza hipotezi üzerinden** sınırlar: tam çözüm
ile arıza-toleranslı alt-çözümler arasındaki ayrım, tespit eşiği ve hipotez önsel olasılıkları
birlikte kullanılır (solution separation / ARAIM hattı; Kalman filtresine uyarlanmış hâli
Arana–Hafez–Joerger–Spenko, IJRR 2020). Üç değerli bir sağlık enum'u bu veriyi taşıyamaz.

**Alternatifler.** *E6'yı nominal koşula daraltmak* — reddedildi. Projenin tezi çalışma anında
hata üst sınırı üretmektir; arızayı kapsam dışı bırakan bir PL, `SPEC.md` §2'de eleştirilen
"aşırı-iyimser kovaryans" problemini bu sefer bütünlük katmanında tekrar üretir.
*Faz 4 API'sini şimdi tasarlamak* — reddedildi. Faz 0'da ölçüm modeli, FDI davranışı ve arıza
enjeksiyon sonuçları yoktur; bugün yazılacak hipotez arayüzü ölçüsüz tahmine dayanır.

**Sonuçlar.** `INTERFACES.md` §6'daki `compute_protection_level` imzası ve `FaultDetector`
arayüzü **bugün değişmez** — Faz 4 açılana kadar dondurulmuş hâlleriyle geçerlidir. Faz 4'e
girerken ikisi de bu ADR ışığında yeniden ele alınır ve gereken genişletme ayrı bir ADR ile
yapılır. Faz 4 planına giriş koşulu olarak şu soru eklenir: *PL'in gördüğü veri, tespit
edilmemiş arıza hipotezlerini sınırlamaya yetiyor mu?*
