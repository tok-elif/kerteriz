# F2.4-D · Taban çizgisi kök-neden denetimi

**Bu dosya sonuç tablosu değildir.** `0032` ve `0039` dizilerinde harici
`robot_localization` taban çizgisinin ıraksamasının kaynağını daraltmak için
yapılan denetimin kaydıdır. Denetim sırasında **hiçbir kod veya yapılandırma
değiştirilmedi**; Q, P₀, R, kapı, `sensor_timeout`, eksen konvansiyonu ve RL
ayarı olduğu gibi bırakıldı.

Denetlenen koşu: F2.4-C'de dondurulan protokol — GNSS konum stride 10
(~0.97 Hz), GNSS hızı kapalı, tam hızlı IMU, withheld-only ortak destek APE,
hizalama yok.

## Elenen hipotezler

### 1. `sensor_timeout` ölçüm düşürüyor — **ELENDİ**

Seyrek GNSS aralığı ~1.03 s, dondurulmuş `sensor_timeout` 0.5 s. "Ölçüm zaman
aşımına uğrayıp reddediliyor olabilir" hipotezi **yanlıştır**.

Kurulu paket kanıtı — `share/robot_localization/params/ekf.yaml:9-11`:

> "The period, in seconds, after which we consider a sensor to have timed out.
> In this event, we carry out a predict cycle on the EKF without correcting it.
> This parameter can be thought of as the minimum frequency with which the
> filter will generate new output."

`include/robot_localization/filter_base.hpp:365-371`:

> "The updates to the filter — both predict and correct — are driven by
> measurements. If we get a gap in measurements … we will continue to call
> `predict()` at the filter's frequency."

Yani parametre **ölçüm reddetmez**; yalnızca ölçümsüz geçen sürede düzeltmesiz
bir predict döngüsü tetikler. IMU ~10 Hz aktığı için pratikte hiç devreye
girmez. Aşağıdaki bölüm 4 bunu ampirik olarak da doğrular: GNSS düzeltmeleri
gerçekten uygulanıyor.

### 2. KITTI IMU eksen/işaret konvansiyonu yanlış — **ELENDİ**

Dört dizinin ham OXTS kayıtlarında `x/y/z` ile `f/l/u` alanları
karşılaştırıldı:

| çift | ort \|a−b\| | ort \|a+b\| | korelasyon |
|---|---|---|---|
| `ax`/`af` | 0.047 – 0.108 | 0.64 – 1.19 | 0.993 – 0.999 |
| `ay`/`al` | 0.075 – 0.374 | 0.39 – 0.78 | 0.903 – 0.963 |
| `az`/`au` | 0.002 – 0.015 | 19.6 | 0.999 – 1.000 |
| `wx`/`wf`, `wy`/`wl`, `wz`/`wu` | 0.00004 – 0.0013 | 0.011 – 0.095 | 0.995 – 1.000 |

`|a−b| ≪ |a+b|` ve korelasyon pozitif ~+1 ⇒ `x≈f`, `y≈l`, `z≈u`, **aynı
işaret**. İşaret çevirmesi veya eksen permütasyonu **yoktur**. Kalan küçük
fark, `x/y/z`'nin gövde (roll/pitch eğik) çerçevesinde, `f/l/u`'nun yer-teğet
çerçevesinde olmasıyla tutarlıdır.

Adaptördeki "dönüşüm gerekmiyor" iddiası **veriyle doğrulandı**.

### 3. İvmeölçer yer çekimi içermiyor olabilir — **ELENDİ**

| dizi | `az` medyan | \|a\| medyan | düşük dinamikte \|a\| medyan |
|---|---|---|---|
| 0013 | 9.800 | 9.811 | 9.835 (n=17) |
| 0027 | 9.802 | 9.824 | 9.752 (n=27) |
| 0032 | 9.788 | 9.803 | 9.777 (n=92) |
| 0039 | 9.781 | 9.858 | 9.697 (n=21) |

`|a| ≈ g ≈ 9.81`, `az ≈ +g` (z-yukarı). OXTS `ax/ay/az` **yer çekimi içeren
özgül kuvvettir**; yer çekimi telafi edilmiş ivme olsaydı `|a| ≈ 0` olurdu.
Dolayısıyla `imu0_remove_gravitational_acceleration: true` doğru ayardır ve
Kerteriz'in CONVENTIONS §7 beklentisi de karşılanır.

### 4. GNSS düzeltmesi filtreye ulaşmıyor olabilir — **ELENDİ**

Yayıncının mesajı yayınlaması tek başına füzyon kanıtı değildir, bu yüzden
seçili her GNSS damgasında RL hatasının öncesi/sonrası ölçüldü:

| dizi | örnek (t) | önce | sonra | düşüş | düzeltme oranı |
|---|---|---|---|---|---|
| 0013 | 9.22 s | 0.4285 | 0.0587 | 0.3698 | 0.86 |
| 0027 | 7.14 s | 0.2412 | 0.0207 | 0.2206 | 0.91 |
| 0032 | 38.24 s | 5.1190 | 2.0080 | 3.1110 | 0.61 |
| 0039 | 38.09 s | 27.1353 | 8.0579 | 19.0775 | 0.70 |

Düzeltmeler **gerçekten uygulanıyor** — 0039'da tek güncellemede 19 m. Üstelik
düzeltme *oranı* zamanla çökmüyor (0.65 – 0.78 bandında kalıyor).

## Doğrulanan gözlem

### `robot_localization` yönelimsiz yer çekimi çıkarma

Kaynak kanıtı — `src/ros_filter.cpp:2613-2647` (sürüm 3.5.4):

```cpp
if (::fabs(msg->orientation_covariance[0] + 1) < 1e-9) {
  // Imu message contains no orientation, so we should use orientation
  // from filter state to transform and remove acceleration
  const Eigen::VectorXd & state = filter_.getState();
  stateTmp.setRPY(state(StateMemberRoll), state(StateMemberPitch),
                  state(StateMemberYaw));
  ...
}
```

Ardından `(0, 0, g)` vektörü bu yönelimin taban matrisinin tersiyle döndürülüp
ölçülen ivmeden çıkarılır.

Yayıncımız `orientation_covariance[0] = -1` gönderdiği için **tam olarak bu
dal** çalışır. Sonuç: yer çekimi çıkarmanın doğruluğu **filtrenin kendi
yönelim kestirimine bağlıdır**. Dondurulmuş yapılandırmada RL'ye hiçbir
yönelim ölçümü verilmez (`imu0_config` yönelim alanları `false`, `odom0` poz
yönelimi `false`); roll/pitch/yaw yalnızca `initial_state`'ten jiro
entegrasyonuyla ilerler.

## Büyüyen sorun: güncellemeler arası ölü-hesap sürüklenmesi

Hız **büyüklüğü** (çerçeveden bağımsız) OXTS'e karşı:

| dizi | RL ort hata | RL maks | Kerteriz ort hata | son \|v\| (RL / gerçek) |
|---|---|---|---|---|
| 0013 | 0.118 m/s | 0.368 | 0.043 | 13.70 / 13.37 |
| 0027 | 0.089 m/s | 0.283 | 0.030 | 21.94 / 21.97 |
| 0032 | 1.362 m/s | 4.681 | 0.069 | 15.48 / 15.54 |
| 0039 | 5.617 m/s | 32.628 | 0.066 | **22.81 / 3.04** |

Düzeltme oranı sabit kaldığına göre (bölüm 4), büyüyen şey **güncellemeler
arasındaki ölü-hesap sürüklenmesidir**. Hız büyüklüğü çerçeveden bağımsız
olduğu için bu bir eksen/çerçeve sorunu **değildir** (bölüm 2 ile de elendi).

### Mekanizma adayı — İZOLE EDİLMEDİ

Yönelim sürüklenmesi → artık yer çekimi sızıntısı → hıza ve konuma çift
entegrasyon. δ rad'lık bir yönelim hatası yaklaşık `g·δ` kadar artık yatay
ivme bırakır.

Bu, doğrulanan gözlemle (yer çekimi çıkarma filtrenin kendi yönelimine bağlı,
yönelim ise hiç ölçülmüyor) **tutarlıdır**, ancak **nedensellik izole
edilmemiştir**. Ayrıca ıraksama sıralaması dönme dinamiğiyle birlikte gidiyor:

| dizi | süre | \|w\| maks | RL RMSE (withheld) |
|---|---|---|---|
| 0027 | 19.4 s | 0.035 rad/s | 0.162 m |
| 0013 | 14.8 s | 0.072 rad/s | 0.164 m |
| 0032 | 40.3 s | 0.112 rad/s | 2.043 m |
| 0039 | 40.7 s | 0.452 rad/s | 8.907 m |

Süre ve dönme dinamiği aynı yönde arttığı için **dört diziyle bu ikisi
ayrıştırılamaz**.

## Hükümler

* Kerteriz ve değerlendirme altyapısında **correctness bug bulunmadı**: ölçüm
  damga kümeleri iki kestirimcide birebir eşit, ölçüm ve withheld kümeleri
  ayrık, her iki kestirimci aynı destekte puanlanıyor, `evo` çapraz kontrolü
  dört dizide iç ATE ile ≤1.8e-15 uyumlu, üç RL tekrarı arasındaki yayılım
  ≤4.4e-11.
* **Mevcut dondurulmuş RL yapılandırması, seyrek ve uzun dizilerde adil bir
  "hangi kestirimci daha doğru" kıyası olarak yorumlanmamalıdır.** RL'den,
  hiç ölçülmeyen kendi yönelimine dayanarak yer çekimi çıkarması ve tek
  mutlak kaynağı ~0.97 Hz'e seyreltilmiş halde ölü-hesap yapması isteniyor.
  Bu bir kod hatası değil, deney kurulumunun sonucudur.
* **Protokol post-hoc DEĞİŞTİRİLMEDİ.** Sonuçlar görüldükten sonra hiçbir
  parametre ayarlanmadı; bu denetim yalnızca gözlemi açıklamak içindir.
* `2011_09_26_drive_0032_sync` **yardımcı dizi olarak kalır**; ön kayıtlı
  altılının parçası değildir ve ön kayıtlı toplu sonuca katılmaz
  (bkz. `sequence_manifest.md`).

## Bilimsel sınır

Bu denetimdeki ve F2.4'teki tüm sayılar, referans ile filtreye verilen GNSS'in
**aynı OXTS/INS çözüm ailesinden** gelmesi kısıtını taşır. Seyreltme deneyi
doğrudan-ölçüm-noktasında-değerlendirme karışmasını azaltır; bağımsız ground
truth **sağlamaz**.

`same OXTS family != independent ground truth`
