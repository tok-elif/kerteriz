# Başka bir AI'a devretme

> Bu proje birden fazla AI ajanı tarafından, farklı oturumlarda geliştirilecek şekilde
> dokümante edildi. Bu dosya devretme protokolüdür.

---

## 1. Okuma sırası

Bir ajan işe başlamadan önce, bu sırayla:

| # | Dosya | Neden |
|---|---|---|
| 1 | `CLAUDE.md` | Kurallar, aktif faz, kapsam sınırları |
| 2 | `docs/SPEC.md` | Projenin kendi kendine yeten tam tanımı |
| 3 | `docs/CONVENTIONS.md` | **Kod yazmadan önce zorunlu.** Çerçeve, kuaterniyon, pertürbasyon, birim |
| 4 | `docs/INTERFACES.md` | Uygulanacak sözleşme |
| 5 | `docs/design/DECISIONS.md` | "Neden böyle?" sorusunun cevabı |
| 6 | `docs/INTEGRATION.md` | Yalnızca yeni platform/sensör ekliyorsa |

`CLAUDE.md`'yi destekleyen araçlarda (Claude Code) 1. adım otomatiktir. Diğer araçlarda
aşağıdaki başlangıç istemini kullan.

---

## 2. Yapıştırılabilir başlangıç istemi

> Aşağıdaki metni yeni bir AI oturumuna olduğu gibi yapıştır, ardından repo dosyalarını ver.

```
Kerteriz adlı bir ROS 2 durum kestirim projesinde çalışıyorsun.

ÖNCE ŞUNLARI OKU, bu sırayla: CLAUDE.md, docs/SPEC.md, docs/CONVENTIONS.md,
docs/INTERFACES.md, docs/design/DECISIONS.md

MİMARİ DONDURULMUŞTUR (ADR-13…22). Aşağıdakiler tercih değil, sözleşmedir.

BOZULMAZ KURALLAR:
1. kerteriz_core ASLA ROS'a bağımlı olmaz. rclcpp başlığı, ROS mesaj tipi veya
   rclcpp CMake bağımlılığı core içinde geçemez.
2. Her J_res'in sayısal türev testi olur. Testsiz Jacobian yazma.
3. docs/CONVENTIONS.md'deki konvansiyonlar sabittir: ENU dünya çerçevesi,
   Hamilton kuaterniyonu, SAĞ pertürbasyon, SI birimleri.
   ZAMAN: mutlak zaman int64 nanosaniye (TimeNs); süre/dt double SANİYE,
   yalnız iki TimeNs farkından hesaplanır. İkisini karıştırma.
   C++17 hedefi sabittir: `std::span` KULLANMA; `ArrayView` kullan.
4. JACOBIAN: yalnızca residual Jacobian vardır.
   J_res = ∂r(X ⊞ δ)/∂δ,  r = z ⊖ h(X).
   "H" adını KULLANMA. "J_r" de yazma — Jr, Lie sağ Jacobian'ıdır (manif::...::Jr).
   Düzeltmede eksi işareti vardır:  δ = −P J_resᵀ S⁻¹ r
5. KOVARYANS: Joseph formu + P ← ½(P+Pᵀ) ZORUNLUDUR. Naif (I−KH)P yazma.
   S.inverse() YASAK — S.ldlt().solve(...) kullan.
6. DURUM: 15-DoF sabit çekirdek + sınırlı kapasiteli augmentation.
   İki ayrı yaşam döngüsü: PersistentCalibration (kalıcı) ve Clone (geçici).
   Karıştırma. Heap'te runtime resize yok; active_dof() değişir, kapasite değişmez.
7. TAHSİS: sayısal hot path (`predict/update/evaluate`) tahsissiz. Değerle dinamik matris
   DÖNDÜRME (`MatX noise() const` gibi). `MeasurementWorkspace&` doldur. Ölçüm kuyruğunun
   `unique_ptr` sahipliği ADR-22 gereği bu sınırın dışındadır.
8. GERİ SARMA: reset(x,P) YOK. save_snapshot()/restore_snapshot().
   Tam snapshot `Estimator::PipelineSnapshot`'ındadır; backend yalnız `BackendSnapshot` taşır.
   Replay sonucunu etkileyen HER mutable state snapshot'a dahildir — durum, kovaryans, zaman,
   augmentation düzeni, klonlar, NIS birikimi, FDI dışlamaları. Yeni stateful bileşen eklersen
   snapshot'a da eklersin.
9. Yeni sensör = yeni Measurement sınıfı + YAML. Backend kodunu değiştirme.
10. Platforma özel hiçbir sayıyı koda gömme. Hepsi YAML'dan gelir.
11. Önce simülasyon, sonra gerçek veri. Her filtre gerçek-veri adaptöründen
    ÖNCE sentetik veride doğrulanır.
12. InEKF için "tüm genişletilmiş durum tanım gereği tutarlı" İDDİASI YAPMA. Exact
    group-affine/log-linear sonuç bias'sız SE₂(3) çekirdeğine aittir; genişletilmiş durumun
    tutarlılığı NEES/NIS ile ölçülür (ADR-21).
13. CLAUDE.md'deki "Şu an ne yapıyoruz" bölümünün kapsamı dışına ÇIKMA.
    Sonraki fazın işini şimdi yapma.

ÇALIŞMA BİÇİMİ:
- Kod yazmadan önce hangi dosyaların değişeceğini söyle.
- Bir şey belirsizse varsayım yapma, sor.
- Bitirdiğinde CLAUDE.md §6'daki dört doğrulama komutunun hepsini çalıştır.
- Conventional commits kullan.

ÖNCE NE YAPACAĞINI ÖZETLE, ONAY BEKLE, SONRA YAZ.
```

---

## 3. Devretme sırasında güncellenecekler

Bir faz bitip iş el değiştirdiğinde:

- [ ] `CLAUDE.md` §3 "Şu an ne yapıyoruz" — aktif faz, kapsam içi/dışı, DoD listesi
- [ ] Yeni bir mimari karar alındıysa `docs/design/DECISIONS.md`'ye ADR ekle
- [ ] Arayüz değiştiyse **önce** `docs/INTERFACES.md`, sonra kod
- [ ] Yeni konvansiyon gerektiyse `docs/CONVENTIONS.md`
- [ ] `results/` güncel mi

**En sık yapılan hata:** kodu değiştirip dokümanı güncellememek. Bir sonraki ajan
eski sözleşmeye göre kod yazar ve sessizce uyumsuzluk üretir.
Arayüz değişikliğinde doküman kodla aynı PR'da güncellenir.

---

## 4. Bir ajanın çıktısını denetleme

Üretilen kodu kabul etmeden önce:

| Kontrol | Nasıl |
|---|---|
| R1 ihlali | `grep -rE "rclcpp\|_msgs/" kerteriz_core/` → boş dönmeli |
| `H` sızması | `grep -nE "\bH\b.*Jacobian\|MatX H" kerteriz_core/` → boş. Yalnız `J_res` olmalı |
| Explicit inverse | `grep -rn "\.inverse()" kerteriz_core/` → kovaryans/innovation yolunda olmamalı |
| Naif kovaryans | Güncellemede `(I - K*H) * P` deseni var mı → Joseph olmalı |
| Simetrizasyon | Güncelleme sonrası `0.5 * (P + P.transpose())` var mı |
| Tahsis | Değerle dönen `MatX`/`VecX` var mı → `MeasurementWorkspace&` olmalı |
| Zaman karışması | `double` tutulan **timestamp** var mı (`dt`'nin double olması doğrudur) |
| C++17 uyumu | `std::span` veya başka C++20 API var mı → `ArrayView`/C++17 eşdeğeri kullan |
| Snapshot bütünlüğü | Yeni stateful alan eklendiyse `Snapshot`'a da eklenmiş mi |
| Jacobian testi | Registry'de testsiz `Measurement` var mı (test paketi düşmeli) |
| Kapsam taşması | Aktif fazın dışındaki dosyalar değişmiş mi |
| Core ROS'suz derleniyor mu | `cmake -S kerteriz_core -B build/core && cmake --build build/core` |

İlk altısı `grep` ile otomatikleştirilebilir. Ama **asıl savunma kapalı formlu
lineer-Gauss testidir** (`CLAUDE.md` §6.1, denetim 3): işaret hatası grep'le değil,
o testle yakalanır.

---

## 5. Neyi devretme

Bazı işler ajana devredilmemeli, çünkü hata sessiz ve pahalıdır:

- **Konvansiyon seçimi.** Kuaterniyon sırası, pertürbasyon tarafı, çerçeve tanımı —
  bunlar bir kere insan tarafından sabitlenir.
- **Gürültü parametreleri.** Allan varyansı ölçümünden gelir, tahminden değil.
- **Sonuçların yorumu.** NEES bandın dışındaysa sebebini anlamak senin işin;
  "bandı genişletelim" diyen bir öneriyi kabul etme.
- **Kapsam kararları.** Yeni özellik eklemek faz planını değiştirir; bu senin kararın.
