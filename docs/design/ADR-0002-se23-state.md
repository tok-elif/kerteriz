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
