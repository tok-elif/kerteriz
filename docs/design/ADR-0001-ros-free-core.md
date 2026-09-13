## ADR-1 · ROS'suz kestirim çekirdeği

**Karar.** `kerteriz_core` bağımsız bir CMake hedefidir. Bağımlılığı yalnızca Eigen ve manif'tir.
ROS başlık dosyası, ROS mesaj tipi veya `rclcpp` bağımlılığı içeremez. ROS 2 paketi çekirdeği sarar.

**Gerekçe.** Çekirdek ROS kurulmamış bir makinede derlenir, test edilir, profillenir. Testler
saniyeler içinde çalışır. Kod micro-ROS'a, gömülü hedefe, ROS 1 köprüsüne veya toplu işleme
hattına taşınabilir. OpenVINS, GTSAM ve Ceres aynı ayrımı kullanır.

**Alternatifler.** Her şeyi ROS düğümünde toplamak — daha hızlı başlar, test edilemez ve taşınamaz hale gelir.

**Sonuçlar.** Bir dönüşüm katmanı maliyeti doğar (`conversions.cpp`). Karşılığında CI hızlanır
ve taşınabilirlik iddiası kanıtlanabilir olur. CI'da çekirdeği ROS'suz derleyen ayrı bir iş vardır.
