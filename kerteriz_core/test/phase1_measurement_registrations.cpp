/// \file
/// Faz 1 GERCEK olcum tiplerinin fabrika kayitlari — CLAUDE.md §6.1 Denetim (4).
///
/// Neden test agacinda: `kerteriz_core` bugun header-only bir INTERFACE
/// hedeftir, yani kaydi tasiyacak bir uretim ceviri birimi YOKTUR. Kayit
/// makrosu ise bir nesne tanimi uretir ve bir .cpp gerektirir. Ilk uretim
/// kaynak dosyasi geldiginde (fabrika geri cagrisi, F1.7+) bu kayitlar oraya
/// tasinacaktir; bugun uydurma bir uretim .cpp'si ACILMAZ.
///
/// Ceviri birimi Jacobian testlerinden AYRIDIR. Denetim (4) ancak kayit ile
/// test AYNI BINARY'de bulustugunda anlamlidir; ayni dosyada toplanirlarsa
/// denetim kendi kendini dogrulamis olur.
///
/// config_name degerleri INTERFACES §3'un kanonik sensor adlaridir. Bu PR'da
/// calisan bir YAML ayristiricisi YOKTUR; adlar fabrika anahtaridir ve
/// benzersizlikleri Denetim (4) tarafindan zorlanir.

#include "kerteriz/measurements/gnss_position.hpp"
#include "kerteriz/measurements/gnss_velocity.hpp"
#include "kerteriz/measurements/non_holonomic.hpp"
#include "kerteriz/measurements/registry.hpp"
#include "kerteriz/measurements/wheel_velocity.hpp"
#include "kerteriz/measurements/zero_velocity.hpp"

namespace kerteriz {

KERTERIZ_REGISTER_MEASUREMENT(GnssPosition, "gnss_position");
KERTERIZ_REGISTER_MEASUREMENT(GnssVelocity, "gnss_velocity");
KERTERIZ_REGISTER_MEASUREMENT(WheelVelocity, "wheel_velocity");
KERTERIZ_REGISTER_MEASUREMENT(NonHolonomic, "non_holonomic");
KERTERIZ_REGISTER_MEASUREMENT(ZeroVelocity, "zero_velocity");

} // namespace kerteriz
