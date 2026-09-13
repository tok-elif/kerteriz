/// \file
/// Sahte olcumlerin FABRIKA KAYITLARI.
///
/// Kasten AYRI bir ceviri birimindedir. Faz 1'de kayit, olcum sinifinin
/// yaninda olacak; Jacobian testi ise test agacinda. Denetim (4) ancak ikisi
/// AYNI BINARY'de bulustugunda anlamlidir — bu ayrim o bulusmayi gercekten
/// sinar. Tek dosyada toplansaydi denetim kendi kendini dogrulamis olurdu.

#include "fake_measurements.hpp"

#include "kerteriz/measurements/registry.hpp"

KERTERIZ_REGISTER_MEASUREMENT(SahteGnss, "sahte_gnss");
KERTERIZ_REGISTER_MEASUREMENT(SahteTeker, "sahte_teker");
