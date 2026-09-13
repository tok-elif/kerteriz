/// \file
/// Sahte olcumlerin FABRIKA KAYITLARI.
///
/// Kasten AYRI bir ceviri birimindedir. Faz 1'de kayit, olcum sinifinin
/// yaninda olacak; Jacobian testi ise test agacinda. Denetim (4) ancak ikisi
/// AYNI BINARY'de bulustugunda anlamlidir — bu ayrim o bulusmayi gercekten
/// sinar. Tek dosyada toplansaydi denetim kendi kendini dogrulamis olurdu.
///
/// Kayitlar tipin KENDI ad alaninda yapilir: makro artik gercek bir tip
/// ifadesi bekler, `SahteGnss` yalnizca burada cozulur. Faz 1'de gercek
/// olcumler icin de dogru yerlesim budur.

#include "fake_measurements.hpp"

#include "kerteriz/measurements/registry.hpp"

namespace kerteriz::test_fakes {

KERTERIZ_REGISTER_MEASUREMENT(SahteGnss, "sahte_gnss");
KERTERIZ_REGISTER_MEASUREMENT(SahteTeker, "sahte_teker");

} // namespace kerteriz::test_fakes
