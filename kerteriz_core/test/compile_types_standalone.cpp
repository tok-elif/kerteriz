/// \file
/// S5 DoD: types.hpp TEK BASINA derlenir.
///
/// Bu ceviri birimi BILEREK yalnizca types.hpp icerir — gtest yok, baska
/// kerteriz basligi yok. Amaci calismak degil DERLENMEK: types.hpp bir gun
/// baska bir kerteriz basligina baglanirsa bu dosya derlenmeyi birakir.
///
/// Denetim (2)'nin static_assert'leri de burada, gtest'ten bagimsiz olarak
/// degerlendirilir.

#include "kerteriz/types.hpp"

namespace {

// Sabitlerin ve tiplerin bu ceviri biriminde gercekten olustugunu zorlar.
constexpr int kDofToplami = kerteriz::kCoreDof + kerteriz::kMaxAugmentDof;
static_assert(kDofToplami == kerteriz::kMaxStateDof);

using Durum = kerteriz::StateMat;
static_assert(Durum::RowsAtCompileTime == kerteriz::kMaxStateDof);

} // namespace
