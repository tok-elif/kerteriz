"""kerteriz_eval duman testi.

Gercek degerlendirme testleri Faz 2'de gelir (SPEC.md §7). Bu dosya paketin
import edilebildigini dogrular ve ayni zamanda paketi "testi olan" hale getirir:
pytest hic test toplayamazsa exit 5 dondurur ve colcon bunu hata sayar
(Jazzy'deki pytest surumunde bu davranis Humble'dan farklidir).
"""

import kerteriz_eval


def test_paket_import_edilebilir():
    assert kerteriz_eval is not None
