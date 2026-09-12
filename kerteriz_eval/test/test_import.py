"""kerteriz_eval duman testi.

Gercek degerlendirme testleri Faz 2'de gelir (SPEC.md §7). Bu dosya paketin
import edilebildigini dogrular ve paketi "testi olan" hale getirir.

NEDEN unittest.TestCase, cikplak pytest fonksiyonu degil:
colcon bu ament_python paketi icin `python3 -m unittest -v` calistirir
(pytest DEGIL). unittest ne cikplak fonksiyonlari toplar ne de icinde
__init__.py olmayan dizinlere iner; bu yuzden test/__init__.py de vardir.

Testsiz kalirsa Jazzy'de `colcon test` exit 5 ile duser (Humble sessizce
gecer) — ilk CI kosusunda tam olarak bu yasandi.
"""

import unittest

import kerteriz_eval


class PaketImportTest(unittest.TestCase):
    def test_paket_import_edilebilir(self):
        self.assertIsNotNone(kerteriz_eval)
