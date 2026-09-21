"""Yörünge değerlendirme yardımcılarının deterministik testleri.

Sınanan şey ZAMAN ESLEŞTIRME SÖZLEŞMESİ ve TUM dışa aktarımıdır; hiçbir
filtre veya doğruluk iddiası yoktur.

NEDEN unittest.TestCase: colcon bu ament_python paketi için
`python3 -m unittest` çalıştırır (pytest DEĞİL).
"""

import math
import os
import tempfile
import unittest

from kerteriz_eval.trajectory_eval import (
    DEFAULT_TOLERANCE_NS,
    Sample,
    TrajectoryFormatError,
    associate,
    common_support,
    read_trajectory_csv,
    translational_ate,
    write_tum_translation_only,
)

BASLIK = "timestamp_ns,px,py,pz,vx,vy,vz\n"


def _yaz(directory, name, text):
    path = os.path.join(directory, name)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text)
    return path


def _csv(satirlar, baslik=BASLIK):
    return baslik + "".join(satirlar)


def _s(t, x=0.0, y=0.0, z=0.0):
    return Sample(t, (x, y, z))


class CsvOkumaTest(unittest.TestCase):
    def test_gecerli_csv_okunur_ve_siralanir(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = _yaz(tmp, "a.csv", _csv([
                "200,2.0,0.0,0.0,0,0,0\n",
                "100,1.0,0.0,0.0,0,0,0\n",
            ]))
            s = read_trajectory_csv(p)
            self.assertEqual([x.stamp_ns for x in s], [100, 200])
            self.assertAlmostEqual(s[0].position[0], 1.0)

    def test_eksik_sutun_hatadir(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = _yaz(tmp, "a.csv", _csv(["100,1,2,3\n"], "timestamp_ns,px,py\n"))
            with self.assertRaises(TrajectoryFormatError) as ctx:
                read_trajectory_csv(p)
            self.assertIn("pz", str(ctx.exception))

    def test_nan_ve_inf_reddedilir(self):
        for bozuk in ("nan", "inf", "-inf"):
            with tempfile.TemporaryDirectory() as tmp:
                p = _yaz(tmp, "a.csv", _csv(["100,1.0,%s,0.0,0,0,0\n" % bozuk]))
                with self.assertRaises(TrajectoryFormatError):
                    read_trajectory_csv(p)

    def test_damga_tamsayi_olmali(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = _yaz(tmp, "a.csv", _csv(["100.5,1.0,0.0,0.0,0,0,0\n"]))
            with self.assertRaises(TrajectoryFormatError):
                read_trajectory_csv(p)

    def test_yinelenen_damga_hatadir(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = _yaz(tmp, "a.csv", _csv([
                "100,1.0,0.0,0.0,0,0,0\n",
                "100,2.0,0.0,0.0,0,0,0\n",
            ]))
            with self.assertRaises(TrajectoryFormatError):
                read_trajectory_csv(p)

    def test_alan_sayisi_uyusmazligi_hatadir(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = _yaz(tmp, "a.csv", _csv(["100,1.0,0.0\n"]))
            with self.assertRaises(TrajectoryFormatError):
                read_trajectory_csv(p)

    def test_bos_dosya_ve_yalniz_baslik_hatadir(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(TrajectoryFormatError):
                read_trajectory_csv(_yaz(tmp, "b.csv", ""))
            with self.assertRaises(TrajectoryFormatError):
                read_trajectory_csv(_yaz(tmp, "c.csv", BASLIK))


class EslestirmeTest(unittest.TestCase):
    def test_en_yakin_damga_secilir(self):
        ref = [_s(1000), _s(2000)]
        est = [_s(1001, 1.0), _s(1900, 2.0), _s(2050, 3.0)]
        m, eksik = associate(ref, est, tolerance_ns=100)
        self.assertEqual(eksik, 0)
        self.assertEqual(m[1000].stamp_ns, 1001)
        self.assertEqual(m[2000].stamp_ns, 2050)  # 50 < 100

    def test_esitlikte_erken_damga_kazanir(self):
        # 900 ve 1100, 1000'e esit uzaklikta. Kural: ERKEN olan.
        ref = [_s(1000)]
        est = [_s(900, -1.0), _s(1100, +1.0)]
        m, _ = associate(ref, est, tolerance_ns=200)
        self.assertEqual(m[1000].stamp_ns, 900)
        # Girdi sirasindan bagimsiz olmali.
        m2, _ = associate(ref, list(reversed(est)), tolerance_ns=200)
        self.assertEqual(m2[1000].stamp_ns, 900)

    def test_tolerans_siniri_kapsayicidir(self):
        ref = [_s(1000)]
        est = [_s(1020)]
        self.assertEqual(len(associate(ref, est, tolerance_ns=20)[0]), 1)
        self.assertEqual(len(associate(ref, est, tolerance_ns=19)[0]), 0)

    def test_eslesmeyen_ornek_sayilir_ve_kaydirilmaz(self):
        ref = [_s(1000), _s(5000)]
        est = [_s(1000, 1.0)]
        m, eksik = associate(ref, est, tolerance_ns=20)
        self.assertEqual(eksik, 1)
        self.assertNotIn(5000, m)  # sessizce en yakina kaydirilmadi

    def test_bir_kestirim_ornegi_birden_fazla_referansa_eslesebilir(self):
        # C++ ATE uygulamasi da ornekleri TUKETMEZ; davranis kasten aynidir.
        # Farkli olsaydi iki uygulamanin sayilari sistematik ayrisirdi.
        ref = [_s(1000), _s(1010)]
        est = [_s(1005, 7.0)]
        m, eksik = associate(ref, est, tolerance_ns=20)
        self.assertEqual(eksik, 0)
        self.assertIs(m[1000], m[1010])

    def test_varsayilan_tolerans_yirmi_milisaniye(self):
        self.assertEqual(DEFAULT_TOLERANCE_NS, 20_000_000)

    def test_negatif_tolerans_hatadir(self):
        with self.assertRaises(ValueError):
            associate([_s(0)], [_s(0)], tolerance_ns=-1)


class OrtakDestekTest(unittest.TestCase):
    def test_farkli_destekli_kestirimciler_ortak_kumeye_duser(self):
        # A referansin ILK ikisini, B SON ikisini kapsiyor -> ortak yalniz orta.
        ref = [_s(100), _s(200), _s(300)]
        a = [_s(100, 1.0), _s(200, 1.0)]
        b = [_s(200, 2.0), _s(300, 2.0)]
        d = common_support(ref, a, b, tolerance_ns=10)
        self.assertEqual(d["stamps"], [200])
        self.assertEqual(len(d["match_a"]), 2)
        self.assertEqual(len(d["match_b"]), 2)
        self.assertEqual(d["unmatched_a"], 1)
        self.assertEqual(d["unmatched_b"], 1)

    def test_ortak_destek_puanlamayi_degistirir(self):
        # MUTASYON KONTROLU: kestirimcileri KENDI kumelerinde puanlamak farkli
        # sayi verir. Ortak destek kullanilmazsa bu test duser.
        ref = [_s(100), _s(200), _s(300)]
        # Ortak damgada hatalar 1.0 ve 3.0; ortak DISI ornekler kasten
        # SIMETRIK DEGIL, yoksa iki oran tesadufen esitlenir ve mutasyon
        # kontrolu bos gecerdi.
        a = [_s(100, 2.0), _s(200, 1.0)]
        b = [_s(200, 3.0), _s(300, 0.0)]
        d = common_support(ref, a, b, tolerance_ns=10)
        ortak = d["stamps"]
        self.assertEqual(ortak, [200])

        ortak_a = translational_ate(ref, d["match_a"], ortak)["rmse_m"]
        ortak_b = translational_ate(ref, d["match_b"], ortak)["rmse_m"]
        kendi_a = translational_ate(ref, d["match_a"])["rmse_m"]
        kendi_b = translational_ate(ref, d["match_b"])["rmse_m"]

        self.assertAlmostEqual(ortak_a, 1.0)
        self.assertAlmostEqual(ortak_b, 3.0)
        # Kendi kumelerinde puanlama BASKA bir oran verir. Arac ortak destek
        # yerine her kestirimciyi kendi kumesinde puanlasaydi bu test duserdi.
        self.assertNotAlmostEqual(kendi_a / kendi_b, ortak_a / ortak_b, places=3)
        self.assertAlmostEqual(ortak_a / ortak_b, 1.0 / 3.0)


class AteTest(unittest.TestCase):
    def test_bilinen_hatalarda_rmse_ve_maks(self):
        ref = [_s(100), _s(200), _s(300)]
        est = [_s(100, 3.0), _s(200, 0.0), _s(300, 4.0)]
        m, _ = associate(ref, est, tolerance_ns=10)
        r = translational_ate(ref, m)
        self.assertAlmostEqual(r["rmse_m"], math.sqrt((9.0 + 0.0 + 16.0) / 3.0))
        self.assertAlmostEqual(r["max_m"], 4.0)
        self.assertEqual(r["count"], 3)

    def test_hizalama_uygulanmaz(self):
        # Sabit bir ofset TAMAMEN hataya girer; origin hizalamasi olsaydi
        # sifirlanirdi.
        ref = [_s(100), _s(200)]
        est = [_s(100, 5.0), _s(200, 5.0)]
        m, _ = associate(ref, est, tolerance_ns=10)
        self.assertAlmostEqual(translational_ate(ref, m)["rmse_m"], 5.0)


class TumDisaAktarimTest(unittest.TestCase):
    def test_bicim_ve_saniye_donusumu(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "t.tum")
            write_tum_translation_only(p, [_s(1_500_000_000, 1.0, 2.0, 3.0)])
            with open(p, encoding="utf-8") as handle:
                satir = handle.read().strip().split()
            self.assertEqual(len(satir), 8)
            self.assertAlmostEqual(float(satir[0]), 1.5, places=9)
            self.assertAlmostEqual(float(satir[1]), 1.0)
            self.assertAlmostEqual(float(satir[2]), 2.0)
            self.assertAlmostEqual(float(satir[3]), 3.0)
            # Birim kuaterniyon (qx qy qz qw) — GERCEK YONELIM DEGILDIR.
            self.assertEqual([float(x) for x in satir[4:]], [0.0, 0.0, 0.0, 1.0])

    def test_sira_korunur_ve_artan_olmali(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "t.tum")
            write_tum_translation_only(p, [_s(100, 1.0), _s(200, 2.0), _s(300, 3.0)])
            with open(p, encoding="utf-8") as handle:
                zamanlar = [float(l.split()[0]) for l in handle]
            self.assertEqual(zamanlar, sorted(zamanlar))

            with self.assertRaises(TrajectoryFormatError):
                write_tum_translation_only(
                    os.path.join(tmp, "u.tum"), [_s(300), _s(100)]
                )

    def test_konum_degistirilmez(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = os.path.join(tmp, "t.tum")
            deger = -123.456789012
            write_tum_translation_only(p, [_s(10, deger, deger, deger)])
            with open(p, encoding="utf-8") as handle:
                alan = handle.read().split()
            for k in (1, 2, 3):
                self.assertAlmostEqual(float(alan[k]), deger, places=9)


if __name__ == "__main__":
    unittest.main()
