"""E1 çizim katmanının deterministik testleri.

PNG PİKSELLERİ KARŞILAŞTIRILMAZ. Sınanan şey CSV sözleşmesi ve şeklin bir
fixture'dan gerçekten üretilebilmesidir.

NEDEN unittest.TestCase: colcon bu ament_python paketi için
`python3 -m unittest` çalıştırır (pytest DEĞİL).
"""

import os
import tempfile
import unittest

from kerteriz_eval.e1_plot import (
    ANEES_COLUMNS,
    ANIS_COLUMNS,
    CsvFormatError,
    read_series,
)

ANEES_FIXTURE = (
    "timestamp_ns,time_s,mean_nees,expected_mean,lower_95,upper_95,"
    "valid_count,invalid_count\n"
    "200000000,0.2,14.5,15.0,13.1,16.9,500,0\n"
    "400000000,0.4,15.5,15.0,13.1,16.9,500,0\n"
    "600000000,0.6,18.2,15.0,13.1,16.9,500,0\n"
)

ANIS_FIXTURE = (
    "timestamp_ns,time_s,mean_nis,expected_mean,lower_95,upper_95,"
    "valid_count,undefined_count,total_dof\n"
    "200000000,0.2,2.9,3.0,2.2,3.9,500,0,1500\n"
    "400000000,0.4,3.1,3.0,2.2,3.9,500,0,1500\n"
    "600000000,0.6,3.4,3.0,2.2,3.9,500,0,1500\n"
)


def _write(directory, name, text):
    path = os.path.join(directory, name)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(text)
    return path


class CsvOkumaTest(unittest.TestCase):
    def test_gecerli_anees_csv_okunur(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(tmp, "anees.csv", ANEES_FIXTURE)
            series = read_series(path, ANEES_COLUMNS)
            self.assertEqual(len(series["time_s"]), 3)
            self.assertAlmostEqual(series["mean_nees"][2], 18.2)
            self.assertAlmostEqual(series["expected_mean"][0], 15.0)

    def test_gecerli_anis_csv_okunur(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(tmp, "anis.csv", ANIS_FIXTURE)
            series = read_series(path, ANIS_COLUMNS)
            self.assertEqual(len(series["time_s"]), 3)
            self.assertAlmostEqual(series["total_dof"][0], 1500.0)

    def test_eksik_sutun_hatadir(self):
        bozuk = ANEES_FIXTURE.replace("lower_95,", "")
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(tmp, "bozuk.csv", bozuk)
            with self.assertRaises(CsvFormatError) as ctx:
                read_series(path, ANEES_COLUMNS)
            self.assertIn("lower_95", str(ctx.exception))

    def test_sayi_olmayan_alan_hatadir(self):
        bozuk = ANEES_FIXTURE.replace("14.5", "yok")
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(tmp, "bozuk.csv", bozuk)
            with self.assertRaises(CsvFormatError):
                read_series(path, ANEES_COLUMNS)

    def test_alan_sayisi_uyusmazligi_hatadir(self):
        bozuk = ANEES_FIXTURE + "800000000,0.8,1.0\n"
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(tmp, "bozuk.csv", bozuk)
            with self.assertRaises(CsvFormatError):
                read_series(path, ANEES_COLUMNS)

    def test_bos_dosya_hatadir(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(tmp, "bos.csv", "")
            with self.assertRaises(CsvFormatError):
                read_series(path, ANEES_COLUMNS)

    def test_yalniz_baslik_hatadir(self):
        only_header = ANEES_FIXTURE.splitlines()[0] + "\n"
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(tmp, "baslik.csv", only_header)
            with self.assertRaises(CsvFormatError):
                read_series(path, ANEES_COLUMNS)


def _matplotlib_var():
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        return False
    return True


class SekilTest(unittest.TestCase):
    @unittest.skipUnless(_matplotlib_var(), "matplotlib kurulu degil")
    def test_fixture_den_sekil_uretilir(self):
        from kerteriz_eval.e1_plot import make_figure

        with tempfile.TemporaryDirectory() as tmp:
            anees = read_series(_write(tmp, "a.csv", ANEES_FIXTURE), ANEES_COLUMNS)
            anis = read_series(_write(tmp, "n.csv", ANIS_FIXTURE), ANIS_COLUMNS)
            out = os.path.join(tmp, "sekil.png")
            make_figure(anees, anis, out, runs=500)
            self.assertTrue(os.path.exists(out))
            # Pikseller karsilastirilmaz; dosyanin bos olmadigi yeterlidir.
            self.assertGreater(os.path.getsize(out), 1000)


if __name__ == "__main__":
    unittest.main()
