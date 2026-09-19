"""Yörünge değerlendirme yardımcıları — ortak zaman desteği ve TUM dışa aktarımı.

Bu modül **filtre matematiğine dokunmaz**. İşi, iki kestirimciyi *aynı* zaman
örnekleri üzerinde karşılaştırılabilir hale getirmek ve harici bir araca
(`evo`) geçerli girdi üretmektir.

Zaman eşleştirme sözleşmesi
---------------------------
C++ tarafındaki ``kerteriz_bringup/ate.hpp`` ile **kasten aynıdır**; iki
uygulamanın aynı şeyi ölçtüğünü söyleyebilmek için:

* her **referans** örneği için damgası en yakın kestirim örneği seçilir,
* eşitlikte **daha erken** damga kazanır (deterministik, girdi sırasından
  bağımsız),
* fark toleransı aşarsa örnek **eşleşmez** ve ortalamaya girmez — sessizce
  kaydırılmaz,
* **enterpolasyon yapılmaz**; yapılsaydı hangi modelin seçildiği sonucu
  etkilerdi ve bu belgelenmemiş bir serbestlik olurdu,
* bir kestirim örneği **birden fazla** referans örneğine eşleşebilir. C++
  uygulaması da örnekleri "tüketmez"; davranış burada da aynıdır ve testlidir.
  Farklı olsaydı iki uygulamanın sayıları sistematik olarak ayrışırdı.

Varsayılan tolerans 20 ms'dir — C++ koşucusunun ``ate_max_dt_ns`` varsayılanı.

Ortak destek — neden gerekli
----------------------------
İki kestirimci referansın *farklı* alt kümelerini kapsayabilir (Faz 1'de biri
``ref[1:]``, diğeri ``ref[:-1]`` idi). Aynı sayı ikisini de "143 eşleşme"
gösterse bile puanlandıkları küme farklıdır. Buradaki ortak destek, **her iki**
kestirimcinin de geçerli eşleşme ürettiği referans damgalarıdır.

TUM dışa aktarımı ve birim kuaterniyon
--------------------------------------
TUM biçimi ``timestamp tx ty tz qx qy qz qw`` ister. Kerteriz/RL CSV'leri
yönelim **taşımaz**, bu yüzden dışa aktarımda birim kuaterniyon yazılır.

**Bu yalnızca yönelimden ETKİLENMEYEN ölçütler için geçerlidir.** `evo`
kaynağında doğrulandı:

* ``evo_ape -r trans_part`` → ``E = pos_est - pos_ref`` (yalnız konum),
* ``evo_rpe -r point_distance`` → ``| ||ref_i-ref_j|| - ||est_i-est_j|| |``
  (yalnız konum).

Buna karşılık ``evo_rpe -r trans_part`` göreli pozun bileşiminden geçer ve
yönelime **bağlıdır**; birim kuaterniyonla kullanılırsa sonuç anlamsızdır.
Bu yüzden dışa aktarım fonksiyonunun adı niyeti taşır ve bu kısıt burada
yazılıdır. Birim kuaterniyonlar **gerçek kestirimci yönelimi değildir** ve
öyleymiş gibi sunulmaz.

`evo` çapraz kontrolü neyi doğrular, neyi doğrulamaz
---------------------------------------------------
CLI dışa aktarımı, kestirim örneklerini **ortak destekteki referans
damgalarıyla** yazar (`_samples_on`). Bu bilinçlidir — `evo` ile karşılaştırma
aynı nokta çiftleri üzerinde yapılsın diye. Sonucu şudur: `evo` bağımsız olarak
yeniden hesapladığı şey **öteleme APE aritmetiğidir**, zaman eşleştirmesi
**değildir**; eşleştirme dışa aktarımda zaten sabitlenmiştir. Eşleştirme
sözleşmesinin güvencesi bu modülün ve ``ate.hpp``'nin birim testleridir, `evo`
değildir. Dürüst ad: *evo ölçüt-aritmetiği çapraz kontrolü*.
"""

import argparse
import bisect
import csv
import math
import os

#: C++ koşucusunun ``ate_max_dt_ns`` varsayılanı ile aynı.
DEFAULT_TOLERANCE_NS = 20_000_000


class TrajectoryFormatError(ValueError):
    """CSV beklenen sözleşmeye uymuyor. Sessizce tolere EDİLMEZ."""


class Sample:
    """Zaman damgalı konum örneği."""

    __slots__ = ("stamp_ns", "position")

    def __init__(self, stamp_ns, position):
        self.stamp_ns = stamp_ns
        self.position = position

    def __repr__(self):  # pragma: no cover - yalnızca teşhis
        return "Sample({}, {})".format(self.stamp_ns, self.position)


def read_trajectory_csv(path):
    """``timestamp_ns,px,py,pz[,...]`` okur ve damgaya göre sıralı döndürür.

    Eksik sütun, sayıya çevrilemeyen alan, NaN/Inf veya yinelenen damga
    HATADIR. Bozuk bir deney çıktısını yarı yarıya değerlendirmek, olmayan bir
    sonucu varmış gibi göstermek olurdu.
    """
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise TrajectoryFormatError("{}: dosya boş".format(path)) from exc

        gerekli = ["timestamp_ns", "px", "py", "pz"]
        eksik = [c for c in gerekli if c not in header]
        if eksik:
            raise TrajectoryFormatError(
                "{}: eksik sütun: {}".format(path, ", ".join(eksik))
            )
        idx = [header.index(c) for c in gerekli]

        out = []
        gorulen = set()
        for satir_no, row in enumerate(reader, start=2):
            if not row:
                continue
            if len(row) != len(header):
                raise TrajectoryFormatError(
                    "{}:{}: {} alan var, {} bekleniyor".format(
                        path, satir_no, len(row), len(header)
                    )
                )
            try:
                stamp = int(row[idx[0]])
            except ValueError as exc:
                raise TrajectoryFormatError(
                    "{}:{}: damga tamsayı değil: {!r}".format(path, satir_no, row[idx[0]])
                ) from exc
            if stamp in gorulen:
                raise TrajectoryFormatError(
                    "{}:{}: yinelenen damga {}".format(path, satir_no, stamp)
                )
            gorulen.add(stamp)

            pos = []
            for k in idx[1:]:
                try:
                    v = float(row[k])
                except ValueError as exc:
                    raise TrajectoryFormatError(
                        "{}:{}: sayı değil: {!r}".format(path, satir_no, row[k])
                    ) from exc
                if not math.isfinite(v):
                    raise TrajectoryFormatError(
                        "{}:{}: sonlu olmayan değer: {!r}".format(path, satir_no, row[k])
                    )
                pos.append(v)
            out.append(Sample(stamp, tuple(pos)))

    if not out:
        raise TrajectoryFormatError("{}: veri satırı yok".format(path))
    out.sort(key=lambda s: s.stamp_ns)
    return out


def associate(reference, estimate, tolerance_ns=DEFAULT_TOLERANCE_NS):
    """Her referans örneğine en yakın kestirim örneğini eşler.

    Döner: ``(eslesme, eslesmeyen)`` — ``eslesme`` referans damgasından
    kestirim ``Sample``'ına sözlük, ``eslesmeyen`` tolerans dışında kalan
    referans örneği sayısı.
    """
    if tolerance_ns < 0:
        raise ValueError("tolerans negatif olamaz")
    damgalar = [s.stamp_ns for s in estimate]
    eslesme = {}
    eslesmeyen = 0
    for ref in reference:
        i = bisect.bisect_left(damgalar, ref.stamp_ns)
        en_iyi = None
        en_iyi_fark = None
        # Önce DAHA ERKEN aday: eşitlikte erken damga kazanır, çünkü yalnızca
        # KESİN olarak daha küçük bir fark onu iter.
        for aday in (i - 1, i):
            if 0 <= aday < len(estimate):
                fark = abs(estimate[aday].stamp_ns - ref.stamp_ns)
                if en_iyi is None or fark < en_iyi_fark:
                    en_iyi = estimate[aday]
                    en_iyi_fark = fark
        if en_iyi is None or en_iyi_fark > tolerance_ns:
            eslesmeyen += 1
            continue
        eslesme[ref.stamp_ns] = en_iyi
    return eslesme, eslesmeyen


def common_support(reference, estimate_a, estimate_b, tolerance_ns=DEFAULT_TOLERANCE_NS):
    """Her İKİ kestirimcinin de geçerli eşleşme ürettiği referans damgaları."""
    a, a_eksik = associate(reference, estimate_a, tolerance_ns)
    b, b_eksik = associate(reference, estimate_b, tolerance_ns)
    ortak = sorted(set(a) & set(b))
    return {
        "stamps": ortak,
        "match_a": a,
        "match_b": b,
        "unmatched_a": a_eksik,
        "unmatched_b": b_eksik,
    }


def translational_ate(reference, eslesme, stamps=None):
    """Öteleme ATE. Hizalama YOK; referans ve kestirim aynı çerçevededir."""
    ref_map = {s.stamp_ns: s.position for s in reference}
    if stamps is None:
        stamps = sorted(eslesme)
    hatalar = []
    for t in stamps:
        if t not in eslesme:
            raise KeyError("damga eşleşmede yok: {}".format(t))
        e = math.dist(eslesme[t].position, ref_map[t])
        hatalar.append(e)
    if not hatalar:
        return {"ok": False, "rmse_m": 0.0, "max_m": 0.0, "count": 0}
    return {
        "ok": True,
        "rmse_m": math.sqrt(sum(x * x for x in hatalar) / len(hatalar)),
        "max_m": max(hatalar),
        "count": len(hatalar),
    }


def write_tum_translation_only(path, samples):
    """TUM dosyası yazar — **yalnızca öteleme ölçütleri için**.

    Yönelim alanlarına birim kuaterniyon konur. Bu, modül başlığında
    doğrulandığı gibi ``evo_ape -r trans_part`` ve ``evo_rpe -r point_distance``
    sonuçlarını **etkilemez**; yönelime bağlı bir ölçütle kullanılırsa sonuç
    anlamsız olur. Fonksiyon adı bu kısıtı taşır.

    Damga **saniye** cinsine çevrilir (TUM sözleşmesi). Konum DEĞİŞTİRİLMEZ.
    """
    onceki = None
    with open(path, "w", encoding="utf-8") as handle:
        for s in samples:
            if onceki is not None and s.stamp_ns <= onceki:
                raise TrajectoryFormatError(
                    "TUM dışa aktarımı artan damga ister: {} <= {}".format(
                        s.stamp_ns, onceki
                    )
                )
            onceki = s.stamp_ns
            for v in s.position:
                if not math.isfinite(v):
                    raise TrajectoryFormatError("sonlu olmayan konum")
            handle.write(
                "{:.9f} {:.9f} {:.9f} {:.9f} 0.0 0.0 0.0 1.0\n".format(
                    s.stamp_ns * 1e-9, s.position[0], s.position[1], s.position[2]
                )
            )
    return path


def _samples_on(stamps, eslesme):
    """Ortak destek damgalarındaki kestirim örnekleri, damga referanstan."""
    return [Sample(t, eslesme[t].position) for t in stamps]


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Ortak zaman desteği üzerinde iki kestirimciyi değerlendirir"
    )
    parser.add_argument("--reference", required=True)
    parser.add_argument("--estimate-a", required=True)
    parser.add_argument("--estimate-b", required=True)
    parser.add_argument("--label-a", default="A")
    parser.add_argument("--label-b", default="B")
    parser.add_argument("--tolerance-ms", type=float, default=DEFAULT_TOLERANCE_NS * 1e-6)
    parser.add_argument(
        "--export-tum-dir",
        default=None,
        help="ortak destek üzerinde referans/A/B için TUM dosyaları yazar",
    )
    args = parser.parse_args(argv)

    tol = int(round(args.tolerance_ms * 1e6))
    ref = read_trajectory_csv(args.reference)
    a = read_trajectory_csv(args.estimate_a)
    b = read_trajectory_csv(args.estimate_b)

    destek = common_support(ref, a, b, tol)
    stamps = destek["stamps"]

    print("referans örnek      : {}".format(len(ref)))
    print("{:<18}: {} örnek, eşleşen {}, eşleşmeyen {}".format(
        args.label_a, len(a), len(destek["match_a"]), destek["unmatched_a"]))
    print("{:<18}: {} örnek, eşleşen {}, eşleşmeyen {}".format(
        args.label_b, len(b), len(destek["match_b"]), destek["unmatched_b"]))
    print("ORTAK destek        : {} damga   (tolerans {:.1f} ms)".format(
        len(stamps), tol * 1e-6))

    if not stamps:
        print("ortak destek boş — değerlendirme yapılamaz")
        return 1

    ra = translational_ate(ref, destek["match_a"], stamps)
    rb = translational_ate(ref, destek["match_b"], stamps)
    print()
    print("ORTAK destekte öteleme ATE (hizalama YOK):")
    for ad, r in ((args.label_a, ra), (args.label_b, rb)):
        print("  {:<18} rmse {:.6f} m   maks {:.6f} m   n {}".format(
            ad, r["rmse_m"], r["max_m"], r["count"]))
    if rb["rmse_m"] > 0:
        print("  oran {} / {} = {:.4f}".format(args.label_a, args.label_b,
                                               ra["rmse_m"] / rb["rmse_m"]))

    if args.export_tum_dir:
        os.makedirs(args.export_tum_dir, exist_ok=True)
        ref_ortak = [s for s in ref if s.stamp_ns in set(stamps)]
        yollar = {
            "reference": write_tum_translation_only(
                os.path.join(args.export_tum_dir, "reference.tum"), ref_ortak),
            args.label_a: write_tum_translation_only(
                os.path.join(args.export_tum_dir, "estimate_a.tum"),
                _samples_on(stamps, destek["match_a"])),
            args.label_b: write_tum_translation_only(
                os.path.join(args.export_tum_dir, "estimate_b.tum"),
                _samples_on(stamps, destek["match_b"])),
        }
        print()
        print("TUM dışa aktarımı (YALNIZCA öteleme ölçütleri için; birim kuaterniyon):")
        for ad, p in yollar.items():
            print("  {:<18} {}".format(ad, p))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
