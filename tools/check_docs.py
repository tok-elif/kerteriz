#!/usr/bin/env python3
"""Denetim (5) — dokuman ve kod tutarliligi.

CLAUDE.md §6.1 · PHASE0.md S11.

Mimari freeze ancak zorlanabiliyorsa gercektir. Donmus tasarimin DISINDA kalan
semboller (eski `H` konvansiyonu, deger dondururen gurultu imzalari, C++20
API'leri) ne kodda ne de dokumanda CARI gibi anlatilmalidir.

Iki taraf farkli isler:

  KOD (.hpp/.cpp) — muafiyet YOKTUR. Bu semboller kodda hic bulunmamalidir.

  DOKUMAN (.md)   — "bunu yapma" uyarilari ve tarihsel ADR metinleri mesrudur;
                    zaten bu yasaklari anlatmak icin sembolu yazmak gerekir.
                    Muafiyet ACIK ve MAKINE-OKUNUR isaretlemeyle verilir.

                    Blok (kendi satirlarinda):
                        <!-- denetim5:muaf <sebep> -->
                        ... muaf satirlar ...
                        <!-- denetim5:muaf-son -->

                    Satir ici (yalnizca kendi satirini muaf kilar):
                        | R8 | ... | <!-- denetim5:muaf-satir <sebep> -->

                    Satir ici bicim tablolar icindir: kendi satirinda bir HTML
                    yorumu Markdown tablosunu BOLERDI. Hucre icindeki yorum ise
                    goruntulenmez.

                    Sezgisel anahtar kelime eslestirmesi (satirda "yasak"
                    geciyorsa gec gibi) KASTEN kullanilmaz: gercek bir drift
                    o kelimeyi tasiyan bir satira dusebilir ve sessizce gecerdi.

Muafiyet yuzeyi kendi kendine buyumesin diye iki kural daha vardir:

  * Bos bolge HATADIR. Hicbir yasakli sembol icermeyen bir muafiyet, metin
    tasindiktan sonra unutulmus demektir ve kaldirilmalidir.

  * Sebep metni, bolgenin yakaladigi HER sembolu adiyla saymalidir. Boylece
    muafiyet bir ALLOWLIST'tir, blanket degil: genis bir bolgeye sonradan
    BASKA bir yasakli sembol girerse sebepte adi gecmedigi icin denetim
    duser. Kod bloklari gibi isaretcinin ici yerine disina konmak zorunda
    olunan yerlerde bu kural muafiyeti dar tutan tek seydir.

NOT: bu dosya .py'dir ve taranmaz — aksi halde asagidaki liste kendini
tetiklerdi.
"""

from __future__ import annotations

import subprocess
import sys
from dataclasses import dataclass, field

# Donmus tasarimin disinda kalan semboller.
# TEK KAYNAK: CLAUDE.md §6.1 tablosunun 5. satiri. Liste oradan kopyalanir;
# yeni mimari yasak BURADA icat edilmez.
YASAKLI_SEMBOLLER = (
    "MatX H",
    "struct Residual",
    "MatX noise",
    "CompositeState<",
    "reset(const NavState",
    ".inverse()",
    "std::span",
)

MUAF_BASLA = "<!-- denetim5:muaf"
MUAF_BITIR = "<!-- denetim5:muaf-son -->"
MUAF_SATIR = "<!-- denetim5:muaf-satir"


@dataclass
class Bolge:
    """Bir muafiyet bolgesi."""

    dosya: str
    basla: int
    sebep: str
    bitir: int | None = None
    yakalanan: list[str] = field(default_factory=list)
    satir_ici: bool = False


def izlenen_dosyalar(desen: str) -> list[str]:
    cikti = subprocess.run(
        ["git", "ls-files", desen], capture_output=True, text=True, check=True
    ).stdout
    return [s for s in cikti.splitlines() if s]


def satirlar(yol: str) -> list[str]:
    with open(yol, encoding="utf-8") as f:
        return f.read().splitlines()


def bulunan_semboller(satir: str) -> list[str]:
    return [s for s in YASAKLI_SEMBOLLER if s in satir]


def kodu_tara() -> list[str]:
    """Kod tarafi: muafiyet yok."""
    ihlaller = []
    for yol in izlenen_dosyalar("*.hpp") + izlenen_dosyalar("*.cpp"):
        for no, satir in enumerate(satirlar(yol), 1):
            for sembol in bulunan_semboller(satir):
                ihlaller.append(f"{yol}:{no}: yasakli sembol kodda: {sembol}")
    return ihlaller


def dokumani_tara() -> tuple[list[str], list[Bolge]]:
    """Dokuman tarafi: yalnizca acik isaretlenmis bolgeler muaftir."""
    ihlaller: list[str] = []
    tum_bolgeler: list[Bolge] = []
    for yol in izlenen_dosyalar("*.md"):
        i, b = belge_tara(yol, satirlar(yol))
        ihlaller.extend(i)
        tum_bolgeler.extend(b)
    return ihlaller, tum_bolgeler


def belge_tara(yol: str, belge: list[str]) -> tuple[list[str], list[Bolge]]:
    """Tek bir dokumani tarar.

    Dosya okumasindan AYRIDIR: oz-test (--self-test) ayni kod yolunu sentetik
    belgelerle kosturabilsin diye. Tarayiciyi test eden bir testin, tarayicinin
    kendisinden farkli bir yol izlemesi anlamsiz olurdu.
    """
    ihlaller: list[str] = []
    tum_bolgeler: list[Bolge] = []

    if True:
        acik: Bolge | None = None
        cit_icinde = False

        for no, satir in enumerate(belge, 1):
            # Cit (```) icinde isaretci TANINMAZ. Isaretci zaten citin icine
            # konamaz — orada render edilir, gizlenmez — bu yuzden cit icindeki
            # bir isaretci ancak sozdiziminin KENDISINI anlatan bir ornektir
            # (bkz. PHASE0.md S11). Yasakli sembol taramasi cit icinde de surer;
            # o satirlar citi DISARIDAN saran bolgeye yazilir.
            if satir.lstrip().startswith("```"):
                cit_icinde = not cit_icinde

            if not cit_icinde and MUAF_SATIR in satir:
                once, kalan = satir.split(MUAF_SATIR, 1)
                if "-->" not in kalan:
                    ihlaller.append(
                        f"{yol}:{no}: satir ici muafiyet isaretcisi kapatilmamis ('-->' yok)"
                    )
                    continue
                sebep, sonra = kalan.split("-->", 1)
                sebep = sebep.strip()
                if not sebep:
                    ihlaller.append(f"{yol}:{no}: satir ici muafiyet sebepsiz")

                # ISARETCININ KENDISI TARANMAZ. Sebep metni zaten kapsadigi
                # sembolleri adiyla sayar (allowlist kurali); isaretci de
                # taransaydi her muafiyet kendi sebebiyle "dolu" gorunur,
                # bos-muafiyet kurali islemez ve yalnizca isaretcide gecen bir
                # sembol icin acilmis muafiyet sahiden dolu sanilirdi.
                b = Bolge(dosya=yol, basla=no, bitir=no, sebep=sebep, satir_ici=True)
                b.yakalanan.extend(bulunan_semboller(once + sonra))
                tum_bolgeler.append(b)
                continue

            if (
                not cit_icinde
                and satir.strip().startswith(MUAF_BASLA)
                and MUAF_BITIR not in satir
            ):
                govde = satir.strip()
                # Kor kesim yapilmaz: '-->' yoksa sondan uc karakter atmak
                # sebebi bozar ve kapanmamis isaretciyi sessizce kabul ederdi.
                # Satir ici bicimdeki ayni acik burada da kapatilir.
                if not govde.endswith("-->"):
                    ihlaller.append(
                        f"{yol}:{no}: muafiyet blok isaretcisi kapatilmamis ('-->' yok)"
                    )
                    continue
                if acik is not None:
                    ihlaller.append(
                        f"{yol}:{no}: ic ice muafiyet bolgesi "
                        f"(onceki {acik.basla}. satirda acildi)"
                    )
                    continue
                sebep = govde[len(MUAF_BASLA) : -len("-->")].strip()
                if not sebep:
                    ihlaller.append(f"{yol}:{no}: muafiyet bolgesi sebepsiz acilmis")
                acik = Bolge(dosya=yol, basla=no, sebep=sebep)
                continue

            if not cit_icinde and satir.strip() == MUAF_BITIR:
                if acik is None:
                    ihlaller.append(f"{yol}:{no}: acilmamis muafiyet bolgesi kapatiliyor")
                    continue
                acik.bitir = no
                tum_bolgeler.append(acik)
                acik = None
                continue

            eslesme = bulunan_semboller(satir)
            if not eslesme:
                continue
            if acik is None:
                for sembol in eslesme:
                    ihlaller.append(f"{yol}:{no}: muaf olmayan yasakli sembol: {sembol}")
            else:
                acik.yakalanan.extend(eslesme)

        if acik is not None:
            ihlaller.append(f"{yol}:{acik.basla}: muafiyet bolgesi kapatilmamis")

    return ihlaller, tum_bolgeler


def bolge_bulgulari(bolgeler: list[Bolge]) -> list[str]:
    """Muafiyet yuzeyini dar tutan iki kural."""
    ihlaller: list[str] = []
    for b in bolgeler:
        if not b.yakalanan:
            ihlaller.append(
                f"{b.dosya}:{b.basla}: muafiyet hicbir yasakli sembol icermiyor "
                f"— kaldirilmali ({b.sebep})"
            )
            continue
        for sembol in sorted({s for s in b.yakalanan if s not in b.sebep}):
            ihlaller.append(
                f"{b.dosya}:{b.basla}: muafiyetin sebebi '{sembol}' sembolunu "
                f"saymiyor — muafiyet ancak adi gecen sembolleri kapsar"
            )
    return ihlaller


# -----------------------------------------------------------------------------
# Oz-test — tarayicinin kendi regresyon testi.
#
# Denetim (5) bir KAPIDIR; kapinin kendisi bozulursa hicbir sey uyarmaz, her sey
# yesil kalir. Bu proje o sinif hatayi birkac kez yasadi. Her ihlal sinifi
# burada kalici olarak kilitlenir; CI gercek taramadan ONCE bunu kosturur.
# -----------------------------------------------------------------------------

OZ_TEST_VAKALARI: tuple[tuple[str, list[str], str], ...] = (
    (
        "muaf olmayan sembol",
        ["Backend `S.inverse()` cagirir."],
        "muaf olmayan yasakli sembol",
    ),
    (
        "kapanmayan blok acilisi",
        [
            "<!-- denetim5:muaf .inverse() — kapanmiyor",
            "`S.inverse()` yasaktir.",
            "<!-- denetim5:muaf-son -->",
        ],
        "blok isaretcisi kapatilmamis",
    ),
    (
        "kapanmayan satir ici isaretci",
        ["`S.inverse()` yasak. <!-- denetim5:muaf-satir .inverse() — kapanmiyor"],
        "satir ici muafiyet isaretcisi kapatilmamis",
    ),
    (
        "kapatilmamis blok",
        ["<!-- denetim5:muaf .inverse() — sebep -->", "`S.inverse()` yasaktir."],
        "muafiyet bolgesi kapatilmamis",
    ),
    (
        "acilmamis blok kapatiliyor",
        ["<!-- denetim5:muaf-son -->"],
        "acilmamis muafiyet bolgesi",
    ),
    (
        "sebepsiz blok",
        [
            "<!-- denetim5:muaf -->",
            "`S.inverse()` yasaktir.",
            "<!-- denetim5:muaf-son -->",
        ],
        "sebepsiz acilmis",
    ),
    (
        "ic ice blok",
        [
            "<!-- denetim5:muaf .inverse() — dis -->",
            "<!-- denetim5:muaf .inverse() — ic -->",
            "`S.inverse()` yasaktir.",
            "<!-- denetim5:muaf-son -->",
        ],
        "ic ice muafiyet bolgesi",
    ),
)

# Bolge kurallari ayri kosar: bunlar tarama degil, muafiyet yuzeyi kurallaridir.
OZ_TEST_BOLGE_VAKALARI: tuple[tuple[str, list[str], str], ...] = (
    (
        "bos blok muafiyeti",
        [
            "<!-- denetim5:muaf .inverse() — artik bir sey yok -->",
            "siradan metin",
            "<!-- denetim5:muaf-son -->",
        ],
        "hicbir yasakli sembol icermiyor",
    ),
    (
        # Asil regresyon: isaretcinin KENDISI taranirsa bu vaka "dolu" gorunur.
        "bos satir ici muafiyet (sembol yalnizca isaretcide)",
        ["Gercek sembol yok. <!-- denetim5:muaf-satir .inverse() — uydurma -->"],
        "hicbir yasakli sembol icermiyor",
    ),
    (
        "sebep sembolu saymiyor",
        [
            "<!-- denetim5:muaf std::span — yalnizca span -->",
            "Ama `S.inverse()` da var.",
            "<!-- denetim5:muaf-son -->",
        ],
        "sembolunu saymiyor",
    ),
)

OZ_TEST_TEMIZ: tuple[tuple[str, list[str]], ...] = (
    ("dogru blok muafiyeti", [
        "<!-- denetim5:muaf .inverse() — yasagi anlatan uyari -->",
        "`S.inverse()` yasaktir.",
        "<!-- denetim5:muaf-son -->",
    ]),
    ("dogru satir ici muafiyet", [
        "| Kovaryans | `S.inverse()` yasak <!-- denetim5:muaf-satir .inverse() — kural --> |",
    ]),
    ("cit icindeki isaretci ornegi taninmaz", [
        "```",
        "<!-- denetim5:muaf <semboller> — <sebep> -->",
        "```",
    ]),
    ("yasakli sembol yok", ["Siradan bir cumle."]),
)


def oz_test() -> int:
    hatalar: list[str] = []

    def bulgular(belge: list[str]) -> list[str]:
        i, b = belge_tara("<oz-test>", belge)
        return i + bolge_bulgulari(b)

    for ad, belge, beklenen in OZ_TEST_VAKALARI + OZ_TEST_BOLGE_VAKALARI:
        cikan = bulgular(belge)
        if not any(beklenen in x for x in cikan):
            hatalar.append(f"'{ad}' yakalanmadi; beklenen '{beklenen}', cikan: {cikan}")

    for ad, belge in OZ_TEST_TEMIZ:
        cikan = bulgular(belge)
        if cikan:
            hatalar.append(f"'{ad}' temiz olmaliydi; cikan: {cikan}")

    toplam = len(OZ_TEST_VAKALARI) + len(OZ_TEST_BOLGE_VAKALARI) + len(OZ_TEST_TEMIZ)
    if hatalar:
        print(f"Oz-test DUSTU — {len(hatalar)}/{toplam} vaka:")
        for h in hatalar:
            print(f"  {h}")
        return 1
    print(f"Oz-test temiz — {toplam} vaka.")
    return 0


def main() -> int:
    ihlaller = kodu_tara()
    dok_ihlaller, bolgeler = dokumani_tara()
    ihlaller.extend(dok_ihlaller)

    ihlaller.extend(bolge_bulgulari(bolgeler))

    print(f"Taranan yasakli sembol: {len(YASAKLI_SEMBOLLER)}")
    print(f"Muafiyet bolgesi: {len(bolgeler)}")
    for b in sorted(bolgeler, key=lambda x: (x.dosya, x.basla)):
        adet = len(b.yakalanan)
        yer = f"{b.dosya}:{b.basla}" if b.satir_ici else f"{b.dosya}:{b.basla}-{b.bitir}"
        print(f"  muaf  {yer}  ({adet} eslesme)  {b.sebep}")

    if ihlaller:
        print(f"\nDenetim (5) DUSTU — {len(ihlaller)} bulgu:")
        for i in sorted(ihlaller):
            print(f"  {i}")
        return 1

    print("\nDenetim (5) temiz.")
    return 0


if __name__ == "__main__":
    if "--self-test" in sys.argv:
        sys.exit(oz_test())
    sys.exit(main())
