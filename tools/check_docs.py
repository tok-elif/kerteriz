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

# Donmus tasarimin disinda kalan semboller (CLAUDE.md §6.1, PHASE0.md S11).
YASAKLI_SEMBOLLER = (
    "MatX H",
    "struct Residual",
    "MatX noise",
    "CompositeState<",
    "EuclideanBlock",
    "reset(const NavState",
    "IntegrityState integrity",
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
        acik: Bolge | None = None
        cit_icinde = False

        for no, satir in enumerate(satirlar(yol), 1):
            # Cit (```) icinde isaretci TANINMAZ. Isaretci zaten citin icine
            # konamaz — orada render edilir, gizlenmez — bu yuzden cit icindeki
            # bir isaretci ancak sozdiziminin KENDISINI anlatan bir ornektir
            # (bkz. PHASE0.md S11). Yasakli sembol taramasi cit icinde de surer;
            # o satirlar citi DISARIDAN saran bolgeye yazilir.
            if satir.lstrip().startswith("```"):
                cit_icinde = not cit_icinde

            if not cit_icinde and MUAF_SATIR in satir:
                sebep = satir.split(MUAF_SATIR, 1)[1].split("-->", 1)[0].strip()
                if not sebep:
                    ihlaller.append(f"{yol}:{no}: satir ici muafiyet sebepsiz")
                b = Bolge(dosya=yol, basla=no, bitir=no, sebep=sebep, satir_ici=True)
                b.yakalanan.extend(bulunan_semboller(satir))
                tum_bolgeler.append(b)
                continue

            if (
                not cit_icinde
                and satir.strip().startswith(MUAF_BASLA)
                and MUAF_BITIR not in satir
            ):
                if acik is not None:
                    ihlaller.append(
                        f"{yol}:{no}: ic ice muafiyet bolgesi "
                        f"(onceki {acik.basla}. satirda acildi)"
                    )
                    continue
                sebep = satir.strip()[len(MUAF_BASLA) : -len("-->")].strip()
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


def main() -> int:
    ihlaller = kodu_tara()
    dok_ihlaller, bolgeler = dokumani_tara()
    ihlaller.extend(dok_ihlaller)

    # Bos bolge = tasinmis veya silinmis metinden arta kalan muafiyet.
    # Muafiyet yuzeyinin sessizce buyumesine izin verilmez.
    for b in bolgeler:
        if not b.yakalanan:
            ihlaller.append(
                f"{b.dosya}:{b.basla}: muafiyet hicbir yasakli sembol icermiyor "
                f"— kaldirilmali ({b.sebep})"
            )
            continue
        adsiz = sorted({s for s in b.yakalanan if s not in b.sebep})
        for sembol in adsiz:
            ihlaller.append(
                f"{b.dosya}:{b.basla}: muafiyetin sebebi '{sembol}' sembolunu "
                f"saymiyor — muafiyet ancak adi gecen sembolleri kapsar"
            )

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
    sys.exit(main())
