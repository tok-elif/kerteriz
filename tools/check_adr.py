#!/usr/bin/env python3
"""ADR dizini ile bolunmus ADR dosyalari arasindaki butunluk.

PHASE0.md S12'den beri ADR'ler ayri dosyalara bolunuyor. Bolunme yarim
kaldiginda ortaya cikan hatalar SESSIZDIR: sarkan bir link, dizinde gorunmeyen
bir dosya, ya da adi icerigiyle uyusmayan bir dosya. Hicbiri derlemeyi
bozmaz, hicbiri testi dusurmez.

Numara surekliliginin tek basina yetmedigi nokta sudur: dizin girisi ile dosya
AYNI numarayi verir. Biri kaybolsa oteki numarayi ayakta tutar ve denetim yesil
kalir. Bu yuzden asagidaki uc kontrol numara sayimindan AYRIDIR ve her biri tek
basina baglayicidir.
"""

from __future__ import annotations

import glob
import io
import os
import re
import sys

DIZIN = "docs/design/DECISIONS.md"
ADR_DESENI = "docs/design/ADR-*.md"

# ADR-0001-ros-free-core.md
DOSYA_ADI = re.compile(r"^ADR-(\d+)-[A-Za-z0-9._-]+\.md$")
# '## ADR-1 · ...' — dizinde de dosyada da ayni bicim
BASLIK = re.compile(r"^#+ ADR-(\d+)", re.M)
# Markdown link hedefi: ](ADR-0001-ros-free-core.md)
LINK = re.compile(r"\]\(\s*(ADR-\d+-[A-Za-z0-9._-]+\.md)\s*\)")


def oku(yol: str) -> str:
    with io.open(yol, encoding="utf-8") as f:
        return f.read()


def main() -> int:
    ihlaller: list[str] = []

    dizin_metni = oku(DIZIN)
    dosyalar = sorted(glob.glob(ADR_DESENI))
    kok = os.path.dirname(DIZIN)

    # --- Numara surekliligi (DECISIONS + bolunmus dosyalar) ---
    numaralar = {int(x) for x in BASLIK.findall(dizin_metni)}
    for yol in dosyalar:
        numaralar |= {int(x) for x in BASLIK.findall(oku(yol))}
    sirali = sorted(numaralar)
    eksik = [i for i in range(1, max(sirali) + 1) if i not in numaralar]
    if eksik:
        ihlaller.append(f"numara surekliligi kirik, eksik ADR: {eksik}")

    # --- Dizindeki her link bir dosyaya isaret etmeli ---
    linkler = set(LINK.findall(dizin_metni))
    for hedef in sorted(linkler):
        if not os.path.isfile(os.path.join(kok, hedef)):
            ihlaller.append(f"{DIZIN}: sarkan link, dosya yok: {hedef}")

    # --- Her dosya dizinde linkli olmali ---
    for yol in dosyalar:
        ad = os.path.basename(yol)
        if ad not in linkler:
            ihlaller.append(f"{yol}: dizinde ({DIZIN}) linki yok")

    # --- Dosya adindaki numara icerideki TEK baslikla ayni olmali ---
    for yol in dosyalar:
        ad = os.path.basename(yol)
        m = DOSYA_ADI.match(ad)
        if not m:
            ihlaller.append(f"{yol}: dosya adi 'ADR-<numara>-<ad>.md' bicimine uymuyor")
            continue
        dosya_no = int(m.group(1))
        basliklar = [int(x) for x in BASLIK.findall(oku(yol))]
        if len(basliklar) != 1:
            ihlaller.append(
                f"{yol}: tam bir ADR basligi bekleniyor, {len(basliklar)} bulundu "
                f"({basliklar})"
            )
            continue
        if basliklar[0] != dosya_no:
            ihlaller.append(
                f"{yol}: dosya adi ADR-{dosya_no} diyor, icerideki baslik "
                f"ADR-{basliklar[0]}"
            )

    print(f"Dizin: {DIZIN}")
    print(f"Bolunmus ADR dosyasi: {len(dosyalar)}")
    print(f"ADR 1..{max(sirali)}, adet {len(sirali)}")

    if ihlaller:
        print(f"\nADR butunlugu DUSTU — {len(ihlaller)} bulgu:")
        for i in sorted(ihlaller):
            print(f"  {i}")
        return 1

    print("\nADR butunlugu temiz.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
