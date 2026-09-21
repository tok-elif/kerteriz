#!/usr/bin/env python3
"""E1 tekrarlanabilirlik dogrulamasi — bayt karsilastirmasi.

`make verify` bunu cagirir: E1 gecici bir dizine SIFIRDAN yeniden uretilir ve
commit'lenmis artefaktlarla karsilastirilir.

============================ NE KARSILASTIRILIR =============================

BILIMSEL (deterministik) — bayt bayt ayni olmak ZORUNDA:

    phase2_eskf_anees.csv
    phase2_eskf_anis.csv
    phase2_eskf_manifest.txt

KOKEN (nondeterministik) — KARSILASTIRILMAZ:

    phase2_eskf_run_metadata.txt   (runtime_s, git_commit)

TOOLCHAIN'E BAGLI — bayt karsilastirilir, ama basarisizligi ayri okunur:

    phase2_eskf_consistency.png

=========================== NEDEN TOLERANS YOK ==============================

Bu betik hicbir sayisal tolerans TANIMLAMAZ. E1 tohumlamasi
(master_seed, run_index) ciftinin saf fonksiyonudur ve kosu tek is
parcaciklidir; ayni kod ayni baytlari uretir. "Neredeyse ayni" kabul eden bir
esik, gercek bir sayisal kaymayi sessizce yutardi.

Nondeterministik alanlar bir YOK-SAYMA LISTESIYLE degil, DOSYA AYRIMIYLA
cozulur: karsilastirilan manifest'te `runtime_s`/`git_commit` hic bulunmaz.
Yok-sayma listesi zamanla buyur ve neyin gercekten dogrulandigi belirsizlesir.

============================= PNG HAKKINDA ==================================

PNG bu toolchain'de bayt kararlidir (ayni girdi -> ayni bayt, tekrar tekrar).
Ancak matplotlib PNG'ye bir `tEXt` chunk'inda KENDI SURUMUNU gomer, dolayisiyla
matplotlib surumu degisirse baytlar da degisir. Bu bir bilimsel gerileme
DEGILDIR. Betik PNG uyusmazliginda iki dosyanin gomulu surumunu de basar ki
surum kaymasi ile gercek degisiklik ayirt edilebilsin. Piksel toleransi
UYDURULMAZ: sayisal kanit CSV'lerdedir, PNG onlarin gorsellestirmesidir.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import struct
import sys

BILIMSEL = (
    "phase2_eskf_anees.csv",
    "phase2_eskf_anis.csv",
    "phase2_eskf_manifest.txt",
)
TOOLCHAIN = ("phase2_eskf_consistency.png",)
KARSILASTIRILMAYAN = ("phase2_eskf_run_metadata.txt",)

# Manifest'te ASLA bulunmamasi gereken alanlar. Buraya dusmeleri, ayrimin
# bozuldugu ve dogrulamanin yok-sayma listesine muhtac kaldigi anlamina gelir.
YASAK_MANIFEST_ALANLARI = ("runtime_s", "git_commit")


def sha256(yol: str) -> str:
    h = hashlib.sha256()
    with open(yol, "rb") as f:
        for blok in iter(lambda: f.read(65536), b""):
            h.update(blok)
    return h.hexdigest()


def png_yazilim_etiketi(yol: str) -> str:
    """PNG'ye gomulu `Software` tEXt chunk'ini dondurur (yoksa bos)."""
    try:
        with open(yol, "rb") as f:
            veri = f.read()
    except OSError:
        return ""
    i = 8
    while i + 8 <= len(veri):
        uzunluk = struct.unpack(">I", veri[i : i + 4])[0]
        tip = veri[i + 4 : i + 8]
        if tip == b"tEXt":
            govde = veri[i + 8 : i + 8 + uzunluk]
            if govde.startswith(b"Software\x00"):
                return govde.split(b"\x00", 1)[1].decode("latin1")
        if tip == b"IEND":
            break
        i += 12 + uzunluk
    return ""


def karsilastir(commit_dizin: str, taze_dizin: str) -> int:
    hatalar: list[str] = []

    print("E1 tekrarlanabilirlik dogrulamasi")
    print("  commit'lenmis : {}".format(commit_dizin))
    print("  taze uretim   : {}".format(taze_dizin))
    print()

    # --- ayrim saglam mi: manifest nondeterministik alan tasimamali ---------
    manifest = os.path.join(taze_dizin, "phase2_eskf_manifest.txt")
    if os.path.exists(manifest):
        with open(manifest, encoding="utf-8") as f:
            metin = f.read()
        for alan in YASAK_MANIFEST_ALANLARI:
            if "\n{} =".format(alan) in metin or metin.startswith("{} =".format(alan)):
                hatalar.append(
                    "manifest nondeterministik alan tasiyor: {} "
                    "(koken dosyasina tasinmali)".format(alan)
                )

    print("BILIMSEL artefaktlar — bayt esitligi ZORUNLU")
    for ad in BILIMSEL:
        a, b = os.path.join(commit_dizin, ad), os.path.join(taze_dizin, ad)
        if not os.path.exists(a):
            hatalar.append("commit'lenmis dosya yok: {}".format(ad))
            print("  {:<32} EKSIK (commit'lenmis)".format(ad))
            continue
        if not os.path.exists(b):
            hatalar.append("taze uretimde dosya yok: {}".format(ad))
            print("  {:<32} EKSIK (taze)".format(ad))
            continue
        ha, hb = sha256(a), sha256(b)
        if ha == hb:
            print("  {:<32} AYNI    sha256 {}".format(ad, ha[:16]))
        else:
            hatalar.append("BILIMSEL ARTEFAKT DEGISTI: {}".format(ad))
            print("  {:<32} FARKLI".format(ad))
            print("      commit'lenmis sha256 {}".format(ha))
            print("      taze          sha256 {}".format(hb))

    print()
    print("TOOLCHAIN'E BAGLI artefakt — bayt esitligi, uyusmazlikta tani basilir")
    for ad in TOOLCHAIN:
        a, b = os.path.join(commit_dizin, ad), os.path.join(taze_dizin, ad)
        if not (os.path.exists(a) and os.path.exists(b)):
            hatalar.append("PNG eksik: {}".format(ad))
            print("  {:<32} EKSIK".format(ad))
            continue
        ha, hb = sha256(a), sha256(b)
        if ha == hb:
            print("  {:<32} AYNI    sha256 {}".format(ad, ha[:16]))
        else:
            sa, sb = png_yazilim_etiketi(a), png_yazilim_etiketi(b)
            hatalar.append("PNG degisti: {}".format(ad))
            print("  {:<32} FARKLI".format(ad))
            print("      commit'lenmis: {}".format(sa or "(Software etiketi yok)"))
            print("      taze         : {}".format(sb or "(Software etiketi yok)"))
            if sa and sb and sa != sb:
                print("      -> gomulu cizim kutuphanesi surumu FARKLI.")
                print("         Bu bir bilimsel gerileme degildir; sayisal kanit")
                print("         CSV'lerdedir ve onlar ayri raporlanir.")
            else:
                print("      -> surum etiketleri ayni; fark cizim girdisinden geliyor")
                print("         olabilir. CSV sonuclarina bakin.")

    print()
    print("KARSILASTIRILMAYAN (koken, nondeterministik)")
    for ad in KARSILASTIRILMAYAN:
        b = os.path.join(taze_dizin, ad)
        print("  {:<32} {}".format(ad, "uretildi" if os.path.exists(b) else "YOK"))
        if not os.path.exists(b):
            hatalar.append("koken dosyasi uretilmedi: {}".format(ad))

    print()
    if hatalar:
        print("DOGRULAMA BASARISIZ")
        for h in hatalar:
            print("  * {}".format(h))
        return 1
    print("DOGRULAMA GECTI — E1 sifirdan bayt bayt yeniden uretildi.")
    return 0


def main(argv=None) -> int:
    p = argparse.ArgumentParser(description="E1 artefaktlarini bayt bayt dogrular")
    p.add_argument("--committed", default="results/e1")
    p.add_argument("--fresh", required=True)
    a = p.parse_args(argv)
    return karsilastir(a.committed, a.fresh)


if __name__ == "__main__":
    raise SystemExit(main())
