"""E1 — Faz 2 ESKF tutarlılık şekli.

Bu modül YALNIZCA ÇİZER. Güven bantları C++ istatistik motorundan (F2.2) gelir
ve burada yeniden hesaplanmaz; bu yüzden `scipy` bağımlılığı da yoktur.

Şekil ESKF'in tek başına koşulduğu bir Faz 2 taban çizgisidir. InEKF Faz 3'te
aynı deney sözleşmesinden geçirilecektir; bu şekil onun koşulduğunu ima etmez.
"""

import argparse
import csv
import os

ANEES_COLUMNS = [
    "timestamp_ns",
    "time_s",
    "mean_nees",
    "expected_mean",
    "lower_95",
    "upper_95",
    "valid_count",
    "invalid_count",
]

ANIS_COLUMNS = [
    "timestamp_ns",
    "time_s",
    "mean_nis",
    "expected_mean",
    "lower_95",
    "upper_95",
    "valid_count",
    "undefined_count",
    "total_dof",
]


class CsvFormatError(ValueError):
    """CSV beklenen sözleşmeye uymuyor. Sessizce tolere EDİLMEZ."""


def read_series(path, required_columns):
    """CSV'yi sütun -> float listesi olarak okur.

    Eksik sütun, eksik alan veya sayıya çevrilemeyen değer HATADIR: bozuk bir
    deney çıktısını yarı yarıya çizmek, olmayan bir sonucu varmış gibi
    göstermek olurdu.
    """
    with open(path, newline="", encoding="utf-8") as handle:
        reader = csv.reader(handle)
        try:
            header = next(reader)
        except StopIteration as exc:
            raise CsvFormatError(f"{path}: dosya boş") from exc

        missing = [c for c in required_columns if c not in header]
        if missing:
            raise CsvFormatError(f"{path}: eksik sütun: {', '.join(missing)}")

        index = {name: header.index(name) for name in required_columns}
        out = {name: [] for name in required_columns}
        for line_no, row in enumerate(reader, start=2):
            if not row:
                continue
            if len(row) != len(header):
                raise CsvFormatError(
                    f"{path}:{line_no}: {len(row)} alan var, {len(header)} bekleniyor"
                )
            for name, pos in index.items():
                try:
                    out[name].append(float(row[pos]))
                except ValueError as exc:
                    raise CsvFormatError(
                        f"{path}:{line_no}: '{name}' sayı değil: {row[pos]!r}"
                    ) from exc

    if not out[required_columns[0]]:
        raise CsvFormatError(f"{path}: veri satırı yok")
    return out


def _panel(axis, series, value_key, title, ylabel):
    axis.fill_between(
        series["time_s"],
        series["lower_95"],
        series["upper_95"],
        color="0.85",
        label="%95 topluluk güven bandı",
    )
    axis.plot(
        series["time_s"],
        series["expected_mean"],
        linestyle="--",
        color="0.35",
        linewidth=1.0,
        label="beklenen ortalama",
    )
    axis.plot(series["time_s"], series[value_key], color="C0", linewidth=1.2, label=title)
    axis.set_xlabel("zaman [s]")
    axis.set_ylabel(ylabel)
    axis.set_title(title)
    axis.grid(True, alpha=0.3)
    axis.legend(loc="upper right", fontsize=8)


def make_figure(anees, anis, output_path, runs=None):
    """İki panelli E1 şekli üretir. matplotlib gerektirir."""
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (top, bottom) = plt.subplots(2, 1, figsize=(9, 7), sharex=True)

    kosu = runs if runs is not None else int(max(anees["valid_count"]))
    fig.suptitle(
        "E1 · ESKF tutarlılık — {} Monte Carlo koşusu, büyük yaw başlangıç "
        "belirsizliği\n%95 topluluk güven bandı (yalnızca ESKF; InEKF Faz 3)".format(kosu)
    )

    _panel(top, anees, "mean_nees", "ANEES (15 DoF)", "ANEES")
    _panel(bottom, anis, "mean_nis", "ANIS", "ANIS")

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(output_path, dpi=150)
    plt.close(fig)
    return output_path


def main(argv=None):
    parser = argparse.ArgumentParser(description="E1 ESKF tutarlılık şekli")
    parser.add_argument("--input-dir", default="results/e1")
    parser.add_argument("--anees", default=None)
    parser.add_argument("--anis", default=None)
    parser.add_argument("--output", default=None)
    parser.add_argument("--runs", type=int, default=None)
    args = parser.parse_args(argv)

    anees_path = args.anees or os.path.join(args.input_dir, "phase2_eskf_anees.csv")
    anis_path = args.anis or os.path.join(args.input_dir, "phase2_eskf_anis.csv")
    output = args.output or os.path.join(args.input_dir, "phase2_eskf_consistency.png")

    anees = read_series(anees_path, ANEES_COLUMNS)
    anis = read_series(anis_path, ANIS_COLUMNS)
    make_figure(anees, anis, output, runs=args.runs)
    print("şekil: {}".format(output))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
