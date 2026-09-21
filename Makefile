# Kerteriz — sonuc ardisik duzeni
#
# ============================ BU DOSYA NE YAPAR ==============================
#
# `make results` commit'lenmis sonuc artefaktlarini SIFIRDAN uretir.
# F2.5-B kapsaminda bu YALNIZCA E1'dir: sentetik Monte Carlo tutarlilik
# deneyi. KITTI sonuclari harici veri seti ve canli bir ROS ardisik duzeni
# gerektirir; onlar F2.5-C'de AYRI bir hedefe baglanacaktir ve bu dosyada
# HENUZ YOKTUR. Var olmayan bir hedefi ima etmemek icin buraya bos bir
# `results-kitti` de konmamistir.
#
# ============================== HERMETIKLIK ==================================
#
# `make results` SU ANDA tamamen hermetiktir:
#
#   * ROS 2 GEREKMEZ            — E1 `kerteriz_sim`'i dogrudan CMake ile kurar
#   * harici veri seti GEREKMEZ — yorunge sentetik olarak uretilir
#   * ag erisimi GEREKMEZ       — manif/Eigen zaten `build/sim` altinda cozulur
#
# Bu ozellik kasitlidir: sonuc uretimi, veri seti edinmeye bagli olmayan bir
# cekirdek tasimalidir, yoksa CI'da hicbir zaman kosamaz.
#
# =========================== PYTHON VARSAYIMLARI =============================
#
# Cizim adimi `kerteriz_eval` paketini colcon KURULUMU OLMADAN, dogrudan kaynak
# agacindan calistirir (`PYTHONPATH=kerteriz_eval`). Boylece temiz bir
# checkout'ta ROS kurulmadan da `make results` calisir.
#
# Gerekenler (kerteriz_eval/setup.py ile ayni):
#
#   python3 >= 3.10 · numpy · matplotlib
#
# `make check-env` bunlari kontrol eder ve eksikse ADINI soyleyerek durur.
# `evo` BURADA GEREKMEZ — evo yalnizca KITTI degerlendirmesi icindir ve
# F2.5-C'nin isidir.

PYTHON      ?= python3
BUILD_SIM   ?= build/sim
RESULTS_E1  ?= results/e1
E1_BIN      := $(BUILD_SIM)/kerteriz_e1_eskf
GIT_COMMIT  ?= $(shell git rev-parse HEAD 2>/dev/null || echo bilinmiyor)
PYPATH      := PYTHONPATH=kerteriz_eval

# Bilimsel artefaktlar: `make verify` bunlari bayt bayt karsilastirir.
E1_BILIMSEL := \
	$(RESULTS_E1)/phase2_eskf_anees.csv \
	$(RESULTS_E1)/phase2_eskf_anis.csv \
	$(RESULTS_E1)/phase2_eskf_manifest.txt
# Koken artefakti: nondeterministik, karsilastirmaya GIRMEZ.
E1_KOKEN    := $(RESULTS_E1)/phase2_eskf_run_metadata.txt

.PHONY: help results results-e1 verify check-env clean-results build-sim

help:
	@echo "Kerteriz — sonuc hedefleri"
	@echo ""
	@echo "  make results        sonuclari sifirdan uret (su anda = results-e1)"
	@echo "  make results-e1     E1 Monte Carlo tutarlilik artefaktlari"
	@echo "  make verify         E1'i gecici dizine yeniden uret ve BAYT karsilastir"
	@echo "  make check-env      Python bagimliliklarini dogrula"
	@echo "  make clean-results  uretilmis E1 artefaktlarini sil"
	@echo ""
	@echo "KITTI sonuclari F2.5-C'dedir; bu Makefile'da henuz hedefi yoktur."

# F2.5-B: results == results-e1. KITTI eklendiginde bu satir genisler.
results: results-e1

check-env:
	@$(PYTHON) -c "import sys; assert sys.version_info >= (3, 10), sys.version" \
		|| { echo "HATA: python3 >= 3.10 gerekli"; exit 1; }
	@$(PYTHON) -c "import numpy" \
		|| { echo "HATA: numpy yok — kurun: pip install numpy"; exit 1; }
	@$(PYTHON) -c "import matplotlib" \
		|| { echo "HATA: matplotlib yok — kurun: pip install matplotlib"; exit 1; }
	@echo "ortam tamam: $$($(PYTHON) --version), numpy + matplotlib mevcut"

# Tazelik kararini CMake verir. Bu hedef BILEREK .PHONY'dir: ikiliyi dosya
# hedefi yapip kaynaklari listelemek, bir kaynak eklendiginde sessizce BAYAT
# bir ikiliyle sonuc uretmek demekti — tam da `make results`'in onlemesi
# gereken hata. `cmake --build` zaten artimlidir, bedeli bos bir kontroldur.
build-sim:
	@cmake -S kerteriz_sim -B $(BUILD_SIM) -DCMAKE_BUILD_TYPE=Release
	@cmake --build $(BUILD_SIM) -j

results-e1: check-env build-sim
	@mkdir -p $(RESULTS_E1)
	./$(E1_BIN) --output-dir $(RESULTS_E1) --git-commit $(GIT_COMMIT)
	$(PYPATH) $(PYTHON) -m kerteriz_eval.e1_plot --input-dir $(RESULTS_E1)

# E1'i GECICI bir dizine yeniden uretir ve commit'lenmislerle karsilastirir.
# Commit'lenmis artefaktlar BU HEDEF TARAFINDAN YAZILMAZ; dogrulama, uzerine
# yazip "ayni" demenin aksine gercek bir karsilastirmadir.
verify: check-env build-sim
	@tmp=$$(mktemp -d) && \
	echo "gecici uretim dizini: $$tmp" && \
	./$(E1_BIN) --output-dir $$tmp --git-commit verify-gecici && \
	$(PYPATH) $(PYTHON) -m kerteriz_eval.e1_plot --input-dir $$tmp && \
	echo "" && \
	$(PYTHON) scripts/verify_e1.py --committed $(RESULTS_E1) --fresh $$tmp; \
	durum=$$?; rm -rf $$tmp; exit $$durum

clean-results:
	rm -f $(E1_BILIMSEL) $(E1_KOKEN) $(RESULTS_E1)/phase2_eskf_consistency.png
