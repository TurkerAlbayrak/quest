# QUEST — CubeSat Yönelim (Attitude) Kestirim Algoritması

> **QUaternion ESTimator (QUEST)** algoritmasının Wahba Problemi'ne dayalı
> matematiksel türetimi, referans Python implementasyonu, uçuş yazılımına
> yönelik tahsissiz (allocation-free) C implementasyonu ve doğrulama testleri.

[![Tests](https://img.shields.io/badge/tests-13%20passing-brightgreen)](tests/test_quest.py)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python](https://img.shields.io/badge/python-3.9%2B-blue)](src/quest.py)
[![C](https://img.shields.io/badge/C-C99-blue)](src/quest_embedded.c)

---

## İçindekiler

- [Bu proje ne işe yarar?](#bu-proje-ne-işe-yarar)
- [QUEST algoritması kısaca](#quest-algoritması-kısaca)
- [Depo yapısı](#depo-yapısı)
- [Kurulum](#kurulum)
- [Hızlı başlangıç](#hızlı-başlangıç)
- [Teknik PDF dokümanı](#teknik-pdf-dokümanı)
- [Python API referansı](#python-api-referansı)
- [Gömülü (C) implementasyon](#gömülü-c-implementasyon)
- [Testler](#testler)
- [Örnekler](#örnekler)
- [Sayısal doğruluk ve sınırlamalar](#sayısal-doğruluk-ve-sınırlamalar)
- [Sıkça sorulan sorular](#sıkça-sorulan-sorular)
- [Katkıda bulunma](#katkıda-bulunma)
- [Kaynakça](#kaynakça)
- [Lisans](#lisans)

---

## Bu proje ne işe yarar?

CubeSat'lar gibi küçük uydularda Yönelim Belirleme ve Kontrol Sistemi
(**ADCS — Attitude Determination and Control System**), uydunun üç eksende
uzaydaki yönelimini bilmesini gerektirir. Bu, güneş panellerinin Güneş'e
yönlendirilmesi, anten/kamera işaretleme (pointing) ve manyetik
detumbling gibi neredeyse her görev için ön koşuldur.

**QUEST**, güneş sensörü, manyetometre, yıldız izleyici gibi birden fazla
sensörden alınan **vektör ölçümlerini** (gövde çerçevesinde) bilinen
referans modelleriyle (eylemsizlik çerçevesinde: Güneş efemerisi, IGRF
manyetik alan modeli, yıldız kataloğu) eşleştirerek, bu ölçümlere en
iyi uyan (en küçük kareler anlamında optimal) yönelim **quaternion**'unu
hesaplayan klasik bir algoritmadır (Shuster & Oh, 1981).

Bu depo şunları içerir:

| Bileşen | Açıklama |
|---|---|
| 📄 `docs/quest_algorithm.pdf` | LaTeX ile hazırlanmış, ~9 sayfalık ayrıntılı teknik doküman (türetim, algoritma, sayısal örnek, gömülü sistem notları) |
| 🐍 `src/quest.py` | NumPy tabanlı, okunabilir referans Python implementasyonu |
| ⚙️ `src/quest_embedded.c` | Uçuş yazılımı için tahsissiz, sabit-iterasyonlu C99 implementasyonu |
| 🧪 `tests/test_quest.py` | Gürültüsüz/gürültülü senaryolar, sınır durumları ve Monte Carlo testleri (pytest) |
| 📊 `examples/` | Güneş sensörü + manyetometre senaryosu, TRIAD karşılaştırması, gürültü-doğruluk analizi grafiği |

---

## QUEST algoritması kısaca

Aşağıdaki akış, tam türetimi içeren `docs/quest_algorithm.pdf` dosyasının
özetidir:

1. **Girdi:** $n \geq 2$ referans vektörü çifti: gövde çerçevesinde ölçülen
   $\vect{b}_i$ ve eylemsizlik çerçevesindeki bilinen karşılığı $\vect{r}_i$.
2. **Wahba Problemi:** Bu ölçümlere en iyi uyan dönme matrisini, ağırlıklı
   en küçük kareler anlamında bulmak:
   $J(\mat A)=\tfrac12\sum_i a_i\|\vect b_i-\mat A\vect r_i\|^2 \to \min$
3. **Öznitelik matrisi:** $\mat B=\sum_i a_i\,\vect b_i\vect r_i^T$
4. **Davenport $K$-matrisi:** $\sigma,\ \mat S=\mat B+\mat B^T,\ \vect z$'den
   kurulan $4\times4$ simetrik matris.
5. **Newton-Raphson:** $K$'nin en büyük özdeğeri $\lambda_{\max}$,
   $\lambda_0=\sum_i a_i$ başlangıcından itibaren tipik olarak **1–3
   iterasyonda** bulunur (tam özayrışım gerekmez — bu QUEST'in hız sırrıdır).
6. **Gibbs vektörü:** $3\times3$'lük doğrusal bir sistem çözülerek
   Rodrigues parametreleri elde edilir.
7. **Quaternion:** Normalize edilerek optimal $\vect q_{\text{opt}}$ elde edilir.

Ayrıntılı türetim, karakteristik polinom katsayıları, $180°$ tekillik durumu
ve gömülü sistem notları için **[docs/quest_algorithm.pdf](docs/quest_algorithm.pdf)**
dosyasına bakın.

---

## Depo yapısı

```
quest-cubesat/
├── README.md                       # Bu dosya
├── LICENSE                         # MIT lisansı
├── requirements.txt                # Python bağımlılıkları
├── .gitignore
│
├── docs/
│   ├── quest_algorithm.tex         # LaTeX kaynak dosyası
│   ├── quest_algorithm.pdf         # Derlenmiş teknik doküman (9 sayfa)
│   └── Makefile                    # `make` ile yeniden derleme
│
├── src/
│   ├── quest.py                    # Referans Python implementasyonu
│   └── quest_embedded.c            # Gömülü sistem (C99) implementasyonu
│
├── examples/
│   ├── sun_magnetometer_example.py # Gerçekçi 2-sensör senaryosu
│   ├── triad_vs_quest.py           # TRIAD ile karşılaştırma
│   ├── monte_carlo_accuracy.py     # Gürültü-doğruluk analizi + grafik
│   └── monte_carlo_accuracy.png    # Üretilen örnek grafik
│
└── tests/
    └── test_quest.py               # pytest birim testleri (13 test)
```

---

## Kurulum

### Python implementasyonu

```bash
git clone <bu-repo-url>
cd quest-cubesat
python3 -m venv .venv && source .venv/bin/activate   # (opsiyonel ama önerilir)
pip install -r requirements.txt
```

Gereksinimler: Python ≥ 3.9, NumPy, (testler için pytest, grafikler için
matplotlib).

### C implementasyonu

Herhangi bir C99 uyumlu derleyici yeterlidir; harici bağımlılık yoktur
(sadece standart `math.h`):

```bash
cd src
cc -O2 -DQUEST_TEST_MAIN quest_embedded.c -o quest_test -lm
./quest_test
```

> **Not:** `-lm` bayrağını (matematik kütüphanesi) kaynak dosyadan **sonra**
> vermeyi unutmayın; aksi halde bazı bağlayıcılarda (linker) `undefined
> reference to sqrt` gibi hatalar alınabilir.

### LaTeX dokümanını yeniden derlemek

```bash
cd docs
make            # ya da: pdflatex quest_algorithm.tex && pdflatex quest_algorithm.tex
```

TeX Live 2023+ (veya `amsmath`, `tikz`, `algorithm`, `algpseudocode`,
`booktabs`, `hyperref` paketlerini içeren herhangi bir dağıtım) yeterlidir.

---

## Hızlı başlangıç

```python
import numpy as np
from src.quest import quest

# Eylemsizlik çerçevesindeki referans vektörleri (örn. Güneş yönü, manyetik alan)
r = np.array([
    [1.0, 0.0, 0.0],   # Güneş efemerisinden
    [0.0, 0.0, 1.0],   # IGRF manyetik alan modelinden
])

# Gövde çerçevesinde ölçülen karşılıkları (sensörlerden, normalize edilmiş)
b = np.array([
    [0.866, 0.500, 0.002],
    [0.001, -0.002, 0.9999],
])

# Sensör güvenilirliğine göre ağırlıklar (isteğe bağlı; varsayılan eşit ağırlık)
weights = [0.7, 0.3]

result = quest(b, r, weights)

print("Optimal quaternion :", result.quaternion)   # [qx, qy, qz, qw]
print("lambda_max          :", result.lambda_max)
print("Wahba loss (J_min)  :", result.loss)
print("Newton-Raphson iters:", result.n_iterations)
```

Çıktı:

```
Optimal quaternion : [0.       0.       0.258819 0.965926]
lambda_max          : 0.9999999999999999
Wahba loss (J_min)  : 1.11e-16
Newton-Raphson iters: 1
```

---

## Teknik PDF dokümanı

**[📄 docs/quest_algorithm.pdf](docs/quest_algorithm.pdf)** aşağıdaki
başlıkları kapsar:

1. Giriş — CubeSat ADCS'te sensörler ve yönelim belirleme ihtiyacı
2. Problem Formülasyonu — Wahba Problemi
3. Davenport'un $K$-Matrisi ve Karakteristik Denklem
4. QUEST — Newton-Raphson ile Hızlı Çözüm
5. Optimal Quaternion'un Çıkarılması (Gibbs vektörü, $180°$ tekilliği)
6. Algoritmanın Tüm Akışı (pseudocode + akış şeması)
7. Sayısal Örnek
8. Gömülü Sistemde İmplementasyon Notları
9. QUEST ile Diğer Yöntemlerin Karşılaştırması (TRIAD, Davenport $q$-metodu, SVD, ESOQ, FOAM)
10. Kaynakça

Doküman LaTeX ile (`amsmath`, `algorithm`, `tikz`) hazırlanmıştır; kaynak
dosya `docs/quest_algorithm.tex` içinde düzenlenebilir haldedir.

---

## Python API referansı

### `quest(body_vectors, ref_vectors, weights=None, tol=1e-12, max_iter=25) -> QuestResult`

Ana fonksiyon. En az 2 vektör çifti gerektirir.

**Parametreler**
- `body_vectors`: `(n, 3)` — gövde çerçevesinde ölçülen (normalize
  edilmemiş olsa da olur; fonksiyon içeride normalize eder) birim vektörler.
- `ref_vectors`: `(n, 3)` — aynı vektörlerin eylemsizlik çerçevesindeki
  bilinen karşılıkları.
- `weights`: `(n,)`, opsiyonel — göreli güven ağırlıkları (örn. ters
  varyans). `None` ise eşit ağırlık kullanılır.
- `tol`, `max_iter`: Newton-Raphson yakınsama toleransı ve maksimum
  iterasyon sayısı.

**Döndürür:** `QuestResult` — alanlar: `quaternion` (`[qx,qy,qz,qw]`,
skaler-son), `lambda_max`, `loss` (Wahba minimum kaybı), `n_iterations`,
`converged`.

**Hatalar:** `QuestError` — 2'den az vektör verildiğinde, referans
vektörleri (neredeyse) paralel/ters-paralel olduğunda (gözlemlenemeyen
eksen), veya Newton-Raphson yakınsamadığında fırlatılır.

### `quest_sequential(body_vectors, ref_vectors, weights=None, **kwargs) -> QuestResult`

`quest()`'in etrafındaki sağlam (robust) sarmalayıcı; dönme açısı
$180°$'ye çok yakın olduğunda ortaya çıkan Gibbs-vektör tekilliğini,
referans çerçevesini $90°$ döndürüp iki çözümden daha iyi olanı seçerek
aşar. Belirsiz durumlarda `quest()` yerine bunu kullanmanız önerilir.

### `quat_to_dcm(q) -> np.ndarray`

Skaler-son ($[q_x,q_y,q_z,q_w]$) birim quaternion'u $3\times3$ yön kosinüs
matrisine (DCM) çevirir.

### Quaternion konvansiyonu

Bu depoda tutarlı olarak **skaler-son** ($[q_x, q_y, q_z, q_w]$) ve
**eylemsizlikten gövdeye** ($\vect b = \mat A(\vect q)\,\vect r$) konvansiyonu
kullanılır. Farklı bir konvansiyon (örn. skaler-ilk, JPL vs. Hamilton)
kullanan bir sistemle entegre ediyorsanız, dönüştürme yaparken bu detaya
dikkat edin.

---

## Gömülü (C) implementasyon

`src/quest_embedded.c`, uçuş yazılımı (flight software) için özel olarak
tasarlanmıştır:

- **Heap tahsisi yok** — `malloc`/`free` kullanılmaz; tüm bellek yığın
  (stack) üzerinde veya çağıran tarafından sağlanır.
- **İstisna (exception) yok** — hatalar `quest_status_t` dönüş kodlarıyla
  bildirilir.
- **Sabit maksimum iterasyon** — Newton-Raphson `QUEST_MAX_NR_ITERS` (25)
  ile sınırlıdır; en kötü durum zamanlaması öngörülebilir (WCET analizi
  için uygundur).
- **Yapılandırılabilir hassasiyet** — `QUEST_REAL` tipi `float` (varsayılan,
  32-bit MCU'lar için) veya `double` olarak değiştirilebilir.

```c
#include "quest_embedded.c"   // ya da ayrı derleyip başlık dosyası çıkarın

quest_vec3_t body[2] = { {{0.866f, 0.500f, 0.0f}}, {{0.0f, 0.0f, 1.0f}} };
quest_vec3_t ref[2]  = { {{1.0f, 0.0f, 0.0f}},     {{0.0f, 0.0f, 1.0f}} };
QUEST_REAL weights[2] = {0.7f, 0.3f};

quest_result_t result;
quest_status_t status = quest_solve(body, ref, weights, 2, &result);

if (status == QUEST_OK) {
    // result.quaternion.{x,y,z,w} kullanıma hazır
} else {
    // status: QUEST_ERR_* kodlarından biri; bkz. enum quest_status_t
}
```

### Hata kodları

| Kod | Anlamı |
|---|---|
| `QUEST_ERR_TOO_FEW_VECTORS` | 2'den az vektör çifti verildi |
| `QUEST_ERR_TOO_MANY_VECTORS` | `QUEST_MAX_VECTORS` sınırı aşıldı |
| `QUEST_ERR_PARALLEL_REFERENCE` | Referans vektörleri (neredeyse) paralel/ters-paralel |
| `QUEST_ERR_SINGULAR_GIBBS` | $180°$ tekilliğine yakın; sıralı QUEST gerekli |
| `QUEST_ERR_NR_NOT_CONVERGED` | Newton-Raphson `QUEST_MAX_NR_ITERS` içinde yakınsamadı |
| `QUEST_ERR_ZERO_DERIVATIVE` | Newton-Raphson türevi sıfırlandı (dejenere durum) |

### Farklı bir MCU hedefine taşıma

- Tek hassasiyet (`float`) varsayılan olarak seçilmiştir; `QUEST_NR_TOL`
  buna göre `1e-6` ayarlanmıştır. `double`'a geçerseniz `1e-12` gibi daha
  sıkı bir tolerans kullanabilirsiniz.
  Dosyanın başındaki yorumlarda bu açıkça belirtilmiştir.
- `QUEST_MAX_VECTORS` (varsayılan 8), tipik bir CubeSat'ta eşzamanlı
  kullanılabilecek sensör sayısına göre ayarlanabilir; yığın kullanımını
  doğrudan etkiler.

---

## Testler

```bash
pip install -r requirements.txt
pytest tests/test_quest.py -v
```

Test kapsamı (13 test, hepsi geçiyor ✅):

- **Gürültüsüz doğrulama:** 2 ve çoklu-vektör senaryolarında analitik
  quaternion ile tam örtüşme (`< 1e-6°` hata).
- **Gürültülü senaryolar:** artan gürültü seviyelerinde (0.1°–2°) hatanın
  makul biçimde büyüdüğünün doğrulanması; 200 denemelik Monte Carlo ile
  ortalama hatanın sapmasız (unbiased) olduğunun kontrolü.
- **Sınır durumları:** tek vektör, paralel/ters-paralel referans
  vektörleri, boyut uyuşmazlığı → `QuestError` fırlatılmalı.
- **$180°$ tekilliği:** `quest_sequential()`'in bu durumda bile doğru
  sonucu verdiğinin doğrulanması.
- **İç tutarlılık:** kestirilen $\mat A$'nın gerçekten $\vect r_i$'yi
  $\vect b_i$'ye eşlediğinin bağımsız kontrolü.

C implementasyonunun kendi kendini test eden sürümü için:

```bash
cd src && cc -O2 -DQUEST_TEST_MAIN quest_embedded.c -o quest_test -lm && ./quest_test
```

---

## Örnekler

### 1. Güneş sensörü + manyetometre (`examples/sun_magnetometer_example.py`)

Tipik bir 1U/3U CubeSat'ta bulunan iki sensörle gerçekçi gürültü
seviyeleri (Güneş sensörü ~0.5°, manyetometre ~1°) altında çalışan uçtan
uca bir senaryo.

```bash
python examples/sun_magnetometer_example.py
```

### 2. TRIAD ile karşılaştırma (`examples/triad_vs_quest.py`)

Klasik 2-vektörlü TRIAD algoritmasıyla, 4 vektörü ağırlıklı biçimde
birleştiren QUEST'i aynı gürültü koşulları altında karşılaştırır ve
QUEST'in ek ölçümleri kullanarak sağladığı doğruluk kazanımını gösterir.

```bash
python examples/triad_vs_quest.py
```

Örnek çıktı:
```
 TRIAD  (2 vectors) : mean = 0.615 deg, std = 0.295 deg
 QUEST  (4 vectors) : mean = 0.506 deg, std = 0.248 deg
QUEST reduces mean attitude error by 17.8% by fusing all available measurements.
```

### 3. Gürültü-doğruluk analizi (`examples/monte_carlo_accuracy.py`)

Farklı sensör gürültü seviyelerinde ve farklı sayıda referans vektörüyle
(2, 4, 8) 300'er denemelik Monte Carlo simülasyonu çalıştırır ve
`monte_carlo_accuracy.png` grafiğini üretir.

```bash
python examples/monte_carlo_accuracy.py
```

![Monte Carlo doğruluk grafiği](examples/monte_carlo_accuracy.png)

---

## Sayısal doğruluk ve sınırlamalar

- QUEST, **tek anlık (single-frame)** bir kestirimcidir; zaman içinde
  filtreleme yapmaz. Sürekli yönelim takibi için çıktısı genellikle bir
  **MEKF (Multiplicative Extended Kalman Filter)** veya **UKF**'e ölçüm
  güncellemesi olarak beslenir.
- Referans vektörlerinin birbirine **paralel veya ters-paralel** olmaması
  gerekir (aksi halde ilgili eksen etrafındaki dönme gözlemlenemez);
  hem Python hem C implementasyonu bu durumu tespit edip hata fırlatır.
- Dönme açısı $180°$'ye çok yaklaştığında Gibbs vektör parametrizasyonu
  tekildir; bu durum için `quest_sequential()` (Python) kullanılmalı, C
  tarafında ise `QUEST_ERR_SINGULAR_GIBBS` kodu değerlendirilmelidir.
- 32-bit `float` ile çalışan gömülü hedeflerde, `1e-9` ve altı toleranslar
  ulaşılamaz; varsayılan tolerans (`1e-6`) buna göre seçilmiştir.

---

## Sıkça sorulan sorular

**S: QUEST ile Davenport $q$-metodu arasındaki fark nedir?**
Her ikisi de aynı $4\times4$ $K$-matrisinin en büyük özdeğerine karşılık
gelen özvektörü arar. Davenport metodu bunu tam bir özayrışımla (SVD/Jacobi
vb.) bulurken, QUEST tek bir Newton-Raphson iterasyon zinciriyle (genellikle
1–3 adım) neredeyse aynı sonuca çok daha ucuza ulaşır.

**S: Kaç referans vektörü gerekli?**
En az 2. Daha fazlası (örn. birden fazla yıldız izleyici ölçümü veya farklı
zamanlarda alınmış Güneş/manyetometre ölçümleri), ağırlıklandırma yoluyla
doğruluğu artırır (bkz. `examples/monte_carlo_accuracy.py`).

**S: TRIAD yerine neden QUEST?**
TRIAD yalnızca 2 vektör kullanabilir ve ek ölçümleri ağırlıklı biçimde
birleştiremez. QUEST, $n\geq2$ ölçümü istatistiksel olarak optimale yakın
biçimde birleştirir (bkz. `examples/triad_vs_quest.py`).

**S: Bu kodu gerçek bir CubeSat'ta uçurabilir miyim?**
`src/quest_embedded.c`, bu amaçla (heap'siz, sabit iterasyonlu) tasarlanmıştır,
ancak uçuşa hazır (flight-ready) yazılım için sensör kalibrasyonu, hata
işleme stratejisi, birim/entegrasyon testleri ve görev-özel doğrulama
sizin sorumluluğunuzdadır. Bu depo eğitim/referans amaçlıdır.

---

## Katkıda bulunma

Katkılar memnuniyetle karşılanır. Lütfen:

1. Yeni bir özellik/düzeltme için bir issue açın veya mevcut birine katılın.
2. `pytest tests/ -v` ile tüm testlerin geçtiğinden emin olun.
3. C tarafında değişiklik yaptıysanız `-Wall -Wextra` ile derleyip
   uyarısız derlendiğini doğrulayın.
4. LaTeX dokümanında değişiklik yaptıysanız `docs/` içinde `make` ile
   yeniden derleyip PDF'in doğru göründüğünü kontrol edin.

## Kaynakça

1. G. Wahba, "A Least Squares Estimate of Satellite Attitude," *SIAM
   Review*, 7(3), p. 409, 1965.
2. M. D. Shuster and S. D. Oh, "Three-Axis Attitude Determination from
   Vector Observations," *Journal of Guidance and Control*, 4(1), pp.
   70–77, 1981.
3. P. B. Davenport, "A Vector Approach to the Algebra of Rotations with
   Applications," NASA Technical Note D-4696, 1968.
4. F. L. Markley and D. Mortari, "Quaternion Attitude Estimation Using
   Vector Observations," *Journal of the Astronautical Sciences*, 48(2),
   pp. 359–380, 2000.
5. F. L. Markley and J. L. Crassidis, *Fundamentals of Spacecraft
   Attitude Determination and Control*, Springer, 2014.

Ayrıntılı kaynakça için `docs/quest_algorithm.pdf` içindeki "Kaynakça"
bölümüne bakın.

## Lisans

Bu proje [MIT Lisansı](LICENSE) altında dağıtılmaktadır.
