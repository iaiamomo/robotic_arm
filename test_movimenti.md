# Test dei movimenti del Braccio++

Come muovere il braccio a mano, con comandi seriali, e come passare da una posizione sul tavolo agli angoli da inviare.

## 1. Come inviare i comandi

- **Serial monitor dell'Arduino IDE:** 115200 baud, fine riga "Newline".
- **Da terminale**, solo per la pinza: `python -m braccio_ns --port COM3 grip 200 150 200`.

La porta seriale può essere aperta da un solo programma alla volta. Chiudi la GUI prima di usare il serial monitor, e viceversa.

Il firmware risponde `OK: ...` oppure `ERROR: ...`. Se aggiunge ` (clamped)`, un valore era fuori dai limiti ed è stato ridotto al limite.

## 2. I tre comandi

| Comando | Cosa fa |
|---|---|
| `HOME` | Va nella posizione home `160 150 157.5 157.5 157.5 95`. **Muove anche la pinza a 160.** |
| `GRIP g` | Muove **solo** la pinza all'angolo `g`. Il resto del braccio resta fermo. |
| `MOVE g r p e s b` | Muove tutti e 6 i giunti insieme. Gli angoli sono in gradi, in quest'ordine: |

| Pos. | Lettera | Giunto | Limiti firmware | Note |
|---|---|---|---|---|
| 1 | `g` | pinza (gripper) | 120 – 230 | **200 = tutta aperta, 150 = stretta.** Non usare 120 o 230: il servo va in blocco e serve spegnere e riaccendere. |
| 2 | `r` | rotazione polso (roll) | 0 – 315 | Fissa a **150** in tutto il codice. |
| 3 | `p` | inclinazione polso (pitch) | 80 – 230 | 157.5 = in linea con l'avambraccio |
| 4 | `e` | gomito (elbow) | 75 – 225 | 157.5 = in linea con il braccio |
| 5 | `s` | spalla (shoulder) | 60 – 250 | 157.5 = braccio dritto in su |
| 6 | `b` | base | 0 – 315 | **95 = dritto verso la griglia** (direzione della HOME). Angoli maggiori ruotano verso sinistra: 140 è circa 45° a sinistra. 1° di comando = 1° reale. |

Con tutti i giunti a 157.5, il braccio è dritto in verticale. La HOME è proprio questa posizione, con la base girata verso la griglia.

## 3. Le coordinate

Il codice ragiona in metri, nel **sistema del robot**:

```
            ▲ x  (avanti, verso la griglia = direzione della HOME)
            │
   y ◄──────●  origine: centro della base, sul piano del tavolo
 (sinistra)        z: verso l'alto (z = 0 è il tavolo)
```

**La griglia** (3 righe × 5 colonne, `CELL_riga_colonna`) vista dall'alto:

```
             col 4    col 3    col 2    col 1    col 0
           ┌────────┬────────┬────────┬────────┬────────┐
  riga 2   │CELL_2_4│CELL_2_3│CELL_2_2│CELL_2_1│CELL_2_0│  x = 36 cm
           ├────────┼────────┼────────┼────────┼────────┤
  riga 1   │CELL_1_4│CELL_1_3│CELL_1_2│CELL_1_1│CELL_1_0│  x = 31 cm
           ├────────┼────────┼────────┼────────┼────────┤
  riga 0   │CELL_0_4│CELL_0_3│CELL_0_2│CELL_0_1│CELL_0_0│  x = 26 cm
           └────────┴────────┴───┬────┴────────┴────────┘
    y =      +12      +6       0 │     −6       −12   (cm, centri delle celle)
                                 │  direzione HOME
                               [base]
```

Le celle sono 5 cm in profondità e 6 cm in larghezza. I cubi sono da 3 cm: un cubo appoggiato sul tavolo ha il centro a z = 1.5 cm, uno sopra un altro cubo a z = 4.5 cm.

### Da coordinate ad angoli: il comando `ik`

`MOVE` vuole angoli, non coordinate. Il comando `ik` calcola il `MOVE` per una cella o un punto, senza muovere il braccio:

```bash
python -m braccio_ns ik CELL_0_2 --above         # 6 cm sopra il cubo in CELL_0_2 (avvicinamento)
python -m braccio_ns ik CELL_0_2                 # punto di presa del cubo sul tavolo in CELL_0_2
python -m braccio_ns ik CELL_2_2 --level 2       # cubo al 2° livello (sopra un altro) in CELL_2_2
python -m braccio_ns ik 0.30 0 0.10              # punto qualsiasi: x y z in metri
python -m braccio_ns ik CELL_0_2 --grip 150      # stessa posa con la pinza stretta
```

Esempio di output:
```
CELL_0_2, level 1 (grasp): x=0.260 y=0.000 z=0.015 m, gripper tilt 24 deg
MOVE 200.0 150.0 226.4 217.0 185.1 95.0
```
Copia la riga `MOVE` nel serial monitor. Se il punto è fuori portata, il comando lo dice ("unreachable"). Il braccio arriva a tutte le altezze di una pila solo tra circa 25 e 39 cm dalla base.

## 4. Esempio: prendere e posare un cubo a mano

Cubo in `CELL_0_2`, da portare in `CELL_1_4`. È la stessa sequenza che usa il programma:

```
HOME
GRIP 200                                  ← mano tutta aperta, braccio fermo
MOVE 200 150 228.0 217.8 170.7 95.0       ← sopra il cubo (ik CELL_0_2 --above)
MOVE 200 150 226.4 217.0 185.1 95.0       ← scende sul cubo (ik CELL_0_2)
GRIP 150                                  ← stringe
MOVE 150 150 228.0 217.8 170.7 95.0       ← risale col cubo
MOVE 150 150 157.5 157.5 157.5 95.0       ← torna in home tenendo il cubo (non usare HOME: aprirebbe la mano)
MOVE 150 150 226.5 181.4 200.6 116.2      ← sopra CELL_1_4 (ik CELL_1_4 --above --grip 150)
MOVE 150 150 229.8 173.0 215.7 116.2      ← scende in CELL_1_4 (ik CELL_1_4 --grip 150)
GRIP 200                                  ← rilascia
MOVE 200 150 226.5 181.4 200.6 116.2      ← risale (ik CELL_1_4 --above)
MOVE 200 150 157.5 157.5 157.5 95.0       ← torna in home con la mano aperta
```

Questi valori dipendono dalle lunghezze dei link nell'URDF e dalla configurazione attuale. Rigenerali sempre con `ik` invece di copiarli da qui.

## 5. Da verificare: il verso di spalla e gomito

Il codice assume che, per spalla, gomito e polso, **angoli sopra 157.5 pieghino il braccio in avanti** (verso la griglia), con `joint_signs` = 1 in `config/default.json`. Per questo i `MOVE` calcolati da `ik` hanno spalla e gomito sopra 157.5.

Il tuo esempio qui sotto ("abbassarsi in avanti" con spalla e gomito a **140**) farebbe pensare al contrario. Prova un giunto alla volta, partendo dalla HOME:

```
MOVE 200 150 157.5 157.5 175 95           ← solo la spalla a 175: il braccio va avanti o indietro?
MOVE 200 150 157.5 175 157.5 95           ← solo il gomito a 175
MOVE 200 150 175 157.5 157.5 95           ← solo il polso a 175
```

Se un giunto va **indietro** (verso il lato opposto alla griglia), il suo `joint_signs` va messo a `-1` in `config/default.json`. Altrimenti i movimenti calcolati vanno nella direzione sbagliata.

## 6. Esempi originali

Posizione HOME scritta come MOVE (pinza a 160, come la HOME del firmware):

    MOVE 160 150 157.5 157.5 157.5 95

Ruotare solo la base di circa 45° verso sinistra:

    MOVE 160 150 157.5 157.5 157.5 140

Abbassarsi un po' in avanti (vedi il punto 5 per il verso di spalla e gomito):

    MOVE 160 150 157.5 140 140 95
