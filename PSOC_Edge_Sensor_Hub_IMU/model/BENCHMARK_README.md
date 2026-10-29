# Benchmark commun du classifieur IMU (sans réseau neuronal)

Objectif : comparer le même calcul C sur CM33, CM55 et, plus tard, sur un STM32.
Ce premier banc mesure le calcul du classifieur et de ses caractéristiques sur
des fenêtres IMU identiques, préenregistrées. Il ne lit pas l'IMU et n'écrit
pas sur l'UART dans la région chronométrée.

## Deux mesures à garder séparées

1. **Calcul seul** : rejouer le même fichier de fenêtres en mémoire flash, faire
   le calcul des caractéristiques puis la classification. Refaire le lot de
   nombreuses fois pour que la mesure soit assez longue. Rapporter cycles,
   temps, nombre de fenêtres et résultat/checksum.
2. **Système complet** : ensuite seulement, mesurer avec l'IMU, les interruptions,
   les attentes et le sommeil. Mesurer le courant avec le même instrument et la
   même batterie/alimentation. Ne pas inclure l'UART dans les mesures de calcul.

Le fichier `windows_2s_binary.csv` du modèle contient les mesures originales.
`generate_fixture.py` produit un en-tête C déterministe avec les 53 fenêtres
complètes sous forme d'entiers quantifiés. Le programme de benchmark doit lire
les nombres de cet en-tête et utiliser `imu_motion_model.h`, sans modifier les
seuils du classifieur. Garder le même en-tête de données et le même compilateur
(options d'optimisation documentées) sur chaque cible.

## À interpréter avec prudence

Le classifieur actuel est extrêmement léger : seulement deux comparaisons après
les statistiques. Ce test donne un premier point de comparaison CPU, pas une
mesure représentative de tous les modèles IA ni des accélérateurs Ethos-U55 ou
NNLite. Le résultat à comparer pour la batterie est l'énergie par fenêtre
(intégrale courant × temps), et non le seul courant instantané. Les fréquences
d'horloge, tensions et modes sommeil doivent être consignés.

## Données d'entrée

Le générateur prend le CSV du modèle en argument et écrit le fichier à l'endroit
choisi avec `--output`. Il regroupe chaque `window_id` et quantifie les six axes
en unités de 1 mg pour l'accéléromètre et 1 mrad/s pour le gyroscope. Unités,
échelle, ordre des axes et compte de fenêtres sont inscrits dans l'en-tête.

## PSoC Edge : lancement du benchmark

Le code est intégré au projet comme mode de compilation. Depuis un terminal
ModusToolbox ouvert dans le dossier racine du dépôt :

```sh
make clean build SENSOR_CORE=CM33 IMU_BENCHMARK_MODE=1
make program SENSOR_CORE=CM33 IMU_BENCHMARK_MODE=1
```

Puis refaire la même séquence avec `SENSOR_CORE=CM55`. Changer de cœur et de
mode de compilation exige de reconstruire l'application complète. Dans
l'IDE, régler `SENSOR_CORE` et `IMU_BENCHMARK_MODE` dans `common.mk`, puis faire
Build et Program au niveau de l'application entière. Laisser
`IMU_BENCHMARK_MODE=0` pour revenir au fonctionnement normal.

Le terminal affiche trois lignes `BENCH` par démarrage. Comparer les cycles ou
microsecondes par lot; les trois exécutions aident à repérer une mesure
perturbée. `correct` doit être égal à `53000` et le checksum doit être non nul.
Le lot exécute 53 000 fenêtres par ligne, soit 5,3 millions d'échantillons.

Pour la consommation, un ampèremètre seul ne donne pas l'énergie d'une
inférence. Mesurer le courant pendant le lot et sa durée, avec la même tension,
le même réglage d'horloge et la même carte. UART n'est utilisé qu'après chaque
chronométrage. Le chiffre est celui de la carte en fonctionnement dans cette
configuration; ce n'est pas une mesure isolée du silicium du cœur.
