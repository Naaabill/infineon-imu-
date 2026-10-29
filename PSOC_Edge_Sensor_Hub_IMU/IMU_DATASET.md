# Collecter des données IMU pour entraîner un modèle

Le mode par défaut utilise le petit classifieur entraîné depuis les trois captures. Pour enregistrer de nouvelles données, le mode collecte remplace temporairement l’inférence: il échantillonne l’IMU à 50 Hz et envoie une ligne CSV par échantillon sur l’UART.

## Activer la collecte

1. Dans `common.mk`, régler `IMU_DATASET_MODE ?= 0` sur `IMU_DATASET_MODE = 1`. Ce réglage partagé active le mode collecte dans les deux images. Le mode collecte est prioritaire sur l’inférence.
2. Dans `proj_cm33_ns/sensor_hub_daq_task.c`, régler `IMU_DATASET_LABEL` sur une classe: `"repos"`, `"mouvement"` ou `"saut"`.
3. Reconstruire et televerser l application multicore complete (CM33 NS et CM55) apres ce changement. La selection du proprietaire est partagee par `common.mk`; le defaut CM33 empeche les deux images de lancer la meme tache sur le BMI270.
4. Ouvrir le port serie dans Tera Term; les messages de demarrage precedent en-tete CSV. Ignorer les lignes qui precedent en-tete lors du traitement.
5. Dans Tera Term, démarrer la capture du journal vers un fichier, effectuer l’activité correspondant à l’étiquette pendant plusieurs essais, arrêter la capture et enregistrer un fichier différent pour chaque classe.

Enregistre les classes dans des fichiers distincts (`repos.csv`, `mouvement.csv`, `saut.csv`). La colonne `label` est aussi incluse dans chaque ligne, ce qui permet de réunir les fichiers ensuite.

## Conseils pour les mesures

- `repos`: pose la carte immobile dans plusieurs orientations et enregistre des périodes de quelques secondes.
- `mouvement`: déplace la carte de plusieurs façons réalistes, sans sauter.
- `saut`: enregistre plusieurs sauts séparés et les périodes de repos entre eux. Fixe solidement la carte; ne saute pas en tenant une carte fragile à la main.
- Fais plusieurs essais séparés pour chaque classe. Ne collecte pas uniquement une longue séquence identique.
- Le débit est de 50 échantillons/s (un échantillon toutes les 20 ms). Un saut étant bref, conserve un peu de données avant et après chaque saut.
- Les données d’accélération sont en m/s²; le gyroscope est en rad/s. Le temps `t_ms` est le compteur de temps du microcontrôleur, pas une date civile.

## Revenir au mode normal

Remettre `IMU_DATASET_MODE = 0` et `IMU_INFERENCE_MODE = 1` dans `common.mk`, puis reconstruire et téléverser l’application multicœur complète. Cela réactive le modèle embarqué. Pour revenir à l’ancien mode par interruption sans modèle, mettre aussi `IMU_INFERENCE_MODE = 0` dans `common.mk` et reconstruire toute l’application.

## Étape modèle

Après la collecte, vérifier les fichiers CSV, équilibrer le nombre d’exemples par classe et découper les séquences en fenêtres. On entraînera ensuite un petit classifieur avec ces données, puis on convertira ses paramètres pour l’inférence embarquée. Il faut garder des essais entiers de côté pour vérifier le modèle sur des mouvements qu’il n’a pas vus pendant l’entraînement.