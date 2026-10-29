# Modèle de mouvement IMU

Le script `train_model.py` entraîne un petit classifieur supervisé en Python standard (aucune bibliothèque externe requise) à partir de `windows_2s_binary.csv`, puis régénère `imu_motion_model.h` et `training_report.json`.

Le modèle binaire prédit `repos` ou `mouvement` sur 100 échantillons à 50 Hz (2 secondes). `saut` est regroupé dans la classe `mouvement`. Les deux caractéristiques apprises sont la variance de la norme d’accélération au carré et la moyenne de l’énergie du gyroscope au carré. Le seuil final est ajusté sur les 53 fenêtres disponibles.

Pour réentraîner après avoir remplacé ou enrichi le CSV: `python train_model.py`, puis copier le nouveau `imu_motion_model.h` vers `proj_cm33_ns/imu_motion_model.h` avant de reconstruire le projet multicœur.

Le rapport conserve une évaluation chronologique de 11 fenêtres: 10 sont correctes. C’est un petit prototype fondé sur une seule capture par fichier; ajoute plusieurs sessions, notamment des repos dans plusieurs orientations, avant de conclure sur sa fiabilité.
## Utiliser le modèle sur la carte

Le dépôt sélectionne le mode d’inférence par défaut dans `common.mk`: `IMU_DATASET_MODE=0`, `IMU_INFERENCE_MODE=1`, `SENSOR_CORE=CM33`. Reconstruire et programmer l’application multicœur complète pour que les images CM33 NS et CM55 partagent le même propriétaire et le même mode. Avec les outils ModusToolbox, lancer l’action Build/Program au niveau de l’application. Le terminal doit montrer une seule tâche DAQ, démarrant sur le CM33, puis les lignes `ML: repos` ou `ML: mouvement`. La classification arrive par fenêtres de 2 secondes après l’interruption de mouvement; après une fenêtre repos, le cœur attend de nouveau.

Pour utiliser le CM55 lors d’une comparaison, construire l’application entière avec `SENSOR_CORE=CM55`; ne lancer qu’un seul propriétaire à la fois. Le modèle doit être reconstruit sur les deux cœurs.

`train_model.py` réentraîne le classifieur depuis ce dossier. Après avoir remplacé/enrichi le CSV, lancer `python train_model.py`, puis copier le nouvel en-tête `imu_motion_model.h` dans `proj_cm33_ns/` avant le build.
