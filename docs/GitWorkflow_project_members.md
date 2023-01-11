# Utilisation de Git avec Gitlab

Voici un tuto pour utiliser Git avec Gitlab dans ce projet avec les commandes associées.

## Mise en place

Pour mettre en place le workflow Git avec Gitlab :
- Créer un `fork` du dépôt et ajouter chaque membre
- Cloner ensuite le dépôt _forké_ sur un ordinateur à l'aide de la commande `git clone` pour en obtenir une copie locale
- Créer une branche de développement (`git checkout -b MA_BRANCH`), **il est important de ne pas travailler sur la branche `main` du _fork_ **


## Utilisation

Cette partie est dédiée à la réalisation d'une `Pull Request` pour intégrer des changements locaux sur le dépôt du groupe.

### 1. Valider les changements
Se positionner sur sa propre branche de développement. 
```sh
git checkout MA_BRANCH
```
Faire les modifications souhaitées puis regarder le status de git localement.
```sh
git status
```
Ajouter les modifications et suivre les fichiers créés.
```sh
git add FILES
```
Enregistrer les modifications localement.
```sh
git commit -m "COMMENT"
```

Pousser les modifs sur sa branche distante

```sh
git push origin MA_BRANCH
```

### 2. Intégrer les changements

On souhaite intégrer les changements effectués avec les dernières mises à jour du dépôt du groupe.  

Mettre à jour sa branche `main` locale

```sh
git checkout main
git pull origin main # Il ne doit pas y avoir de conflit ici !!
```

On va créer une nouvelle branche à partir de sa propre branche de développement qui sera dédiée à l'intégration des changements

_Utile pour voir les problèmes d'intégration sans casser ses fonctionnalités déjà développées_

```sh
git checkout MA_BRANCH
git checkout -b MON_INTEGRATION # crée une nouvelle branche locale et nous positionne dessus
```
Intégrer ensuite les `commits` de la branche `main` à nos changements.
```sh
git rebase main 
```
Cette étape peut entraîner des conflits, il est nécessaire de les résoudre pour poursuivre.

### 3. Sauvegarde ses modifications

Une fois les conflits résolus, la branche `MON_INTEGRATION` est la plus à jour ! On va la renommer pour qu'elle devienne notre nouvelle branche de développement.

```sh
git branch -D MA_BRANCHE # Supprime ma branche
git branch -m MON_INTEGRATION MA_BRANCHE # renomme la branche MON_INTEGRATION en MA_BRANCHE
```

Sauvegarder les modifications sur la branche de développement du dépôt _forké_.
```sh
git push origin MA_BRANCHE # Il faudra peut-être mettre l'option -f pour force push
```


### 4. Demander l'intégration sur la branche `main`

Faire une demande de Merge Request depuis votre branche `MA_BRANCHE` sur la branche `main` 

_ou bien demander au PO (à éviter)_

## Le PO

Gérer la merge request si elle est demandée

Une autre possibilité (si pas de merge request) pourrait être :

 Se mettre sur la branche `main`

```sh
git checkout main
```
Mettre à jour la branche `main` locale

```sh
git pull origin main # il ne doit pas y avoir de conflit si tout est bien fait !!
```

Merge la branche `MA_BRANCHE` sur la branche `main`

```sh
git fetch origin MA_BRANCHE # récupère les ajout à appliquer
# Si aucun conflit => on peut merge
git merge MA_BRANCHE
# Si il y un conflit => demander au développeur de fournir une branche d'intégration à jour
```

