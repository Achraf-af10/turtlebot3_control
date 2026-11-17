# Modèle, commande et portée des résultats

Le point contrôlé est la projection au sol du milieu de l'essieu. Le repère
`base_footprint` du Burger standard convient à ce modèle ; une autre définition
de la pose exige de transformer position et vitesse. On suppose le roulement
sans glissement, un sol plan et des mesures exprimées dans les repères annoncés.
La détection de valeurs invalides ne vérifie pas physiquement ces hypothèses.

## Partie cinématique retenue

Pour la pose mesurée $(x,y,\theta)$ et la référence $(x_r,y_r,\theta_r)$ :

$$
e_1=\cos\theta(x_r-x)+\sin\theta(y_r-y),\qquad
 e_2=-\sin\theta(x_r-x)+\cos\theta(y_r-y),\qquad
 e_3=\operatorname{atan2}(\sin(\theta_r-\theta),\cos(\theta_r-\theta)).
$$

La loi nominale discutée à partir du papier est :

$$
v_c=v_r\cos e_3+k_1 e_1,\qquad
\omega_c=\omega_r+k_2v_re_2+k_3\sin e_3,
\qquad k_1,k_2,k_3>0.
$$

Le code conserve cette loi dans `nominal_command`, puis applique séparément les
limites matérielles et la rampe de commande. Il n'ajoute pas de terme de mode
glissant arbitraire sur `cmd_vel`. La boucle dynamique du papier attend une
entrée de couple, absente du Burger standard : elle reste non réalisée.

Sous le modèle cinématique, les erreurs vérifient exactement :

$$
\dot e_1=-v+\omega e_2+v_r\cos e_3,\qquad
\dot e_2=-\omega e_1+v_r\sin e_3,\qquad
\dot e_3=\omega_r-\omega.
$$

Avec $L=(e_1^2+e_2^2)/2+(1-\cos e_3)/k_2$ et les **vitesses réalisées** $v,\omega$,
la différentiation donne :

$$
\boxed{\dot L=-k_1e_1^2-\frac{k_3}{k_2}\sin^2e_3
-e_1(v-v_c)-\frac{\sin e_3}{k_2}(\omega-\omega_c).}
$$

`Ldot_ideal` est la somme des deux premiers termes. Les deux autres disparaissent
uniquement si $v=v_c$ et $\omega=\omega_c$. `Ldot_kinematic_measured` utilise les
vitesses de l'odométrie pour évaluer l'identité complète. C'est une estimation
conditionnée au modèle et à la qualité/synchronisation des mesures ; ce n'est
pas une mesure indépendante de la dérivée réelle.

La négativité semi-définie de la dérivée nominale ne suffit pas, seule, à prouver
la convergence des trois erreurs : $e_2$ n'y figure pas directement. Il faut une
analyse supplémentaire du système en boucle fermée, des conditions sur la
référence et une restriction du domaine angulaire. Pour une référence variable,
on ne peut pas invoquer sans adaptation le théorème autonome de LaSalle.
L'erreur angulaire est une variable sur le cercle ; aucune stabilité globale
dans une coordonnée angulaire réelle unique n'est revendiquée ici.

Les saturations, la discrétisation, les délais, la quantification et le suivi
imparfait des moteurs produisent les termes résiduels. Leur présence empêche de
transférer automatiquement la preuve nominale au robot. Les tests vérifient
l'identité mathématique et un contre-exemple où une saturation rend $\dot L>0$ ;
ils ne démontrent pas la robustesse du système réel.

## Trajectoires

Le cercle est tangent à l'orientation initiale et passe par la position au moment
du `start`. Le huit utilise, avant rotation et translation :

$$
X=a\sin(\nu t),\qquad Y=b\sin(2\nu t),\qquad
\nu=\frac{v_{\max}}{\sqrt{a^2+4b^2}}.
$$

La norme de la vitesse est au plus $v_{\max}$. Pour $a,b>0$, les deux composantes
de la dérivée ne s'annulent jamais simultanément. Le passage au croisement ne
crée donc pas de division par zéro. L'orientation et la vitesse angulaire sont
calculées analytiquement :

$$
\theta_r=\operatorname{atan2}(\dot Y,\dot X),\quad
v_r=\sqrt{\dot X^2+\dot Y^2},\quad
\omega_r=\frac{\dot X\ddot Y-\dot Y\ddot X}{\dot X^2+\dot Y^2}.
$$

La rotation initiale est $\theta_0-\operatorname{atan2}(2b,a)$ ; elle aligne la
tangente initiale avec le robot. La courbe entière est tournée, pas seulement
son orientation.

Au démarrage, on remplace le temps de parcours par
$s(t)=t-T(1-e^{-t/T})$, d'où $\dot s=1-e^{-t/T}$. La position est évaluée en
$s(t)$ et **les deux vitesses** sont multipliées par $\dot s$. La cohérence
cinématique est ainsi conservée, avec des vitesses initiales nulles. Une hypothèse
$v_r\ge v_{\min}>0$ ne vaut donc pas à l'instant initial de cette rampe.

## Limites de commande et expérimentation

Avec $b$ la séparation des roues, les vitesses tangentielles demandées sont
$v_L=v-b\omega/2$ et $v_R=v+b\omega/2$. Le limiteur applique une même réduction
aux deux commandes pour satisfaire simultanément les bornes sur $v$, $\omega$,
$v_L$ et $v_R$. Il interpole ensuite entre l'ancienne commande et cette cible
pour borner les variations de $v$ et $\omega$. Le domaine de vitesse étant
convexe, ce segment reste admissible si la commande précédente l'était.

Il s'agit de limites **sur les consignes**, pas de bornes prouvées sur
l'accélération réelle, le courant ou le couple. Les arrêts de surveillance
contournent volontairement cette interpolation et demandent immédiatement zéro.

Une petite dimension de huit ou un petit rayon peut rendre la vitesse angulaire
de référence irréalisable. Les sorties restent limitées, mais cela ne garantit
plus le suivi. Surveiller les saturations et diminuer la vitesse de référence,
augmenter les dimensions ou revoir les gains à partir des mesures. Le seuil
d'erreur configurable arrête l'essai au lieu de poursuivre une référence devenue
hors du domaine expérimental.

Pour reproduire ultérieurement la partie dynamique en couple, il faudra disposer
d'une interface physique adaptée et caractérisée, identifier ses paramètres et
ses limites, puis revoir la démonstration avec les erreurs de la boucle interne.
Le présent dépôt ne remplace pas ces étapes par une conversion non vérifiée
couple-vers-vitesse ou couple-vers-PWM.
