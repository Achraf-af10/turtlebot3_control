from tracking_error import compute_tracking_error

# Robot et reference exactement au meme endroit
errors = compute_tracking_error(
    x=-2.0,
    y=-0.5,
    theta=0.0,
    x_ref=-2.0,
    y_ref=-0.5,
    theta_ref=0.0
)

print("Erreurs initiales :", errors)

# Robot decale de 0.2 m sur l'axe y, orientation nulle
errors = compute_tracking_error(
    x=-2.0,
    y=-0.5,
    theta=0.0,
    x_ref=-2.0,
    y_ref=-0.3,
    theta_ref=0.0
)

print("Erreurs avec decalage lateral :", errors)
