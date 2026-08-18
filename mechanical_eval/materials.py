from dataclasses import dataclass


@dataclass(frozen=True)
class IsotropicMaterial:
    name: str
    density_kg_m3: float
    youngs_modulus_pa: float
    poissons_ratio: float
    damping: float
    elastic_strain_ceiling: float
    invalidity_strain: float
    calibration_note: str


# Conservative bulk values for an initial, isotropic PLA model. These are not
# universal filament properties; benchmark calibration coupons must eventually
# select a profile and slicer-derived orthotropic fields can replace this model.
PLA_BASELINE = IsotropicMaterial(
    name="effective isotropic printed-PLA v1",
    density_kg_m3=1200.0,
    youngs_modulus_pa=2.5e9,
    poissons_ratio=0.35,
    damping=0.02,
    elastic_strain_ceiling=0.02,
    invalidity_strain=0.05,
    calibration_note=("Literature-informed dense nominal-100%-infill profile. Calibrate density, modulus, "
                      "damping, friction, and clearance using coupons printed with the production recipe."),
)
