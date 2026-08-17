from dataclasses import dataclass


@dataclass(frozen=True)
class IsotropicMaterial:
    name: str
    density_kg_m3: float
    youngs_modulus_pa: float
    poissons_ratio: float
    damping: float
    calibration_note: str


# Conservative bulk values for an initial, isotropic PLA model. These are not
# universal filament properties; benchmark calibration coupons must eventually
# select a profile and slicer-derived orthotropic fields can replace this model.
PLA_BASELINE = IsotropicMaterial(
    name="PLA isotropic baseline v1",
    density_kg_m3=1240.0,
    youngs_modulus_pa=2.3e9,
    poissons_ratio=0.36,
    damping=0.02,
    calibration_note="Conservative initial bulk profile; calibrate by printed tensile/torsion coupons.",
)
