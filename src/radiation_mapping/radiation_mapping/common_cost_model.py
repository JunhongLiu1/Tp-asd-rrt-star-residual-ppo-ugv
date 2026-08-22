import json
import math
from pathlib import Path


class CommonCostModel:
    def __init__(self, config_path):
        self.config_path = Path(config_path).expanduser().resolve()

        if not self.config_path.is_file():
            raise FileNotFoundError(
                f"Cost configuration not found: {self.config_path}"
            )

        with self.config_path.open("r", encoding="utf-8") as file:
            self.config = json.load(file)

        normalization = self.config["normalization"]
        time_model = self.config["time_model"]
        time_extension = self.config["time_penalised_extension"]

        self.reference_length_m = float(
            normalization["reference_length_m"]
        )
        self.reference_time_s = float(
            normalization["reference_time_s"]
        )
        self.radiation_reference_usv_h = float(
            normalization["radiation_dose_rate_reference_usv_h"]
        )

        self.nominal_speed_m_s = float(
            time_model["nominal_speed_m_s"]
        )
        self.minimum_speed_factor = float(
            time_model["minimum_speed_factor"]
        )
        self.terrain_speed_penalty_gain = float(
            time_model["terrain_speed_penalty_gain"]
        )

        self.time_penalty_lambda = float(
            time_extension["time_penalty_lambda"]
        )

        self.profiles = self.config["base_cost_profiles"]

        self._validate()

    @staticmethod
    def clamp01(value):
        return max(
            0.0,
            min(
                1.0,
                float(value),
            ),
        )

    def _validate(self):
        positive_values = {
            "reference_length_m": self.reference_length_m,
            "reference_time_s": self.reference_time_s,
            "radiation_reference_usv_h":
                self.radiation_reference_usv_h,
            "nominal_speed_m_s": self.nominal_speed_m_s,
        }

        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be positive")

        if not 0.0 < self.minimum_speed_factor <= 1.0:
            raise ValueError(
                "minimum_speed_factor must be inside (0, 1]"
            )

        if self.terrain_speed_penalty_gain < 0.0:
            raise ValueError(
                "terrain_speed_penalty_gain cannot be negative"
            )

        if self.time_penalty_lambda < 0.0:
            raise ValueError(
                "time_penalty_lambda cannot be negative"
            )

        for name, profile in self.profiles.items():
            weights = [
                float(profile["distance_weight"]),
                float(profile["terrain_weight"]),
                float(profile["radiation_weight"]),
            ]

            if any(weight < 0.0 for weight in weights):
                raise ValueError(
                    f"Negative weight in profile: {name}"
                )

            if not math.isclose(
                sum(weights),
                1.0,
                rel_tol=0.0,
                abs_tol=1e-9,
            ):
                raise ValueError(
                    f"Weights in profile {name} sum to "
                    f"{sum(weights)}, not 1.0"
                )

    def profile_names(self):
        return tuple(self.profiles.keys())

    def estimate_speed_m_s(self, terrain_impedance):
        terrain_cost = self.clamp01(terrain_impedance)

        speed_factor = max(
            self.minimum_speed_factor,
            1.0
            - self.terrain_speed_penalty_gain
            * terrain_cost,
        )

        return self.nominal_speed_m_s * speed_factor

    def evaluate_edge(
        self,
        distance_m,
        terrain_impedance,
        dose_rate_usv_h,
        profile_name="balanced",
        include_time_penalty=False,
    ):
        distance_m = float(distance_m)
        dose_rate_usv_h = float(dose_rate_usv_h)

        if not math.isfinite(distance_m) or distance_m < 0.0:
            raise ValueError(
                "distance_m must be finite and non-negative"
            )

        if (
            not math.isfinite(dose_rate_usv_h)
            or dose_rate_usv_h < 0.0
        ):
            raise ValueError(
                "dose_rate_usv_h must be finite and non-negative"
            )

        if profile_name not in self.profiles:
            raise KeyError(
                f"Unknown cost profile: {profile_name}"
            )

        profile = self.profiles[profile_name]

        terrain_cost = self.clamp01(terrain_impedance)

        radiation_cell_cost = self.clamp01(
            dose_rate_usv_h
            / self.radiation_reference_usv_h
        )

        speed_m_s = self.estimate_speed_m_s(
            terrain_cost
        )

        traversal_time_s = (
            distance_m / speed_m_s
            if distance_m > 0.0
            else 0.0
        )

        predicted_dose_usv = (
            dose_rate_usv_h
            * traversal_time_s
            / 3600.0
        )

        distance_term = (
            distance_m
            / self.reference_length_m
        )

        terrain_term = (
            terrain_cost
            * distance_m
            / self.reference_length_m
        )

        radiation_term = (
            radiation_cell_cost
            * traversal_time_s
            / self.reference_time_s
        )

        time_term = (
            traversal_time_s
            / self.reference_time_s
        )

        base_cost = (
            float(profile["distance_weight"])
            * distance_term
            + float(profile["terrain_weight"])
            * terrain_term
            + float(profile["radiation_weight"])
            * radiation_term
        )

        time_penalty = (
            self.time_penalty_lambda * time_term
            if include_time_penalty
            else 0.0
        )

        total_cost = base_cost + time_penalty

        return {
            "profile_name": profile_name,
            "distance_m": distance_m,
            "terrain_impedance": terrain_cost,
            "dose_rate_usv_h": dose_rate_usv_h,
            "radiation_cell_cost": radiation_cell_cost,
            "estimated_speed_m_s": speed_m_s,
            "traversal_time_s": traversal_time_s,
            "predicted_dose_usv": predicted_dose_usv,
            "distance_term": distance_term,
            "terrain_term": terrain_term,
            "radiation_term": radiation_term,
            "time_term": time_term,
            "base_cost": base_cost,
            "time_penalty": time_penalty,
            "total_cost": total_cost,
        }
