"""
ROS-free logic for the buoy search: angle helpers, the particle filter, candidate
tracking, and the coverage map that plans the search legs. Kept out of
buoy_search_node.py so it can be unit tested without ROS or hardware.

All positions are UTM (easting, northing) in meters. All angles are degrees,
math convention (counter-clockwise from east) unless the name says otherwise.
"""

import math

import numpy as np


def angle_difference(a: float, b: float) -> float:
    """Return the smallest signed angle from b to a in degrees, in [-180, 180)."""
    return (a - b + 180) % 360 - 180


def compass_to_math(heading_deg: float) -> float:
    """Convert a compass heading (clockwise from north) to math convention (counter-clockwise from east)."""
    return (90 - heading_deg) % 360


class ParticleFilter:
    """
    Static-target particle filter over the buoy's position. Particles are seeded around
    the first measurement instead of the whole search area, so a few hundred particles
    are enough to cover the measurement's uncertainty.
    """

    def __init__(
        self,
        num_particles: int,
        center_x: float,
        center_y: float,
        spread: float,
        rng=None,
    ):
        self.rng = rng if rng is not None else np.random.default_rng()
        self.num_particles = num_particles
        self.xs = center_x + self.rng.normal(0.0, spread, num_particles)
        self.ys = center_y + self.rng.normal(0.0, spread, num_particles)
        self.weights = np.full(num_particles, 1.0 / num_particles)

    def update(
        self,
        bearing: float,
        boat_x: float,
        boat_y: float,
        observed_range: float,
        bearing_std: float,
        range_std: float,
        gate_sigmas: float = 3.0,
    ) -> bool:
        """
        Weight particles by a bearing + range measurement taken from (boat_x, boat_y).
        Returns False, without changing the filter, when no particle is within gate_sigmas
        of the measurement (the detection belongs to something else).
        """
        dx = self.xs - boat_x
        dy = self.ys - boat_y
        predicted_bearing = np.degrees(np.arctan2(dy, dx))
        bearing_error = (predicted_bearing - bearing + 180) % 360 - 180
        range_error = np.hypot(dx, dy) - observed_range

        sq_mahalanobis = (bearing_error / bearing_std) ** 2 + (
            range_error / range_std
        ) ** 2
        if np.min(sq_mahalanobis) > 2 * gate_sigmas**2:
            return False

        # Work in log space so weights never all underflow to zero
        log_w = np.log(self.weights) - 0.5 * sq_mahalanobis
        log_w -= np.max(log_w)
        w = np.exp(log_w)
        self.weights = w / np.sum(w)
        return True

    def resample(self, jitter_std: float):
        """Systematic resampling, then a small jitter to avoid particle deprivation."""
        positions = (
            self.rng.random() + np.arange(self.num_particles)
        ) / self.num_particles
        cumulative = np.cumsum(self.weights)
        cumulative[-1] = 1.0
        indices = np.searchsorted(cumulative, positions)
        self.xs = self.xs[indices] + self.rng.normal(
            0.0, jitter_std, self.num_particles
        )
        self.ys = self.ys[indices] + self.rng.normal(
            0.0, jitter_std, self.num_particles
        )
        self.weights = np.full(self.num_particles, 1.0 / self.num_particles)

    def estimate(self):
        """Weighted mean position (x, y)."""
        return float(np.sum(self.xs * self.weights)), float(
            np.sum(self.ys * self.weights)
        )

    def spread(self) -> float:
        """Root of the summed weighted variance in x and y, in meters."""
        x, y = self.estimate()
        var = np.sum(self.weights * ((self.xs - x) ** 2 + (self.ys - y) ** 2))
        return float(math.sqrt(var))


class Candidate:
    """A possible buoy: its filter plus the evidence gathered for it."""

    def __init__(
        self, filt: ParticleFilter, t: float, confidence: float, observed_range: float
    ):
        self.filter = filt
        self.first_seen = t
        self.last_seen = t
        self.last_update = t
        self.hits = 1
        self.confidence_sum = confidence
        self.last_range = observed_range
        self.close_hits = (
            0  # hits taken while the boat was within confirm range of the estimate
        )
        self.missed_visible_time = 0.0

    def add_hit(
        self, t: float, confidence: float, observed_range: float, close: bool = False
    ):
        self.last_seen = t
        self.hits += 1
        self.confidence_sum += confidence
        self.last_range = observed_range
        self.close_hits += close
        self.missed_visible_time = 0.0

    def estimate(self):
        return self.filter.estimate()

    def distance_to(self, x: float, y: float) -> float:
        cx, cy = self.estimate()
        return math.hypot(cx - x, cy - y)


class RejectedZone:
    """
    A place already investigated. Detections inside it are ignored until ignore_until, and
    after that a candidate there needs extra hits for every earlier strike, so the boat is not
    pulled back to the same false positive again and again.
    """

    def __init__(self, x: float, y: float, radius: float, ignore_until: float):
        self.x = x
        self.y = y
        self.radius = radius
        self.ignore_until = ignore_until
        self.strikes = 1

    def contains(self, x: float, y: float) -> bool:
        return math.hypot(self.x - x, self.y - y) <= self.radius


def measurement_to_point(
    boat_x: float, boat_y: float, bearing: float, observed_range: float
):
    """Project a global bearing (math convention) and range from the boat into a UTM point."""
    return (
        boat_x + observed_range * math.cos(math.radians(bearing)),
        boat_y + observed_range * math.sin(math.radians(bearing)),
    )


def range_std_for(
    observed_range: float, base_std: float, quadratic_std: float
) -> float:
    """Stereo depth error grows with the square of range."""
    return base_std + quadratic_std * observed_range**2


def point_visible(
    boat_x: float,
    boat_y: float,
    heading: float,
    x: float,
    y: float,
    hfov_deg: float,
    min_range: float,
    max_range: float,
) -> bool:
    """Whether (x, y) is inside the camera's horizontal FOV and its reliable detection range."""
    dx = x - boat_x
    dy = y - boat_y
    distance = math.hypot(dx, dy)
    if distance < min_range or distance > max_range:
        return False
    bearing = math.degrees(math.atan2(dy, dx))
    return abs(angle_difference(bearing, heading)) <= hfov_deg / 2.0


def travel_cost(dx: float, dy: float, upwind_deg, no_go_deg: float) -> float:
    """
    Distance sailed to cover (dx, dy). Inside the no-go zone the boat must tack, which costs
    cos(angle off the wind) / cos(no-go angle) times the straight-line distance.
    """
    distance = math.hypot(dx, dy)
    if upwind_deg is None or distance == 0.0:
        return distance
    off_wind = abs(angle_difference(math.degrees(math.atan2(dy, dx)), upwind_deg))
    if off_wind >= no_go_deg:
        return distance
    return (
        distance * math.cos(math.radians(off_wind)) / math.cos(math.radians(no_go_deg))
    )


class CoverageMap:
    """
    Grid over the search circle. Each cell holds the probability that the buoy is in it and has
    not been detected yet. Every cell the camera looks at loses probability at a detection rate
    that falls off with range, so whatever the boat sails (legs, approaches, tacks) is credited.
    The total left is the chance the buoy is still unfound.
    """

    def __init__(
        self,
        center_x: float,
        center_y: float,
        radius: float,
        cell_size: float,
        prior_center_std: float = 0.0,
    ):
        self.center_x = center_x
        self.center_y = center_y
        self.radius = radius
        self.cell_size = cell_size
        n = max(int(math.ceil(radius / cell_size)), 1)
        offsets = (np.arange(-n, n) + 0.5) * cell_size
        gx, gy = np.meshgrid(offsets, offsets)
        inside = np.hypot(gx, gy) <= radius
        self.dx = gx[inside]
        self.dy = gy[inside]
        self.xs = center_x + self.dx
        self.ys = center_y + self.dy

        # Uniform prior, or weighted toward the center when the buoy is probably near it
        prior = np.ones(self.xs.size)
        if prior_center_std > 0:
            prior = np.exp(-0.5 * (self.dx**2 + self.dy**2) / prior_center_std**2) + 0.1
        self.p = prior / prior.sum()
        self.pass_start = self.p.copy()
        self.sweep = (
            None  # world unit vector the current pass sweeps across the area along
        )

    def coverage(self) -> float:
        """Probability the buoy would have been detected by now if it is in the area."""
        return float(1.0 - self.p.sum())

    def start_pass(self):
        """Measure 'done' against the current map, so the next pass targets what is least covered."""
        self.pass_start = self.p.copy()
        self.sweep = None

    def observe(
        self,
        boat_x: float,
        boat_y: float,
        heading: float,
        hfov_deg: float,
        min_range: float,
        reliable_range: float,
        max_range: float,
        detect_rate: float,
        dt: float,
    ):
        """Credit the cells in the camera's view for dt seconds of looking without a confirmed buoy."""
        dist = np.hypot(self.xs - boat_x, self.ys - boat_y)
        near = np.flatnonzero((dist >= min_range) & (dist <= max_range))
        if near.size == 0:
            return
        bearing = np.degrees(np.arctan2(self.ys[near] - boat_y, self.xs[near] - boat_x))
        seen = near[np.abs((bearing - heading + 180) % 360 - 180) <= hfov_deg / 2.0]
        # Full rate out to reliable_range, falling linearly to zero at max_range
        falloff = np.clip(
            (max_range - dist[seen]) / max(max_range - reliable_range, 1e-6), 0.0, 1.0
        )
        self.p[seen] *= np.exp(-detect_rate * falloff * dt)

    def plan_leg(
        self,
        boat_x: float,
        boat_y: float,
        direction_deg: float,
        spacing: float,
        min_leg: float,
        lead_in: float,
        min_leg_gain: float,
        line_shift: float = 0.0,
        upwind_deg=None,
        no_go_deg: float = 60.0,
    ):
        """
        Pick the next leg of a lawnmower sweep. Legs run along direction_deg + 90 (cross-wind when
        direction_deg is the wind) on lines at most spacing apart, spread evenly across the circle
        and moved sideways by line_shift of a gap (0.5 puts a later pass between the earlier lines).

        The lines are taken in order from the side the boat started the pass on, and the map
        decides which stretches of a line still need sailing: those where the leg would find at
        least min_leg_gain of what it would have found at the start of the pass. So water already
        seen (by an approach, a tack or a neighbouring line) is skipped, and a line cut short by
        an approach is the next one sailed, from where the boat left it. Of the stretches on a line
        the nearest is taken first. The leg starts lead_in before its stretch, since the camera
        looks ahead. Returns (start, end, cost), where cost is the distance to sail including
        tacking, or None when the pass is complete.
        """
        r = self.radius
        num_lines = max(int(math.ceil(2 * r / spacing)), 1)
        line_gap = 2 * r / num_lines
        num_bins = max(int(math.ceil(2 * r / self.cell_size)), 1)
        bin_centers = -r + (np.arange(num_bins) + 0.5) * self.cell_size
        join_bins = int(math.ceil(max(min_leg, 2 * spacing) / self.cell_size))

        theta = math.radians(direction_deg)
        normal = (math.cos(theta), math.sin(theta))
        along = (-math.sin(theta), math.cos(theta))
        u = self.dx * normal[0] + self.dy * normal[1]
        v = self.dx * along[0] + self.dy * along[1]
        cell_bin = np.clip(((v + r) // self.cell_size).astype(int), 0, num_bins - 1)

        # Sweep from the side of the area the boat is on when the pass starts. The sweep is kept as
        # a world direction so a wind shift that flips the line order does not reverse it.
        sweep = (
            0.0
            if self.sweep is None
            else self.sweep[0] * normal[0] + self.sweep[1] * normal[1]
        )
        if abs(sweep) < 0.5:
            boat_u = (boat_x - self.center_x) * normal[0] + (
                boat_y - self.center_y
            ) * normal[1]
            sweep = 1.0 if boat_u <= 0 else -1.0
            self.sweep = (sweep * normal[0], sweep * normal[1])

        for i in range(num_lines) if sweep > 0 else reversed(range(num_lines)):
            offset = -r + (i + 0.5 + line_shift) * line_gap
            if abs(offset) >= r:
                continue
            half_chord = math.sqrt(r**2 - offset**2)

            # What one pass along the line finds: all of it within spacing / 4 to the side, falling
            # to none at 3/4 spacing, so neighbouring lines cover the strip between them together
            lateral = np.abs(u - offset)
            near = np.flatnonzero(lateral < 0.75 * spacing)
            weight = np.clip(
                (0.75 * spacing - lateral[near]) / (0.5 * spacing), 0.0, 1.0
            )
            gain = np.bincount(
                cell_bin[near], weights=weight * self.p[near], minlength=num_bins
            )
            reference = np.bincount(
                cell_bin[near],
                weights=weight * self.pass_start[near],
                minlength=num_bins,
            )
            worth = (
                (np.abs(bin_centers) <= half_chord)
                & (reference > 0)
                & (gain >= min_leg_gain * reference)
            )
            bins = np.flatnonzero(worth)
            if bins.size == 0:
                continue

            # Stretches worth sailing, joined across gaps cheaper to sail through than to come back for
            runs = [[bins[0], bins[0]]]
            for j in bins[1:]:
                if j - runs[-1][1] <= join_bins:
                    runs[-1][1] = j
                else:
                    runs.append([j, j])

            # The stretch whose start is nearest, so any others on the line are ahead, not behind
            best = None
            for j0, j1 in runs:
                v_lo, v_hi = -r + j0 * self.cell_size, -r + (j1 + 1) * self.cell_size
                for heading in (1.0, -1.0):
                    v_start = (v_lo if heading > 0 else v_hi) - heading * lead_in
                    v_end = v_hi if heading > 0 else v_lo
                    v_start = min(max(v_start, -half_chord), half_chord)
                    v_end = min(max(v_end, -half_chord), half_chord)
                    if heading * (v_end - v_start) < min_leg:
                        # Too short to sail as a leg; stretch it within the chord
                        v_end = min(
                            max(v_start + heading * min_leg, -half_chord), half_chord
                        )
                        v_start = min(
                            max(v_end - heading * min_leg, -half_chord), half_chord
                        )
                    leg_start = (
                        self.center_x + offset * normal[0] + v_start * along[0],
                        self.center_y + offset * normal[1] + v_start * along[1],
                    )
                    leg_end = (
                        self.center_x + offset * normal[0] + v_end * along[0],
                        self.center_y + offset * normal[1] + v_end * along[1],
                    )
                    if math.hypot(leg_end[0] - boat_x, leg_end[1] - boat_y) < min_leg:
                        continue  # would count as reached as soon as it is sent
                    to_start = travel_cost(
                        leg_start[0] - boat_x,
                        leg_start[1] - boat_y,
                        upwind_deg,
                        no_go_deg,
                    )
                    cost = to_start + travel_cost(
                        leg_end[0] - leg_start[0],
                        leg_end[1] - leg_start[1],
                        upwind_deg,
                        no_go_deg,
                    )
                    if best is None or to_start < best[0]:
                        best = (to_start, leg_start, leg_end, cost)
            if best is not None:
                return best[1:]
        return None
