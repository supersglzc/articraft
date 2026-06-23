from __future__ import annotations

# Vendored gear generation support adapted from `cq_gears`
# (https://github.com/meadiode/cq_gears), Apache License 2.0.
#
# Original work:
# Involute-profile gear generator, involute profile gear generator
# Copyright 2021 meadiode@github
#
# This module has been ported from mesh geometry to the SDK's native mesh layer.
# The involute-gear math is preserved verbatim from the original; only the
# geometry *construction* (formerly NURBS faces sewn into a shell) is replaced
# by polygon extrusions / lofts and boolean operations on ``MeshGeometry``.
import warnings

import numpy as np

from sdk._core.v0._mesh.booleans import (
    boolean_difference,
    boolean_intersection,
    boolean_union,
)
from sdk._core.v0._mesh.native_build import cylinder_z, round_polygon_2d
from sdk._core.v0._mesh.primitives import (
    ExtrudeGeometry,
    LoftGeometry,
    MeshGeometry,
    _adopt_mesh_geometry,
)

__all__ = [
    "GearBase",
    "SpurGear",
    "HerringboneGear",
    "RingGear",
    "HerringboneRingGear",
    "PlanetaryGearset",
    "HerringbonePlanetaryGearset",
    "BevelGear",
    "BevelGearPair",
    "RackGear",
    "HerringboneRackGear",
    "Worm",
    "CrossedHelicalGear",
    "CrossedGearPair",
    "HyperbolicGear",
    "HyperbolicGearPair",
    "gear",
    "addGear",
]


# ---------------------------------------------------------------------------
# Pure math helpers (unchanged from the original cq_gears port)
# ---------------------------------------------------------------------------
def sphere_to_cartesian(r, gamma, theta):
    """Convert spherical coordinates to cartesian."""

    return (
        r * np.sin(gamma) * np.sin(theta),
        r * np.sin(gamma) * np.cos(theta),
        r * np.cos(gamma),
    )


def s_arc(sr, c_gamma, c_theta, r_delta, start, end, n=32):
    """Get arc points plotted on a sphere's surface."""

    t = np.expand_dims(np.linspace(start, end, n), axis=1)
    a = sphere_to_cartesian(1.0, c_gamma + r_delta, c_theta)
    k = sphere_to_cartesian(1.0, c_gamma, c_theta)
    c = np.cos(t) * a + np.sin(t) * np.cross(k, a) + np.dot(k, a) * (1.0 - np.cos(t)) * k
    c = c * sr
    return [dim.squeeze() for dim in np.hsplit(c, 3)]


def s_inv(gamma0, gamma):
    """Spherical involute curve function."""

    phi = np.arccos(np.tan(gamma0) / np.tan(gamma))
    return np.arccos(np.cos(gamma) / np.cos(gamma0)) / np.sin(gamma0) - phi


def circle3d_by3points(a, b, c):
    """Find a circle in 3D space defined by three points."""

    u = b - a
    w = np.cross(c - a, u)
    u = u / np.linalg.norm(u)
    w = w / np.linalg.norm(w)
    v = np.cross(w, u)

    bx = np.dot(b - a, u)
    cx, cy = np.dot(c - a, u), np.dot(c - a, v)

    h = ((cx - bx / 2.0) ** 2 + cy**2 - (bx / 2.0) ** 2) / (2.0 * cy)
    cc = a + u * (bx / 2.0) + v * h
    r = np.linalg.norm(a - cc)

    return r, cc


def rotation_matrix(axis, alpha):
    """Construct a 3D rotation transform matrix."""

    ux, uy, uz = axis
    sina, cosa = np.sin(alpha), np.cos(alpha)
    return np.array(
        (
            (
                cosa + (1.0 - cosa) * ux**2,
                ux * uy * (1.0 - cosa) - uz * sina,
                ux * uz * (1.0 - cosa) + uy * sina,
            ),
            (
                uy * ux * (1.0 - cosa) + uz * sina,
                cosa + (1.0 - cosa) * uy**2,
                uy * uz * (1.0 - cosa) - ux * sina,
            ),
            (
                uz * ux * (1.0 - cosa) - uy * sina,
                uz * uy * (1.0 - cosa) + ux * sina,
                cosa + (1.0 - cosa) * uz**2,
            ),
        )
    )


def angle_between(o, a, b):
    """Find the angle between vectors OA and OB."""

    p = a - o
    q = b - o
    return np.arccos(np.dot(p, q) / (np.linalg.norm(p) * np.linalg.norm(q)))


# ---------------------------------------------------------------------------
# Native-mesh construction helpers
# ---------------------------------------------------------------------------
def _profile_xy(points):
    """Drop the Z column of an (N, 3) point array into a list of ``(x, y)``."""
    return [(float(p[0]), float(p[1])) for p in points]


def _rotate_xy(points, angle):
    """Rotate a list of ``(x, y)`` points about the origin by ``angle`` radians."""
    ca, sa = np.cos(angle), np.sin(angle)
    return [(x * ca - y * sa, x * sa + y * ca) for (x, y) in points]


def _extrude_profile(profile_xy, height, z_center):
    """Straight extrusion of a closed XY polygon, centered then shifted to ``z_center``."""
    geom = ExtrudeGeometry(profile_xy, height)
    return geom.translate(0.0, 0.0, z_center) if z_center else geom


def _twist_extrude_profile(profile_xy, height, z_base, twist_angle, slices):
    """Loft a closed XY polygon along +Z while rotating it by ``twist_angle`` total.

    Replaces mesh geometry's ``twistExtrude``: ``slices`` rings are stacked from
    ``z_base`` to ``z_base + height`` and each ring is rotated incrementally.
    """
    slices = max(2, int(slices))
    profiles = []
    for k in range(slices + 1):
        frac = k / slices
        angle = twist_angle * frac
        ring = _rotate_xy(profile_xy, angle)
        z = z_base + height * frac
        profiles.append([(x, y, z) for (x, y) in ring])
    return LoftGeometry(profiles)


def _loft_rings(rings, *, cap=True):
    """Loft a sequence of 3D point rings into a solid.

    ``cap=False`` leaves the end rings open (use when the rings are non-planar
    and the body is closed by a separate union/intersection).
    """
    return LoftGeometry([list(ring) for ring in rings], cap=cap)


# ---------------------------------------------------------------------------
class GearBase(MeshGeometry):
    ka = 1.0
    kd = 1.25

    curve_points = 20
    surface_splines = 5

    # Number of slices used to approximate helical / twisted extrusions.
    twist_slices = 24

    def __init__(self, *args, **kv_args):
        raise NotImplementedError("Constructor is not defined")

    def build(self, **kv_params):
        params = {**self.build_params, **kv_params}
        return self._build(**params)

    def _finalize(self):
        """Build the default body and adopt it into ``self`` as a ``MeshGeometry``."""
        body = self.build()
        _adopt_mesh_geometry(self, body)


class SpurGear(GearBase):
    def __init__(
        self,
        module,
        teeth_number,
        width,
        pressure_angle=20.0,
        helix_angle=0.0,
        clearance=0.0,
        backlash=0.0,
        addendum_coeff=None,
        dedendum_coeff=None,
        **build_params,
    ):
        MeshGeometry.__init__(self)
        if addendum_coeff is not None and addendum_coeff <= 0:
            raise ValueError("Addendum coefficient (addendum_coeff) must be greater than 0.")
        if dedendum_coeff is not None and dedendum_coeff <= 0:
            raise ValueError("Dedendum coefficient (dedendum_coeff) must be greater than 0.")

        self.ka = addendum_coeff if addendum_coeff is not None else self.ka
        self.kd = dedendum_coeff if dedendum_coeff is not None else self.kd
        self.m = m = module
        self.z = z = teeth_number
        self.a0 = a0 = np.radians(pressure_angle)
        self.clearance = clearance
        self.backlash = backlash
        self.helix_angle = np.radians(helix_angle)
        self.width = width

        d0 = m * z
        adn = self.ka / (z / d0)
        ddn = self.kd / (z / d0)

        if 2.0 * ddn + 2.0 * clearance >= d0:
            raise ValueError(
                "Invalid dedendum or clearance: resulting dedendum circle diameter is negative or zero."
            )

        da = d0 + 2.0 * adn
        dd = d0 - 2.0 * ddn - 2.0 * clearance
        s0 = m * (np.pi / 2.0 - backlash * np.tan(a0))
        inv_a0 = np.tan(a0) - a0

        self.r0 = r0 = d0 / 2.0
        self.ra = ra = da / 2.0
        self.rd = rd = dd / 2.0
        self.rb = rb = np.cos(a0) * d0 / 2.0
        self.rr = rr = max(rb, rd)
        self.tau = tau = np.pi * 2.0 / z

        if helix_angle != 0.0:
            self.twist_angle = width / (r0 * np.tan(np.pi / 2.0 - self.helix_angle))
        else:
            self.surface_splines = 2
            self.twist_angle = 0.0

        self.build_params = build_params

        r = np.linspace(rr, ra, self.curve_points)
        cos_a = r0 / r * np.cos(a0)
        a = np.arccos(np.clip(cos_a, -1.0, 1.0))
        inv_a = np.tan(a) - a
        s = r * (s0 / d0 + inv_a0 - inv_a)
        phi = s / r
        self.t_lflank_pts = np.dstack(
            (np.cos(phi) * r, np.sin(phi) * r, np.zeros(self.curve_points))
        ).squeeze()

        b = np.linspace(phi[-1], -phi[-1], self.curve_points)
        self.t_tip_pts = np.dstack(
            (np.cos(b) * ra, np.sin(b) * ra, np.zeros(self.curve_points))
        ).squeeze()

        self.t_rflank_pts = np.dstack(
            ((np.cos(-phi) * r)[::-1], (np.sin(-phi) * r)[::-1], np.zeros(self.curve_points))
        ).squeeze()

        rho = tau - phi[0] * 2.0
        p1 = np.array((self.t_rflank_pts[-1][0], self.t_rflank_pts[-1][1], 0.0))
        p2 = np.array((np.cos(-phi[0] - rho / 2.0) * rd, np.sin(-phi[0] - rho / 2.0) * rd, 0.0))
        p3 = np.array((np.cos(-phi[0] - rho) * rr, np.sin(-phi[0] - rho) * rr, 0.0))

        bcr, bcxy = circle3d_by3points(p1, p2, p3)
        t1 = np.arctan2(p1[1] - bcxy[1], p1[0] - bcxy[0])
        t2 = np.arctan2(p3[1] - bcxy[1], p3[0] - bcxy[0])
        if t1 < 0.0:
            t1 += np.pi * 2.0
        if t2 < 0.0:
            t2 += np.pi * 2.0
        t1, t2 = min(t1, t2), max(t1, t2)
        t = np.linspace(t1 + np.pi * 2.0, t2 + np.pi * 2.0, self.curve_points)
        self.t_root_pts = np.dstack(
            (bcxy[0] + bcr * np.cos(t), bcxy[1] + bcr * np.sin(t), np.zeros(self.curve_points))
        ).squeeze()

        self._finalize()

    def tooth_points(self):
        return np.concatenate(
            (self.t_lflank_pts, self.t_tip_pts, self.t_rflank_pts, self.t_root_pts)
        )

    def gear_points(self):
        tpts = self.tooth_points()
        pts = tpts.copy()
        angle = self.tau
        for _ in range(self.z - 1):
            pts = np.concatenate((pts, tpts @ rotation_matrix((0.0, 0.0, 1.0), angle)))
            angle += self.tau
        return pts

    def _gear_profile_xy(self):
        """The full closed cross-section polygon (all teeth) in the XY plane."""
        return _profile_xy(self.gear_points())

    def _twist_slice_count(self):
        n = int(np.ceil(abs(self.twist_angle) / (np.pi / 4.0)))
        return max(self.twist_slices, n)

    def _build_body(self):
        """Build the raw gear solid (no bore / hub / spokes etc.)."""
        profile = self._gear_profile_xy()
        if self.twist_angle == 0.0:
            return _extrude_profile(profile, self.width, 0.0).translate(0.0, 0.0, self.width / 2.0)
        return _twist_extrude_profile(
            profile, self.width, 0.0, self.twist_angle, self._twist_slice_count()
        )

    def _make_bore(self, body, bore_d):
        if bore_d is None:
            return body
        tool = cylinder_z(bore_d / 2.0, self.width + 0.02, self.width / 2.0)
        return boolean_difference(body, tool)

    def _teeth_cutout_profile(self, t1, t2):
        """A wedge profile (XY) spanning teeth t1..t2 used to remove teeth."""
        at1 = t1 * self.tau + self.tau / 2.0
        at2 = t2 * self.tau + self.tau / 2.0
        rc = self.ra + max(self.ra, 1.0)
        rd = self.rd - max(self.rd * 0.05, 1.0e-4)
        n = max(8, int(abs(at2 - at1) / (np.pi / 64.0)))
        outer_angles = np.linspace(at1, at2, n)
        inner_angles = np.linspace(at2, at1, n)
        pts = [(np.cos(at1) * rd, np.sin(at1) * rd)]
        pts += [(np.cos(a) * rc, np.sin(a) * rc) for a in outer_angles]
        pts += [(np.cos(a) * rd, np.sin(a) * rd) for a in inner_angles]
        return pts

    def _remove_teeth(self, body, t1, t2):
        profile = self._teeth_cutout_profile(t1, t2)
        if self.twist_angle == 0.0:
            tool = _extrude_profile(profile, self.width + 0.2, self.width / 2.0)
        else:
            tool = _twist_extrude_profile(
                profile, self.width + 0.2, -0.1, self.twist_angle, self._twist_slice_count()
            )
        return boolean_difference(body, tool)

    def _make_missing_teeth(self, body, missing_teeth):
        if missing_teeth is None:
            return body
        if isinstance(missing_teeth[0], (list, tuple)):
            for t1, t2 in missing_teeth:
                body = self._remove_teeth(body, t1, t2)
        else:
            t1, t2 = missing_teeth
            body = self._remove_teeth(body, t1, t2)
        return body

    def _make_recess(
        self,
        body,
        hub_d,
        recess_d,
        recess=None,
        bottom_recess=None,
        bottom_hub_d=None,
        bottom_recess_d=None,
    ):
        if recess is None and bottom_recess is None:
            return body
        if recess is not None:
            assert recess_d is not None, "Top face recess diameter is not set"
        if bottom_recess is not None:
            assert bottom_recess_d is not None or recess_d is not None, (
                "Bottom face recess diameter is not set"
            )
        if recess:
            outer = cylinder_z(recess_d / 2.0, recess, self.width - recess / 2.0)
            if hub_d is not None:
                outer = boolean_difference(
                    outer, cylinder_z(hub_d / 2.0, recess + 0.02, self.width - recess / 2.0)
                )
            body = boolean_difference(body, outer)
        if bottom_recess:
            if bottom_hub_d is None:
                bottom_hub_d = hub_d
            if bottom_recess_d is None:
                bottom_recess_d = recess_d
            outer = cylinder_z(bottom_recess_d / 2.0, bottom_recess, bottom_recess / 2.0)
            if bottom_hub_d is not None:
                outer = boolean_difference(
                    outer,
                    cylinder_z(bottom_hub_d / 2.0, bottom_recess + 0.02, bottom_recess / 2.0),
                )
            body = boolean_difference(body, outer)
        return body

    def _make_hub(self, body, hub_d, hub_length, bore_d):
        if hub_length is None:
            return body
        assert hub_d is not None, "Hub diameter is not set"
        hub = cylinder_z(hub_d / 2.0, hub_length, self.width + hub_length / 2.0)
        if bore_d is not None:
            hub = boolean_difference(
                hub, cylinder_z(bore_d / 2.0, hub_length + 0.02, self.width + hub_length / 2.0)
            )
        return boolean_union(body, hub)

    def _make_spokes(self, body, spokes_id, spokes_od, n_spokes, spoke_width, spoke_fillet):
        if n_spokes is None:
            return body
        assert n_spokes > 1, "Number of spokes must be > 1"
        assert spoke_width is not None, "Spoke width is not set"
        assert spokes_od is not None, "Outer spokes diameter is not set"
        if spokes_id is None:
            r1 = spoke_width / 2.0
        else:
            r1 = max(spoke_width / 2.0, spokes_id / 2.0)
        r2 = spokes_od / 2.0
        r1 += 0.0001
        r2 -= 0.0001
        tau = np.pi * 2.0 / n_spokes
        a1 = np.arcsin((spoke_width / 2.0) / (spokes_id / 2.0))
        a2 = np.arcsin((spoke_width / 2.0) / (spokes_od / 2.0))
        a3 = tau - a2
        a4 = tau - a1

        n_arc = 24
        outer_arc = np.linspace(a2, a3, n_arc)
        inner_arc = np.linspace(a4, a1, n_arc)
        profile = [(np.cos(a1) * r1, np.sin(a1) * r1)]
        profile += [(np.cos(a) * r2, np.sin(a) * r2) for a in outer_arc]
        profile += [(np.cos(a4) * r1, np.sin(a4) * r1)]
        profile += [(np.cos(a) * r1, np.sin(a) * r1) for a in inner_arc]

        if spoke_fillet is not None:
            profile = round_polygon_2d(profile, spoke_fillet, kind="fillet")

        cut_h = self.width + 1.0
        for i in range(n_spokes):
            rotated = _rotate_xy(profile, tau * i)
            tool = _extrude_profile(rotated, cut_h, self.width / 2.0)
            body = boolean_difference(body, tool)
        return body

    def _make_chamfer(self, body, chamfer=None, chamfer_top=None, chamfer_bottom=None):
        e = 0.01
        if chamfer is None and chamfer_top is None and chamfer_bottom is None:
            return body
        if chamfer is not None:
            if chamfer_top is None:
                chamfer_top = chamfer
            if chamfer_bottom is None:
                chamfer_bottom = chamfer
        if chamfer_top is not None:
            if isinstance(chamfer_top, (list, tuple)):
                wx, wy = chamfer_top
            else:
                wx, wy = chamfer_top, chamfer_top
            # Revolved chamfer cutter around the top outer edge.
            profile = [
                (self.ra - wx, self.width + e),
                (self.ra + e, self.width + e),
                (self.ra + e, self.width - wy),
            ]
            cutter = self._revolve_profile(profile)
            body = boolean_difference(body, cutter)
        if chamfer_bottom is not None:
            if isinstance(chamfer_bottom, (list, tuple)):
                wx, wy = chamfer_bottom
            else:
                wx, wy = chamfer_bottom, chamfer_bottom
            profile = [
                (self.ra + e, wy),
                (self.ra + e, -e),
                (self.ra - wx, -e),
            ]
            cutter = self._revolve_profile(profile)
            body = boolean_difference(body, cutter)
        return body

    @staticmethod
    def _revolve_profile(profile_rz, segments=128):
        """Revolve a closed ``(r, z)`` profile about the Z axis into a solid."""
        from sdk._core.v0._mesh.primitives import LatheGeometry

        return LatheGeometry(profile_rz, segments=segments)

    def _build(
        self,
        bore_d=None,
        missing_teeth=None,
        hub_d=None,
        hub_length=None,
        recess_d=None,
        recess=None,
        bottom_recess=None,
        bottom_recess_d=None,
        bottom_hub_d=None,
        n_spokes=None,
        spoke_width=None,
        spoke_fillet=None,
        spokes_id=None,
        spokes_od=None,
        chamfer=None,
        chamfer_top=None,
        chamfer_bottom=None,
        *args,
        **kv_args,
    ):
        body = self._build_body()
        body = self._make_chamfer(body, chamfer, chamfer_top, chamfer_bottom)
        body = self._make_bore(body, bore_d)
        body = self._make_missing_teeth(body, missing_teeth)
        body = self._make_recess(
            body,
            hub_d,
            recess_d,
            recess,
            bottom_recess=bottom_recess,
            bottom_hub_d=bottom_hub_d,
            bottom_recess_d=bottom_recess_d,
        )
        body = self._make_hub(body, hub_d, hub_length, bore_d)
        if spokes_id is None:
            spokes_id = hub_d
        if spokes_od is None:
            spokes_od = recess_d
        body = self._make_spokes(body, spokes_id, spokes_od, n_spokes, spoke_width, spoke_fillet)
        return body


class HerringboneGear(SpurGear):
    def _build_body(self):
        profile = self._gear_profile_xy()
        if self.twist_angle == 0.0:
            return _extrude_profile(profile, self.width, 0.0).translate(0.0, 0.0, self.width / 2.0)
        half = self.width / 2.0
        slices = self._twist_slice_count()
        lower = _twist_extrude_profile(profile, half, 0.0, self.twist_angle, slices)
        # Continue from the top profile of the lower half, twisting back.
        top_profile = _rotate_xy(profile, self.twist_angle)
        upper = _twist_extrude_profile(top_profile, half, half, -self.twist_angle, slices)
        return boolean_union(lower, upper)

    def _remove_teeth(self, body, t1, t2):
        profile = self._teeth_cutout_profile(t1, t2)
        if self.twist_angle == 0.0:
            tool = _extrude_profile(profile, self.width + 0.2, self.width / 2.0)
            return boolean_difference(body, tool)
        half = self.width / 2.0
        slices = self._twist_slice_count()
        lower = _twist_extrude_profile(profile, half + 0.05, -0.05, self.twist_angle, slices)
        top_profile = _rotate_xy(profile, self.twist_angle)
        upper = _twist_extrude_profile(top_profile, half + 0.05, half, -self.twist_angle, slices)
        body = boolean_difference(body, lower)
        body = boolean_difference(body, upper)
        return body


class RingGear(SpurGear):
    def __init__(
        self,
        module,
        teeth_number,
        width,
        rim_width,
        pressure_angle=20.0,
        helix_angle=0.0,
        clearance=0.0,
        backlash=0.0,
        **build_params,
    ):
        MeshGeometry.__init__(self)
        self.m = m = module
        self.z = z = teeth_number
        self.a0 = a0 = np.radians(pressure_angle)
        self.clearance = clearance
        self.backlash = backlash
        self.helix_angle = np.radians(helix_angle)
        self.width = width
        self.rim_width = rim_width

        d0 = m * z
        adn = self.ka / (z / d0)
        ddn = self.kd / (z / d0)
        da = d0 - 2.0 * adn
        dd = d0 + 2.0 * ddn + 2.0 * clearance
        s0 = m * (np.pi / 2.0 + backlash * np.tan(a0))
        inv_a0 = np.tan(a0) - a0

        self.r0 = r0 = d0 / 2.0
        self.ra = ra = da / 2.0
        self.rd = rd = dd / 2.0
        self.rb = rb = np.cos(a0) * d0 / 2.0
        self.rr = rr = max(rb, rd)
        self.tau = tau = np.pi * 2.0 / z

        if helix_angle != 0.0:
            self.twist_angle = width / (r0 * np.tan(np.pi / 2.0 - self.helix_angle))
        else:
            self.surface_splines = 2
            self.twist_angle = 0.0

        self.rim_r = rd + rim_width
        self.build_params = build_params

        r = np.linspace(ra, rr, self.curve_points)
        cos_a = r0 / r * np.cos(a0)
        a = np.arccos(np.clip(cos_a, -1.0, 1.0))
        inv_a = np.tan(a) - a
        s = r * (s0 / d0 + inv_a0 - inv_a)
        phi = s / r
        self.t_lflank_pts = np.dstack(
            (np.cos(phi) * r, np.sin(phi) * r, np.zeros(self.curve_points))
        ).squeeze()
        b = np.linspace(phi[-1], -phi[-1], self.curve_points)
        self.t_tip_pts = np.dstack(
            (np.cos(b) * rd, np.sin(b) * rd, np.zeros(self.curve_points))
        ).squeeze()
        self.t_rflank_pts = np.dstack(
            ((np.cos(-phi) * r)[::-1], (np.sin(-phi) * r)[::-1], np.zeros(self.curve_points))
        ).squeeze()

        rho = tau - phi[0] * 2.0
        p1 = np.array((self.t_rflank_pts[-1][0], self.t_rflank_pts[-1][1], 0.0))
        p2 = np.array((np.cos(-phi[0] - rho / 2.0) * ra, np.sin(-phi[0] - rho / 2.0) * ra, 0.0))
        p3 = np.array((np.cos(-phi[0] - rho) * ra, np.sin(-phi[0] - rho) * ra, 0.0))

        bcr, bcxy = circle3d_by3points(p1, p2, p3)
        t1 = np.arctan2(p1[1] - bcxy[1], p1[0] - bcxy[0])
        t2 = np.arctan2(p3[1] - bcxy[1], p3[0] - bcxy[0])
        if t1 < 0.0:
            t1 += np.pi * 2.0
        if t2 < 0.0:
            t2 += np.pi * 2.0
        t1, t2 = min(t1, t2), max(t1, t2)
        t = np.linspace(t2 + np.pi * 2.0, t1 + np.pi * 2.0, self.curve_points)
        self.t_root_pts = np.dstack(
            (bcxy[0] + bcr * np.cos(t), bcxy[1] + bcr * np.sin(t), np.zeros(self.curve_points))
        ).squeeze()

        self._finalize()

    def _build_body(self):
        """Ring gear: solid rim annulus with the tooth profile subtracted inside."""
        # Outer solid disc up to the rim radius.
        if self.twist_angle == 0.0:
            outer = cylinder_z(self.rim_r, self.width, self.width / 2.0)
            teeth_tool = _extrude_profile(
                self._gear_profile_xy(), self.width + 0.02, self.width / 2.0
            )
        else:
            outer = cylinder_z(self.rim_r, self.width, self.width / 2.0)
            teeth_tool = _twist_extrude_profile(
                self._gear_profile_xy(),
                self.width + 0.02,
                -0.01,
                self.twist_angle,
                self._twist_slice_count(),
            )
        return boolean_difference(outer, teeth_tool)

    def _make_chamfer(self, body, chamfer=None, chamfer_top=None, chamfer_bottom=None):
        e = 0.01
        if chamfer is None and chamfer_top is None and chamfer_bottom is None:
            return body
        if chamfer is not None:
            if chamfer_top is None:
                chamfer_top = chamfer
            if chamfer_bottom is None:
                chamfer_bottom = chamfer
        if chamfer_top is not None:
            if isinstance(chamfer_top, (list, tuple)):
                wx, wy = chamfer_top
            else:
                wx, wy = chamfer_top, chamfer_top
            profile = [
                (self.ra - e, self.width - wy),
                (self.ra - e, self.width + e),
                (self.ra + wx, self.width + e),
            ]
            cutter = self._revolve_profile(profile)
            body = boolean_difference(body, cutter)
        if chamfer_bottom is not None:
            if isinstance(chamfer_bottom, (list, tuple)):
                wx, wy = chamfer_bottom
            else:
                wx, wy = chamfer_bottom, chamfer_bottom
            profile = [
                (self.ra + wx, -e),
                (self.ra - e, -e),
                (self.ra - e, wy),
            ]
            cutter = self._revolve_profile(profile)
            body = boolean_difference(body, cutter)
        return body

    def _build(self, chamfer=None, chamfer_top=None, chamfer_bottom=None, *args, **kv_args):
        body = self._build_body()
        return self._make_chamfer(body, chamfer, chamfer_top, chamfer_bottom)


class HerringboneRingGear(RingGear):
    def _build_body(self):
        if self.twist_angle == 0.0:
            return super()._build_body()
        half = self.width / 2.0
        slices = self._twist_slice_count()
        outer = cylinder_z(self.rim_r, self.width, self.width / 2.0)
        profile = self._gear_profile_xy()
        lower = _twist_extrude_profile(profile, half + 0.01, -0.005, self.twist_angle, slices)
        top_profile = _rotate_xy(profile, self.twist_angle)
        upper = _twist_extrude_profile(
            top_profile, half + 0.01, half - 0.005, -self.twist_angle, slices
        )
        teeth_tool = boolean_union(lower, upper)
        return boolean_difference(outer, teeth_tool)


class PlanetaryGearset(GearBase):
    gear_cls = SpurGear
    ring_gear_cls = RingGear

    def __init__(
        self,
        module,
        sun_teeth_number,
        planet_teeth_number,
        width,
        rim_width,
        n_planets,
        pressure_angle=20.0,
        helix_angle=0.0,
        clearance=0.0,
        backlash=0.0,
        **build_params,
    ):
        MeshGeometry.__init__(self)
        ring_z = sun_teeth_number + planet_teeth_number * 2
        self.sun = self.gear_cls(
            module,
            sun_teeth_number,
            width,
            pressure_angle=pressure_angle,
            helix_angle=helix_angle,
            clearance=clearance,
            backlash=backlash,
        )
        self.planet = self.gear_cls(
            module,
            planet_teeth_number,
            width,
            pressure_angle=pressure_angle,
            helix_angle=-helix_angle,
            clearance=clearance,
            backlash=backlash,
        )
        self.ring = self.ring_gear_cls(
            module,
            ring_z,
            width,
            rim_width,
            pressure_angle=pressure_angle,
            helix_angle=-helix_angle,
            clearance=clearance,
            backlash=backlash,
        )
        self.orbit_r = self.sun.r0 + self.planet.r0
        self.n_planets = n_planets

        if ((sun_teeth_number + planet_teeth_number) % n_planets) and (
            (sun_teeth_number % n_planets) or (ring_z % n_planets)
        ):
            warnings.warn(
                "Planet gears being spaced evenly probably won't mesh properly "
                "(if at all) with the given number of teeth for the sun/planet "
                "gears and the number of planets"
            )
        self.build_params = build_params
        self._finalize()

    def _build(
        self,
        build_sun=True,
        build_planets=True,
        build_ring=True,
        sun_build_args={},
        planet_build_args={},
        ring_build_args={},
        **kv_args,
    ):
        parts = []
        if build_sun:
            in_args = self.build_params.get("sun_build_args", {})
            args = {**self.build_params, **in_args, **kv_args, **sun_build_args}
            sun = self.sun.build(**args)
            if (self.planet.z % 2) != 0:
                sun = sun.copy().rotate_z(self.sun.tau / 2.0)
            parts.append(sun)

        if build_planets and self.n_planets > 0:
            planet_a = np.pi * 2.0 / self.n_planets
            tobuild = (
                build_planets
                if isinstance(build_planets, (list, tuple))
                else [True] * self.n_planets
            )
            in_args = self.build_params.get("planet_build_args", {})
            args = {**self.build_params, **in_args, **kv_args, **planet_build_args}
            planet_body = self.planet.build(**args)
            for i, bld in enumerate(tobuild):
                if not bld:
                    continue
                placed = planet_body.copy().rotate_z(self.planet.tau / 2.0)
                placed = placed.translate(
                    np.cos(i * planet_a) * self.orbit_r,
                    np.sin(i * planet_a) * self.orbit_r,
                    0.0,
                )
                parts.append(placed)

        if build_ring:
            in_args = self.build_params.get("ring_build_args", {})
            args = {**self.build_params, **in_args, **kv_args, **ring_build_args}
            ring = self.ring.build(**args)
            ring = ring.copy().rotate_z(self.ring.tau / 2.0)
            parts.append(ring)

        if not parts:
            raise ValueError("Planetary gearset has no parts to build")
        result = parts[0]
        for part in parts[1:]:
            result = boolean_union(result, part)
        return result


class HerringbonePlanetaryGearset(PlanetaryGearset):
    gear_cls = HerringboneGear
    ring_gear_cls = HerringboneRingGear


class BevelGear(GearBase):
    surface_splines = 12

    def __init__(
        self,
        module,
        teeth_number,
        cone_angle,
        face_width,
        pressure_angle=20.0,
        helix_angle=0.0,
        clearance=0.0,
        backlash=0.0,
        **build_params,
    ):
        MeshGeometry.__init__(self)
        self.m = m = module
        self.z = z = teeth_number
        self.a0 = a0 = np.radians(pressure_angle)
        self.clearance = clearance
        self.backlash = backlash
        self.helix_angle = np.radians(helix_angle)
        self.face_width = face_width

        self.gamma_p = gamma_p = np.radians(cone_angle)
        rp = m * z / 2.0
        self.gs_r = gs_r = rp / np.sin(gamma_p)
        assert gs_r > face_width, f"face_width value is too big, it should be < {gs_r:0.3f}"
        self.gamma_b = gamma_b = np.arcsin(np.cos(a0) * np.sin(gamma_p))
        self.gamma_f = gamma_f = gamma_p + np.arctan(self.ka * m / gs_r)
        self.gamma_r = gamma_r = gamma_p - np.arctan(self.kd * m / gs_r)
        self.tau = tau = np.pi * 2.0 / z

        if helix_angle != 0.0:
            beta = np.arctan(face_width * np.tan(self.helix_angle) / (2.0 * gs_r - face_width))
            self.twist_angle = np.arcsin(gs_r / rp * np.sin(beta)) * 2.0
        else:
            self.surface_splines = 2
            self.twist_angle = 0.0
        assert not np.isnan(self.twist_angle), "Twist angle is NaN"

        self.build_params = build_params
        self.cone_h = np.cos(gamma_r) * gs_r
        phi_r = s_inv(gamma_b, gamma_p)
        self.mp_theta = mp_theta = np.pi / z + 2.0 * phi_r

        gamma_tr = max(gamma_b, gamma_r)
        gamma = np.linspace(gamma_tr, gamma_f, self.curve_points)
        theta = s_inv(gamma_b, gamma) + backlash / (module * teeth_number)
        self.t_lflank_pts = np.dstack(sphere_to_cartesian(1.0, gamma, theta)).squeeze()

        theta_tip = np.linspace(theta[-1], mp_theta - theta[-1], self.curve_points)
        self.t_tip_pts = np.dstack(
            sphere_to_cartesian(1.0, np.full(self.curve_points, gamma_f), theta_tip)
        ).squeeze()

        self.t_rflank_pts = np.dstack(
            sphere_to_cartesian(1.0, gamma[::-1], mp_theta - theta[::-1])
        ).squeeze()

        if gamma_r < gamma_b:
            p1 = self.t_rflank_pts[-1]
            p2 = np.array(sphere_to_cartesian(1.0, gamma_b, theta[0] + tau))
            p3 = np.array(sphere_to_cartesian(1.0, gamma_r, (tau + mp_theta) / 2.0))
            rr, rcc = circle3d_by3points(p1, p2, p3)
            rcc_gamma = np.arccos(np.dot(p3, rcc) / (np.linalg.norm(p3) * np.linalg.norm(rcc)))
            p1p3 = angle_between(rcc, p1, p3)
            a_start = (np.pi - p1p3 * 2.0) / 2.0
            a_end = -a_start + np.pi
            self.t_root_pts = np.dstack(
                s_arc(
                    1.0,
                    gamma_r + rcc_gamma,
                    (tau + mp_theta) / 2.0,
                    rcc_gamma,
                    np.pi / 2.0 + a_start,
                    np.pi / 2.0 + a_end,
                    self.curve_points,
                )
            ).squeeze()
        else:
            r_theta = np.linspace(mp_theta - theta[0], theta[0] + tau, self.curve_points)
            self.t_root_pts = np.dstack(
                sphere_to_cartesian(1.0, np.full(self.curve_points, gamma_tr), r_theta)
            ).squeeze()

        self._finalize()

    def tooth_points(self):
        return np.concatenate(
            (self.t_lflank_pts, self.t_tip_pts, self.t_rflank_pts, self.t_root_pts)
        )

    def gear_points(self):
        tpts = self.tooth_points()
        pts = tpts.copy()
        angle = self.tau
        for _ in range(self.z - 1):
            pts = np.concatenate((pts, tpts @ rotation_matrix((0.0, 0.0, 1.0), angle)))
            angle += self.tau
        return pts

    def _bevel_ring(self, radius, twist):
        """A full closed ring of gear points scaled to ``radius`` (on the unit sphere)
        and twisted about Z by ``twist``. Returns a list of 3D points."""
        tpts = self.tooth_points()
        r_mat = rotation_matrix((0.0, 0.0, 1.0), twist)
        rotated = (tpts @ r_mat) * radius
        ring_pts = rotated.copy()
        angle = self.tau
        for _ in range(self.z - 1):
            ring_pts = np.concatenate((ring_pts, rotated @ rotation_matrix((0.0, 0.0, 1.0), angle)))
            angle += self.tau
        return [(float(p[0]), float(p[1]), float(p[2])) for p in ring_pts]

    def _build_body(self):
        """Build a solid bevel gear by lofting closed tooth-section rings between
        the inner (small) and outer (large) spherical radii and capping each end
        to the central axis, producing a watertight tapered toothed cone."""
        # Spherical radii at the back (large) and front (small) cones.
        pc_h = np.cos(self.gamma_r) * self.gs_r
        pc_f = pc_h / np.cos(self.gamma_f)
        tc_h = np.cos(self.gamma_f) * (self.gs_r - self.face_width)
        tc_f = tc_h / np.cos(self.gamma_r)
        ta1 = -(pc_f - self.gs_r) / self.face_width * self.twist_angle
        ta2 = (self.gs_r - tc_f) / self.face_width * self.twist_angle

        n_sections = max(3, self.surface_splines)
        radii = np.linspace(pc_f, tc_f, n_sections)
        twists = np.linspace(ta1, ta2, n_sections)
        rings = [self._bevel_ring(r, tw) for r, tw in zip(radii, twists)]

        # Flatten each spherical tooth ring to its mean z so every loft profile is
        # planar in XY. LoftGeometry then builds a tapered toothed frustum with
        # planar end caps -- a watertight single-body bevel gear that matches the
        # exact involute tooth outline (the spherical z-curvature is approximated
        # by the per-ring mean plane).
        profiles = []
        for ring in rings:
            mean_z = sum(p[2] for p in ring) / len(ring)
            profiles.append([(p[0], p[1], mean_z) for p in ring])
        return LoftGeometry(profiles, cap=True)

    def _make_bore(self, body, bore_d):
        if bore_d is None:
            return body
        tool = cylinder_z(bore_d / 2.0, self.cone_h * 3.0)
        return boolean_difference(body, tool)

    def _build(self, bore_d=None, trim_bottom=True, trim_top=True, **kv_args):
        body = self._build_body()
        # Reorient so the back cone sits at z=0 and the gear opens upward,
        # matching the original mesh geometry placement.
        body = body.rotate((1.0, 0.0, 0.0), np.pi)
        body = body.translate(0.0, 0.0, self.cone_h)
        t_align_angle = -self.mp_theta / 2.0 - np.pi / 2.0 + np.pi / self.z
        body = body.rotate((0.0, 0.0, 1.0), t_align_angle)
        return self._make_bore(body, bore_d)


class BevelGearPair(GearBase):
    gear_cls = BevelGear

    def __init__(
        self,
        module,
        gear_teeth,
        pinion_teeth,
        face_width,
        axis_angle=90.0,
        pressure_angle=20.0,
        helix_angle=0.0,
        clearance=0.0,
        backlash=0.0,
        **build_params,
    ):
        MeshGeometry.__init__(self)
        self.axis_angle = axis_angle = np.radians(axis_angle)
        aa_sin = np.sin(axis_angle)
        aa_cos = np.cos(axis_angle)
        delta_gear = np.arctan(aa_sin / (pinion_teeth / gear_teeth + aa_cos))
        delta_pinion = np.arctan(aa_sin / (gear_teeth / pinion_teeth + aa_cos))
        self.gear = self.gear_cls(
            module,
            gear_teeth,
            np.degrees(delta_gear),
            face_width,
            pressure_angle,
            helix_angle,
            clearance,
            backlash,
        )
        self.pinion = self.gear_cls(
            module,
            pinion_teeth,
            np.degrees(delta_pinion),
            face_width,
            pressure_angle,
            -helix_angle,
            backlash=backlash,
        )
        self.build_params = build_params
        self._finalize()

    def _build(
        self,
        build_gear=True,
        build_pinion=True,
        transform_pinion=True,
        gear_build_args={},
        pinion_build_args={},
        **kv_args,
    ):
        parts = []
        if build_gear:
            in_args = self.build_params.get("gear_build_args", {})
            args = {**self.build_params, **in_args, **kv_args, **gear_build_args}
            parts.append(self.gear.build(**args))
        if build_pinion:
            in_args = self.build_params.get("pinion_build_args", {})
            args = {**self.build_params, **in_args, **kv_args, **pinion_build_args}
            pinion = self.pinion.build(**args)
            if transform_pinion:
                pinion = pinion.copy()
                pinion = pinion.translate(0.0, 0.0, -self.pinion.cone_h)
                if self.pinion.z % 2 == 0:
                    pinion = pinion.rotate_z(np.pi / self.pinion.z)
                pinion = pinion.rotate((0.0, 1.0, 0.0), self.axis_angle)
                pinion = pinion.translate(0.0, 0.0, self.gear.cone_h)
            parts.append(pinion)
        if not parts:
            raise ValueError("Bevel gear pair has no parts to build")
        result = parts[0]
        for part in parts[1:]:
            result = boolean_union(result, part)
        return result


class RackGear(GearBase):
    def __init__(
        self,
        module,
        length,
        width,
        height,
        pressure_angle=20.0,
        helix_angle=0.0,
        clearance=0.0,
        backlash=0.0,
        **build_params,
    ):
        MeshGeometry.__init__(self)
        self.m = m = module
        self.a0 = a0 = np.radians(pressure_angle)
        self.clearance = clearance
        self.backlash = backlash
        self.helix_angle = np.radians(helix_angle)
        self.width = width
        self.length = length
        self.height = height
        self.build_params = build_params

        adn = self.ka * m
        ddn = self.kd * m
        self.la = la = adn
        self.ld = ld = -(ddn + clearance)
        s0 = m * (np.pi / 2.0 - backlash * np.tan(a0)) / 2.0

        p1x = np.tan(a0) * abs(ld)
        p1p2 = (abs(la) + abs(ld)) / np.cos(a0)
        p1 = (-s0 - p1x, ld, 0.0)
        p2 = (np.sin(a0) * p1p2 + p1[0], np.cos(a0) * p1p2 + p1[1], 0.0)
        p3 = (-p2[0], p2[1], 0.0)
        p4 = (-p1[0], p1[1], 0.0)
        p5 = (p4[0] + (np.pi * m - p4[0] * 2.0), p4[1], 0.0)

        self.t_lflank_pts = np.array((p1, p2))
        self.t_tip_pts = np.array((p2, p3))
        self.t_rflank_pts = np.array((p3, p4))
        self.t_root_pts = np.array((p4, p5))
        self.tooth_height = abs(la) + abs(ld)
        self.z = int(np.ceil(self.length / (np.pi * self.m)))

        self._finalize()

    def tooth_points(self):
        return np.concatenate(
            (self.t_lflank_pts, self.t_tip_pts, self.t_rflank_pts, self.t_root_pts)
        )

    def gear_points(self):
        tpts = self.tooth_points()
        pts = tpts.copy()
        for i in range(10):
            ttpts = tpts.copy()
            ttpts[:, 0] += np.pi * self.m * (i + 1)
            pts = np.concatenate((pts, ttpts))
        return pts

    def _rack_cross_profile(self):
        """The toothed cross-section in the XY plane (x along length, y the tooth
        height), closed downward to ``ld - height``. Extruded along Z for width.

        The toothed top edge is generated across enough teeth to cover the rack
        length, then clipped to ``x in [0, length]`` with exact segment/line
        intersections so the resulting polygon is simple (non self-intersecting).
        """
        bottom_y = self.ld - self.height
        length = self.length

        # Raw toothed top edge across enough teeth (start one tooth before 0 so
        # the left clip lands inside a real tooth).
        raw = []
        n_teeth = self.z + 3
        for i in range(-1, n_teeth):
            x0 = np.pi * self.m * i
            for seg in (
                self.t_lflank_pts,
                self.t_tip_pts,
                self.t_rflank_pts,
                self.t_root_pts,
            ):
                for px, py, _pz in seg:
                    raw.append((px + x0, py))
        # Drop consecutive duplicates.
        edge = [raw[0]]
        for pt in raw[1:]:
            if abs(pt[0] - edge[-1][0]) > 1e-12 or abs(pt[1] - edge[-1][1]) > 1e-12:
                edge.append(pt)

        def clip_to_strip(pts, x_lo, x_hi):
            # Walk the polyline, inserting exact crossings at x=x_lo / x=x_hi and
            # keeping only points within [x_lo, x_hi].
            result = []
            prev = None
            for cur in pts:
                if prev is not None:
                    for xb in sorted((x_lo, x_hi)):
                        if (prev[0] - xb) * (cur[0] - xb) < 0:
                            t = (xb - prev[0]) / (cur[0] - prev[0])
                            result.append((xb, prev[1] + t * (cur[1] - prev[1])))
                if x_lo - 1e-12 <= cur[0] <= x_hi + 1e-12:
                    result.append((min(max(cur[0], x_lo), x_hi), cur[1]))
                prev = cur
            # Drop consecutive duplicates.
            cleaned = [result[0]]
            for pt in result[1:]:
                if abs(pt[0] - cleaned[-1][0]) > 1e-9 or abs(pt[1] - cleaned[-1][1]) > 1e-9:
                    cleaned.append(pt)
            return cleaned

        top = clip_to_strip(edge, 0.0, length)
        # Ensure the top edge starts at x=0 and ends at x=length.
        if top[0][0] > 1e-9:
            top.insert(0, (0.0, top[0][1]))
        if top[-1][0] < length - 1e-9:
            top.append((length, top[-1][1]))

        profile = list(top)
        profile += [(length, bottom_y), (0.0, bottom_y)]
        # Drop consecutive / wrap duplicates.
        cleaned = [profile[0]]
        for pt in profile[1:]:
            if abs(pt[0] - cleaned[-1][0]) > 1e-9 or abs(pt[1] - cleaned[-1][1]) > 1e-9:
                cleaned.append(pt)
        if (
            abs(cleaned[0][0] - cleaned[-1][0]) <= 1e-9
            and abs(cleaned[0][1] - cleaned[-1][1]) <= 1e-9
        ):
            cleaned.pop()
        return cleaned

    def _build_body(self):
        profile = self._rack_cross_profile()
        # Extrude along Z (width). ExtrudeGeometry extrudes an XY profile along Z.
        return ExtrudeGeometry(profile, self.width).translate(0.0, 0.0, self.width / 2.0)

    def _build(self, *args, **kv_args):
        return self._build_body()


class HerringboneRackGear(RackGear):
    # Herringbone racks differ from straight racks only by the helical tooth lead,
    # which our straight-extrusion construction does not capture; the toothed
    # cross-section is identical, so we reuse the straight-rack body.
    pass


class Worm(GearBase):
    surface_splines = 8
    t_face_parts = 2

    def __init__(
        self,
        module,
        lead_angle,
        n_threads,
        length,
        pressure_angle=20.0,
        clearance=0.0,
        backlash=0.0,
        **build_params,
    ):
        MeshGeometry.__init__(self)
        self.m = m = module
        self.a0 = a0 = np.radians(pressure_angle)
        self.clearance = clearance
        self.backlash = backlash
        self.lead_angle = np.radians(lead_angle)
        self.length = length
        self.n_threads = n_threads
        self.build_params = build_params

        d0 = self.n_threads * m / np.abs(np.tan(self.lead_angle))
        self.r0 = d0 / 2.0
        adn = self.ka * m
        ddn = self.kd * m
        self.la = la = adn
        self.ld = ld = -(ddn + clearance)
        self.ra = self.r0 + adn
        self.rd = self.r0 - ddn

        s0 = m * (np.pi / 2.0 - backlash * np.tan(a0)) / 2.0
        p1x = np.tan(a0) * abs(ld)
        p1p2 = (abs(la) + abs(ld)) / np.cos(a0)
        p1 = (-s0 - p1x, ld, 0.0)
        p2 = (np.sin(a0) * p1p2 + p1[0], np.cos(a0) * p1p2 + p1[1], 0.0)
        p3 = (-p2[0], p2[1], 0.0)
        p4 = (-p1[0], p1[1], 0.0)
        p5 = (p4[0] + (np.pi * m - p4[0] * 2.0), p4[1], 0.0)
        self.t_lflank_pts = np.array((p1, p2))
        self.t_tip_pts = np.array((p2, p3))
        self.t_rflank_pts = np.array((p3, p4))
        self.t_root_pts = np.array((p4, p5))
        self.tooth_height = abs(la) + abs(ld)

        self._finalize()

    def tooth_points(self):
        return np.concatenate(
            (self.t_lflank_pts, self.t_tip_pts, self.t_rflank_pts, self.t_root_pts)
        )

    def gear_points(self):
        tpts = self.tooth_points()
        pts = tpts.copy()
        for i in range(10):
            ttpts = tpts.copy()
            ttpts[:, 0] += np.pi * self.m * (i + 1)
            pts = np.concatenate((pts, ttpts))
        return pts

    def _build_body(self):
        """Build a worm as helical thread(s) wrapped around a core cylinder.

        The worm is constructed with its axis along +Z (so the thread sweep maps
        onto :class:`LoftGeometry`'s Z-stacked rings), then rotated to align the
        axis with local X to match the original mesh geometry convention.
        """
        step = np.pi * self.m * self.n_threads
        turns = int(np.ceil(self.length / step)) + 2
        total_len = turns * step
        z_start = -total_len / 2.0
        tau = np.pi * 2.0 / self.n_threads

        slices_per_turn = max(8, self.surface_splines * 4)
        n_slices = slices_per_turn * turns
        sign = np.sign(self.lead_angle) if self.lead_angle != 0.0 else 1.0

        # Tooth profile: (px = along-thread, py = radial offset).
        tooth = self.tooth_points()

        body = None
        for th in range(self.n_threads):
            base_angle = tau * th
            rings = []
            for sidx in range(n_slices + 1):
                frac = sidx / n_slices
                z_pos = z_start + total_len * frac
                alpha = base_angle + sign * 2.0 * np.pi * (z_pos - z_start) / step
                ring = []
                for px, py, _pz in tooth:
                    rr = self.r0 + py
                    a = alpha + px / self.r0
                    ring.append((rr * np.cos(a), rr * np.sin(a), z_pos))
                rings.append(ring)
            thread = _loft_rings(rings)
            body = thread if body is None else boolean_union(body, thread)

        core = cylinder_z(self.rd, total_len + 2.0)
        body = boolean_union(body, core)

        # Trim to the requested length along the (Z) axis.
        keep = cylinder_z(self.ra * 2.0, self.length)
        body = boolean_intersection(body, keep)

        # Align the worm axis with local X.
        return body.rotate_y(np.pi / 2.0)

    def _make_bore(self, body, bore_d):
        if bore_d is None:
            return body
        from sdk._core.v0._mesh.native_build import cylinder_x

        tool = cylinder_x(bore_d / 2.0, self.length + 2.0)
        return boolean_difference(body, tool)

    def _build(self, bore_d=None):
        body = self._build_body()
        return self._make_bore(body, bore_d)


class CrossedHelicalGear(SpurGear):
    def __init__(
        self,
        module,
        teeth_number,
        width,
        pressure_angle=20.0,
        helix_angle=0.0,
        clearance=0.0,
        backlash=0.0,
        **build_params,
    ):
        MeshGeometry.__init__(self)
        self.m = m = module
        self.z = z = teeth_number
        self.a0 = a0 = np.radians(pressure_angle)
        self.clearance = clearance
        self.backlash = backlash
        self.helix_angle = np.radians(helix_angle)
        self.width = width

        at0 = np.arctan(a0 / np.cos(self.helix_angle))
        mt = m / np.cos(self.helix_angle)
        d0 = mt * z
        adn = self.ka / (z / d0)
        ddn = self.kd / (z / d0)
        da = d0 + 2.0 * adn
        dd = d0 - 2.0 * ddn - 2.0 * clearance
        inv_a0 = np.tan(at0) - at0

        self.r0 = r0 = d0 / 2.0
        self.ra = ra = da / 2.0
        self.rd = rd = dd / 2.0
        self.rb = rb = np.cos(at0) * r0
        self.rr = rr = max(rb, rd)
        self.tau = tau = np.pi * 2.0 / z
        s0 = r0 * np.pi / z

        if helix_angle != 0.0:
            self.twist_angle = width / (r0 * np.tan(np.pi / 2.0 - self.helix_angle))
        else:
            self.surface_splines = 2
            self.twist_angle = 0.0
        self.build_params = build_params

        r = np.linspace(rr, ra, self.curve_points)
        cos_a = r0 / r * np.cos(at0)
        a = np.arccos(np.clip(cos_a, -1.0, 1.0))
        inv_a = np.tan(a) - a
        s = r * (s0 / d0 + inv_a0 - inv_a)
        phi = s / r
        self.t_lflank_pts = np.dstack(
            (np.cos(phi) * r, np.sin(phi) * r, np.zeros(self.curve_points))
        ).squeeze()
        b = np.linspace(phi[-1], -phi[-1], self.curve_points)
        self.t_tip_pts = np.dstack(
            (np.cos(b) * ra, np.sin(b) * ra, np.zeros(self.curve_points))
        ).squeeze()
        self.t_rflank_pts = np.dstack(
            ((np.cos(-phi) * r)[::-1], (np.sin(-phi) * r)[::-1], np.zeros(self.curve_points))
        ).squeeze()

        rho = tau - phi[0] * 2.0
        p1 = np.array((self.t_rflank_pts[-1][0], self.t_rflank_pts[-1][1], 0.0))
        p2 = np.array((np.cos(-phi[0] - rho / 2.0) * rd, np.sin(-phi[0] - rho / 2.0) * rd, 0.0))
        p3 = np.array((np.cos(-phi[0] - rho) * rr, np.sin(-phi[0] - rho) * rr, 0.0))
        bcr, bcxy = circle3d_by3points(p1, p2, p3)
        t1 = np.arctan2(p1[1] - bcxy[1], p1[0] - bcxy[0])
        t2 = np.arctan2(p3[1] - bcxy[1], p3[0] - bcxy[0])
        if t1 < 0.0:
            t1 += np.pi * 2.0
        if t2 < 0.0:
            t2 += np.pi * 2.0
        t1, t2 = min(t1, t2), max(t1, t2)
        t = np.linspace(t1 + np.pi * 2.0, t2 + np.pi * 2.0, self.curve_points)
        self.t_root_pts = np.dstack(
            (bcxy[0] + bcr * np.cos(t), bcxy[1] + bcr * np.sin(t), np.zeros(self.curve_points))
        ).squeeze()

        self._finalize()


class CrossedGearPair(GearBase):
    gear1_cls = CrossedHelicalGear
    gear2_cls = CrossedHelicalGear

    def __init__(
        self,
        module,
        gear1_teeth_number,
        gear2_teeth_number,
        gear1_width,
        gear2_width,
        pressure_angle=20.0,
        shaft_angle=90.0,
        gear1_helix_angle=None,
        clearance=0.0,
        backlash=0.0,
        **build_params,
    ):
        MeshGeometry.__init__(self)
        if gear1_helix_angle is None:
            g1_helix = shaft_angle / 2.0
            g2_helix = shaft_angle / 2.0
        else:
            g1_helix = gear1_helix_angle
            g2_helix = shaft_angle - gear1_helix_angle
        self.gear1 = self.gear1_cls(
            module,
            gear1_teeth_number,
            gear1_width,
            pressure_angle=pressure_angle,
            helix_angle=g1_helix,
            clearance=clearance,
            backlash=backlash,
        )
        self.gear2 = self.gear2_cls(
            module,
            gear2_teeth_number,
            gear2_width,
            pressure_angle=pressure_angle,
            helix_angle=g2_helix,
            clearance=clearance,
            backlash=backlash,
        )
        self.shaft_angle = np.radians(shaft_angle)
        self.build_params = build_params
        self._finalize()

    def _build(
        self,
        build_gear1=True,
        build_gear2=True,
        transform_gear2=True,
        gear1_build_args={},
        gear2_build_args={},
        **kv_args,
    ):
        parts = []
        if build_gear1:
            args = {**self.build_params, **kv_args, **gear1_build_args}
            parts.append(self.gear1.build(**args))
        if build_gear2:
            args = {**self.build_params, **kv_args, **gear2_build_args}
            gear2 = self.gear2.build(**args)
            if transform_gear2:
                ratio = self.gear1.z / self.gear2.z
                align_angle = 0.0 if self.gear2.z % 2 else np.pi / self.gear2.z
                align_angle += (self.gear2.twist_angle + self.gear1.twist_angle * ratio) / 2.0
                gear2 = gear2.copy()
                gear2 = gear2.translate(0.0, 0.0, -self.gear2.width / 2.0)
                gear2 = gear2.rotate_z(align_angle)
                gear2 = gear2.rotate((1.0, 0.0, 0.0), self.shaft_angle)
                gear2 = gear2.translate(self.gear1.r0 + self.gear2.r0, 0.0, self.gear1.width / 2.0)
            parts.append(gear2)
        if not parts:
            raise ValueError("Crossed gear pair has no parts to build")
        result = parts[0]
        for part in parts[1:]:
            result = boolean_union(result, part)
        return result


class HyperbolicGear(SpurGear):
    surface_splines = 2

    def __init__(
        self,
        module,
        teeth_number,
        width,
        twist_angle,
        pressure_angle=20.0,
        clearance=0.0,
        backlash=0.0,
        **build_params,
    ):
        super().__init__(
            module=module,
            teeth_number=teeth_number,
            width=width,
            pressure_angle=pressure_angle,
            helix_angle=0.0,
            clearance=clearance,
            backlash=backlash,
            **build_params,
        )
        self.twist_angle = np.radians(twist_angle)
        ln = np.cos(self.twist_angle) * self.r0
        ht = np.sin(self.twist_angle) * self.r0
        rpx = (self.r0 + ln) / 2.0
        rpy = ht / 2.0
        self.throat_r = np.sqrt(rpx**2 + rpy**2)
        # Rebuild the body now that twist_angle is set (super() built it untwisted).
        self._finalize()


class HyperbolicGearPair(GearBase):
    gear_cls = HyperbolicGear

    def __init__(
        self,
        module,
        gear1_teeth_number,
        width,
        shaft_angle,
        gear2_teeth_number=None,
        pressure_angle=20.0,
        clearance=0.0,
        backlash=0.0,
        **build_params,
    ):
        MeshGeometry.__init__(self)
        if gear2_teeth_number is None:
            gear2_teeth_number = gear1_teeth_number
        g1_r0 = module * gear1_teeth_number / 2.0
        g2_r0 = module * gear2_teeth_number / 2.0
        alpha = np.radians(shaft_angle / 2.0)
        hh = (width / 2.0) * np.tan(alpha)
        gear1_twist_angle = np.arcsin(hh / g1_r0) * 2.0
        gear2_twist_angle = np.arcsin(hh / g2_r0) * 2.0
        if np.isnan(gear1_twist_angle) or np.isnan(gear2_twist_angle):
            raise ValueError(
                "Impossible to calculate the twist angle for the given shaft angle / teeth number / gear width"
            )
        self.shaft_angle = np.radians(shaft_angle)
        self.gear1 = self.gear_cls(
            module,
            gear1_teeth_number,
            width,
            twist_angle=np.degrees(gear1_twist_angle),
            pressure_angle=pressure_angle,
            clearance=clearance,
            backlash=backlash,
        )
        self.gear2 = self.gear_cls(
            module,
            gear2_teeth_number,
            width,
            twist_angle=np.degrees(gear2_twist_angle),
            pressure_angle=pressure_angle,
            clearance=clearance,
            backlash=backlash,
        )
        self.build_params = build_params
        self._finalize()

    def _build(
        self,
        build_gear1=True,
        build_gear2=True,
        transform_gear2=True,
        gear1_build_args={},
        gear2_build_args={},
        **kv_args,
    ):
        parts = []
        if build_gear1:
            args = {**self.build_params, **kv_args, **gear1_build_args}
            parts.append(self.gear1.build(**args))
        if build_gear2:
            args = {**self.build_params, **kv_args, **gear2_build_args}
            gear2 = self.gear2.build(**args)
            if transform_gear2:
                ratio = self.gear1.z / self.gear2.z
                align_angle = 0.0 if self.gear2.z % 2 else np.pi / self.gear2.z
                align_angle += (self.gear2.twist_angle + self.gear1.twist_angle * ratio) / 2.0
                gear2 = gear2.copy()
                gear2 = gear2.translate(0.0, 0.0, -self.gear2.width / 2.0)
                gear2 = gear2.rotate_z(align_angle)
                gear2 = gear2.rotate((1.0, 0.0, 0.0), self.shaft_angle)
                gear2 = gear2.translate(
                    self.gear1.throat_r + self.gear2.throat_r, 0.0, self.gear1.width / 2.0
                )
            parts.append(gear2)
        if not parts:
            raise ValueError("Hyperbolic gear pair has no parts to build")
        result = parts[0]
        for part in parts[1:]:
            result = boolean_union(result, part)
        return result


# ---------------------------------------------------------------------------
# Workplane-style helpers retained for API compatibility. With mesh geometry removed,
# these operate on native ``MeshGeometry`` instances.
# ---------------------------------------------------------------------------
def gear(target, gear_, *build_args, **build_kv_args):
    """Build ``gear_`` and merge it onto ``target`` (a ``MeshGeometry``)."""
    gear_body = gear_.build(*build_args, **build_kv_args)
    if isinstance(target, MeshGeometry):
        return boolean_union(target, gear_body)
    return gear_body


def addGear(target, gear_, *build_args, **build_kv_args):
    return gear(target, gear_, *build_args, **build_kv_args)
