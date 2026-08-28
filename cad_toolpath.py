"""STEP-to-hot-wire geometry for WWCutter.

The module deliberately contains no Qt code.  It turns a STEP solid or shell
into two matched section contours, validates that their straight rulings follow
the imported surface, and intersects those rulings with the machine towers.
"""

from dataclasses import dataclass
from pathlib import Path

import numpy as np

try:
    import cadquery as cq
except ImportError:  # Defer the dependency error until STEP import is requested.
    cq = None


class CADImportError(RuntimeError):
    pass


class CADGeometryError(RuntimeError):
    pass


@dataclass
class FaceMesh:
    face_index: int
    vertices: np.ndarray
    triangles: list


@dataclass
class SectionCandidate:
    index: int
    label: str
    kind: str
    wire: object
    center: np.ndarray
    normal: np.ndarray
    area: float
    length: float
    face_index: int = None
    face: object = None


@dataclass
class StepModel:
    path: str
    shape: object
    sections: list
    face_meshes: list
    bounds_min: np.ndarray
    bounds_max: np.ndarray


@dataclass
class MachineGeometry:
    tower_span: float
    foam_left_gap: float
    horizontal_travel: float
    vertical_travel: float
    margin: float = 5.0


@dataclass
class CADToolpath:
    root_section: SectionCandidate
    tip_section: SectionCandidate
    root_points: np.ndarray
    tip_points: np.ndarray
    root_xy: np.ndarray
    tip_xy: np.ndarray
    cut_root: np.ndarray
    cut_tip: np.ndarray
    tower_left: np.ndarray
    tower_right: np.ndarray
    model_vertices: list
    axis_origin: np.ndarray
    axis: np.ndarray
    horizontal: np.ndarray
    vertical: np.ndarray
    stock_rotation: float
    surface_error: float
    deviation_detail: str
    limitations: tuple
    interior_count: int
    ignored_interior_count: int
    wire_length_min: float
    wire_length_max: float


def require_cadquery():
    if cq is None:
        raise CADImportError(
            "STEP support requires CadQuery. Install it with "
            "`python -m pip install cadquery`."
        )


def _array(vector):
    return np.asarray(vector.toTuple(), dtype=float)


def _unit(vector):
    vector = np.asarray(vector, dtype=float)
    length = np.linalg.norm(vector)
    if length <= 1e-12:
        raise CADGeometryError("Cannot determine a direction from coincident geometry.")
    return vector / length


def _wire_plane(wire):
    points = np.asarray([_array(p) for p in wire.sample(64)[0]])
    center = np.mean(points, axis=0)
    _, _, basis = np.linalg.svd(points - center, full_matrices=False)
    normal = _unit(basis[-1])
    deviation = float(np.max(np.abs((points - center) @ normal)))
    return center, normal, deviation


def _wire_area(wire, center, normal):
    points = np.asarray([_array(p) for p in wire.sample(128)[0]])
    vertical_hint = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(vertical_hint, normal)) > 0.9:
        vertical_hint = np.array([0.0, 1.0, 0.0])
    x_axis = _unit(np.cross(vertical_hint, normal))
    y_axis = _unit(np.cross(normal, x_axis))
    points_2d = np.column_stack(((points - center) @ x_axis,
                                 (points - center) @ y_axis))
    x, y = points_2d[:, 0], points_2d[:, 1]
    return abs(float(np.sum(x * np.roll(y, -1) - y * np.roll(x, -1))) * 0.5)


def _make_face_mesh(face, face_index, tolerance):
    vertices, triangles = face.tessellate(tolerance)
    return FaceMesh(
        face_index,
        np.asarray([_array(vertex) for vertex in vertices], dtype=float),
        [tuple(triangle) for triangle in triangles],
    )


def load_step_model(path, mesh_tolerance=1.5):
    """Load STEP geometry, using a deliberately coarse display-only mesh."""
    require_cadquery()
    path = str(Path(path))
    try:
        imported = cq.importers.importStep(path)
        shape = imported.val()
    except Exception as exc:
        raise CADImportError("Could not import STEP file: {}".format(exc)) from exc

    if not shape or shape.isNull():
        raise CADImportError("The STEP file does not contain usable geometry.")

    bounds = shape.BoundingBox()
    bounds_min = np.array([bounds.xmin, bounds.ymin, bounds.zmin], dtype=float)
    bounds_max = np.array([bounds.xmax, bounds.ymax, bounds.zmax], dtype=float)
    diagonal = max(np.linalg.norm(bounds_max - bounds_min), 1.0)
    planarity_tolerance = max(0.02, diagonal * 1e-5)
    display_tolerance = max(mesh_tolerance, diagonal / 500.0)

    face_meshes = []
    sections = []
    faces = shape.Faces()
    for face_index, face in enumerate(faces):
        try:
            face_meshes.append(_make_face_mesh(face, face_index, display_tolerance))
        except Exception:
            pass

        if face.geomType() != "PLANE":
            continue
        try:
            wire = face.outerWire()
            center = _array(face.Center())
            normal = _unit(_array(face.normalAt()))
            area = float(face.Area())
            length = float(wire.Length())
        except Exception:
            continue
        label = "Face {}  area {:.1f} mm²  center ({:.1f}, {:.1f}, {:.1f})".format(
            face_index + 1, area, *center
        )
        sections.append(SectionCandidate(
            len(sections), label, "face", wire, center, normal,
            area, length, face_index, face,
        ))

    # Open STEP shells may have no cap faces. Their free edges are combined into
    # selectable boundary loops so they remain useful inputs.
    boundary_edges = []
    for edge in shape.Edges():
        try:
            if len(edge.ancestors(shape, "Face").Faces()) == 1:
                boundary_edges.append(edge)
        except Exception:
            continue
    if boundary_edges:
        try:
            boundary_wires = cq.Wire.combine(boundary_edges, tol=planarity_tolerance)
        except Exception:
            boundary_wires = []
        for boundary_index, wire in enumerate(boundary_wires):
            try:
                center, normal, deviation = _wire_plane(wire)
                if deviation > planarity_tolerance:
                    continue
                area = _wire_area(wire, center, normal) if wire.Closed() else 0.0
                length = float(wire.Length())
            except Exception:
                continue
            label = "Boundary {}  length {:.1f} mm  center ({:.1f}, {:.1f}, {:.1f})".format(
                boundary_index + 1, length, *center
            )
            sections.append(SectionCandidate(
                len(sections), label, "boundary", wire, center, normal,
                area, length,
            ))

    if len(sections) < 2:
        raise CADGeometryError(
            "The model needs two planar end faces or two planar open boundaries."
        )
    return StepModel(path, shape, sections, face_meshes, bounds_min, bounds_max)


def _aligned_normals(first, second):
    second = second if np.dot(first, second) >= 0 else -second
    return _unit(first + second)


def section_frame(root, tip):
    axis = _aligned_normals(root.normal, tip.normal)
    if np.dot(tip.center - root.center, axis) < 0:
        axis = -axis
    vertical_hint = np.array([0.0, 0.0, 1.0])
    vertical = vertical_hint - np.dot(vertical_hint, axis) * axis
    if np.linalg.norm(vertical) < 0.2:
        vertical_hint = np.array([0.0, 1.0, 0.0])
        vertical = vertical_hint - np.dot(vertical_hint, axis) * axis
    vertical = _unit(vertical)
    horizontal = _unit(np.cross(vertical, axis))
    return root.center.copy(), axis, horizontal, vertical


def auto_detect_section_pair(model):
    """Choose the most plausible opposing end sections of a two-section loft."""
    best = None
    diagonal = max(np.linalg.norm(model.bounds_max - model.bounds_min), 1.0)
    for first_index, first in enumerate(model.sections):
        for second in model.sections[first_index + 1:]:
            parallel = abs(float(np.dot(first.normal, second.normal)))
            if parallel < np.cos(np.radians(12.0)):
                continue
            axis = _aligned_normals(first.normal, second.normal)
            separation = abs(float(np.dot(second.center - first.center, axis)))
            if separation < diagonal * 0.02:
                continue
            size_ratio = min(first.length, second.length) / max(first.length, second.length)
            kind_bonus = 1.15 if first.kind == second.kind == "face" else 1.0
            score = separation * (0.35 + 0.65 * size_ratio) * parallel * kind_bonus
            if best is None or score > best[0]:
                if max(first.area, second.area) > 0 and abs(first.area - second.area) > 1e-6:
                    # Conventional wing files put the larger root section near
                    # the left tower. Users can swap the pair in the UI.
                    pair = (first, second) if first.area >= second.area else (second, first)
                elif np.dot(second.center - first.center, axis) >= 0:
                    pair = first, second
                else:
                    pair = second, first
                best = score, pair
    if best is None:
        raise CADGeometryError(
            "No convincing pair of parallel end sections was found. Select them manually."
        )
    return best[1]


def _similarity_error(reference, candidate):
    reference = reference - np.mean(reference, axis=0)
    candidate = candidate - np.mean(candidate, axis=0)
    matrix = candidate.T @ reference
    left, singular, right = np.linalg.svd(matrix)
    rotation = left @ right
    scale = np.sum(singular) / max(np.sum(candidate * candidate), 1e-12)
    fitted = scale * candidate @ rotation
    denominator = max(np.sqrt(np.mean(np.sum(reference * reference, axis=1))), 1e-12)
    return float(np.sqrt(np.mean(np.sum((reference - fitted) ** 2, axis=1))) / denominator)


def _candidate_shifts(reference, candidate, count=16):
    def signature(points):
        centered = points - np.mean(points, axis=0)
        radial = np.linalg.norm(centered, axis=1)
        return (radial - np.mean(radial)) / max(np.std(radial), 1e-12)

    first = signature(reference)
    second = signature(candidate)
    correlation = np.fft.ifft(np.fft.fft(first) * np.conj(np.fft.fft(second))).real
    count = min(count, len(correlation))
    return np.argpartition(correlation, -count)[-count:]


def align_section_samples(root_xy, tip_xyz, horizontal, vertical):
    """Resolve independent STEP wire seams and orientations without undoing twist."""
    tip_xy = np.column_stack((tip_xyz @ horizontal, tip_xyz @ vertical))
    best = None
    for reversed_order in (False, True):
        candidate_xyz = tip_xyz[::-1] if reversed_order else tip_xyz
        candidate_xy = tip_xy[::-1] if reversed_order else tip_xy
        for shift in _candidate_shifts(root_xy, candidate_xy):
            shifted_xy = np.roll(candidate_xy, int(shift), axis=0)
            error = _similarity_error(root_xy, shifted_xy)
            if best is None or error < best[0]:
                best = error, np.roll(candidate_xyz, int(shift), axis=0)
    return best[1], best[0]


def _edge_in_face(edge, face):
    return any(candidate.isSame(edge) for candidate in face.Edges())


def _topological_edge_groups(model, root, tip):
    """Group each section's consecutive edges by their shared lateral face."""
    if root.face is None or tip.face is None:
        return None
    root_edges = root.wire.Edges()
    tip_edges = tip.wire.Edges()
    groups = []
    covered_root = set()
    covered_tip = set()
    for face in model.shape.Faces():
        if face.isSame(root.face) or face.isSame(tip.face):
            continue
        root_indices = [index for index, edge in enumerate(root_edges)
                        if _edge_in_face(edge, face)]
        tip_indices = [index for index, edge in enumerate(tip_edges)
                       if _edge_in_face(edge, face)]
        if not root_indices or not tip_indices:
            continue
        groups.append((min(root_indices),
                       [root_edges[index] for index in root_indices],
                       [tip_edges[index] for index in tip_indices]))
        covered_root.update(root_indices)
        covered_tip.update(tip_indices)
    if (len(covered_root) != len(root_edges)
            or len(covered_tip) != len(tip_edges)):
        return None
    groups.sort(key=lambda item: item[0])
    return [(root_group, tip_group) for _, root_group, tip_group in groups]


def _sample_topological_groups(groups, max_segment, max_points, closed):
    lengths = [
        (cq.Wire.assembleEdges(root_edges).Length(),
         cq.Wire.assembleEdges(tip_edges).Length())
        for root_edges, tip_edges in groups
    ]
    counts = [max(2, int(np.ceil(max(pair) / max_segment)) + 1)
              for pair in lengths]
    total = sum(count - 1 for count in counts)
    if total > max_points:
        scale = max_points / total
        counts = [max(2, int(np.floor((count - 1) * scale)) + 1)
                  for count in counts]

    root_result = []
    tip_result = []
    for group_index, ((root_edges, tip_edges), count) in enumerate(
            zip(groups, counts)):
        root_wire = cq.Wire.assembleEdges(root_edges)
        tip_wire = cq.Wire.assembleEdges(tip_edges)
        root_points = np.asarray([
            _array(point) for point in root_wire.sample(count)[0]
        ])
        tip_points = np.asarray([
            _array(point) for point in tip_wire.sample(count)[0]
        ])
        if root_result and np.linalg.norm(root_result[-1] - root_points[-1]) < (
                np.linalg.norm(root_result[-1] - root_points[0])):
            root_points = root_points[::-1]
        if tip_result:
            if np.linalg.norm(tip_result[-1] - tip_points[-1]) < np.linalg.norm(
                    tip_result[-1] - tip_points[0]):
                tip_points = tip_points[::-1]
        else:
            direct = (np.linalg.norm(root_points[0] - tip_points[0])
                      + np.linalg.norm(root_points[-1] - tip_points[-1]))
            reverse = (np.linalg.norm(root_points[0] - tip_points[-1])
                       + np.linalg.norm(root_points[-1] - tip_points[0]))
            if reverse < direct:
                tip_points = tip_points[::-1]
        if group_index:
            root_points = root_points[1:]
            tip_points = tip_points[1:]
        root_result.extend(root_points)
        tip_result.extend(tip_points)

    root_result = np.asarray(root_result)
    tip_result = np.asarray(tip_result)
    if closed:
        if np.allclose(root_result[0], root_result[-1]):
            root_result = root_result[:-1]
            tip_result = tip_result[:-1]
        root_result = np.vstack((root_result, root_result[0]))
        tip_result = np.vstack((tip_result, tip_result[0]))
    return root_result, tip_result


def _topological_edge_pairs(model, root, tip):
    """Pair cap edges through their common lateral face when STEP topology allows it."""
    root_edges = root.wire.Edges()
    tip_edges = tip.wire.Edges()
    if not root_edges or len(root_edges) != len(tip_edges):
        return None
    pairs = []
    used_tip_edges = set()
    for root_edge in root_edges:
        matched = None
        try:
            adjacent_faces = root_edge.ancestors(model.shape, "Face").Faces()
        except Exception:
            return None
        for face in adjacent_faces:
            if ((root.face is not None and face.isSame(root.face))
                    or (tip.face is not None and face.isSame(tip.face))):
                continue
            candidates = [
                (index, edge) for index, edge in enumerate(tip_edges)
                if index not in used_tip_edges and _edge_in_face(edge, face)
            ]
            if len(candidates) == 1:
                matched = candidates[0]
                break
        if matched is None:
            pairs = None
            break
        used_tip_edges.add(matched[0])
        pairs.append((root_edge, matched[1]))
    if pairs is not None and len(pairs) == len(tip_edges):
        return pairs

    # Some CAD systems export several consecutive boundary edges on one
    # lateral face. In that case topology identifies the face but not a unique
    # edge. Preserve cyclic edge order and choose the orientation/offset whose
    # edge types, relative lengths, and endpoints agree best.
    def lateral_faces(edge, cap):
        try:
            return [face for face in edge.ancestors(model.shape, "Face").Faces()
                    if cap is None or not face.isSame(cap)]
        except Exception:
            return []

    root_lateral = [lateral_faces(edge, root.face) for edge in root_edges]
    tip_lateral = [lateral_faces(edge, tip.face) for edge in tip_edges]
    root_total = max(sum(edge.Length() for edge in root_edges), 1e-12)
    tip_total = max(sum(edge.Length() for edge in tip_edges), 1e-12)
    best = None
    for reverse in (False, True):
        indices = list(range(len(tip_edges)))
        if reverse:
            indices.reverse()
        for shift in range(len(indices)):
            order = indices[shift:] + indices[:shift]
            score = 0.0
            valid = True
            for root_index, tip_index in enumerate(order):
                shared = any(
                    first.isSame(second)
                    for first in root_lateral[root_index]
                    for second in tip_lateral[tip_index]
                )
                if not shared:
                    valid = False
                    break
                root_edge = root_edges[root_index]
                tip_edge = tip_edges[tip_index]
                if root_edge.geomType() != tip_edge.geomType():
                    score += 1000.0
                root_fraction = root_edge.Length() / root_total
                tip_fraction = tip_edge.Length() / tip_total
                score += 10.0 * abs(root_fraction - tip_fraction)
                root_ends = np.asarray([_array(v) for v in root_edge.Vertices()])
                tip_ends = np.asarray([_array(v) for v in tip_edge.Vertices()])
                if len(root_ends) == len(tip_ends) == 2:
                    direct = (np.linalg.norm(root_ends[0] - tip_ends[0])
                              + np.linalg.norm(root_ends[1] - tip_ends[1]))
                    reverse_distance = (
                        np.linalg.norm(root_ends[0] - tip_ends[1])
                        + np.linalg.norm(root_ends[1] - tip_ends[0])
                    )
                    score += min(direct, reverse_distance) / max(
                        np.linalg.norm(root.center - tip.center), 1.0
                    )
            if valid and (best is None or score < best[0]):
                best = score, order
    if best is None:
        return None
    return [(root_edge, tip_edges[tip_index])
            for root_edge, tip_index in zip(root_edges, best[1])]


def _sample_topological_pairs(pairs, max_segment, max_points, closed):
    segment_counts = [
        max(2, int(np.ceil(max(root.Length(), tip.Length()) / max_segment)) + 1)
        for root, tip in pairs
    ]
    total = sum(count - 1 for count in segment_counts)
    if total > max_points:
        scale = max_points / total
        segment_counts = [max(2, int(np.floor((count - 1) * scale)) + 1)
                          for count in segment_counts]

    root_result = []
    tip_result = []
    fractions_for = lambda count: np.linspace(0.0, 1.0, count)
    for pair_index, ((root_edge, tip_edge), count) in enumerate(zip(pairs, segment_counts)):
        fractions = fractions_for(count)
        root_points = np.asarray([
            _array(point) for point in root_edge.positions(fractions)
        ])
        tip_points = np.asarray([
            _array(point) for point in tip_edge.positions(fractions)
        ])
        direct = (np.linalg.norm(root_points[0] - tip_points[0])
                  + np.linalg.norm(root_points[-1] - tip_points[-1]))
        reverse = (np.linalg.norm(root_points[0] - tip_points[-1])
                   + np.linalg.norm(root_points[-1] - tip_points[0]))
        if reverse < direct:
            tip_points = tip_points[::-1]
        if pair_index:
            root_points = root_points[1:]
            tip_points = tip_points[1:]
        root_result.extend(root_points)
        tip_result.extend(tip_points)

    root_result = np.asarray(root_result)
    tip_result = np.asarray(tip_result)
    if closed:
        if np.allclose(root_result[0], root_result[-1]):
            root_result = root_result[:-1]
            tip_result = tip_result[:-1]
        root_result = np.vstack((root_result, root_result[0]))
        tip_result = np.vstack((tip_result, tip_result[0]))
    return root_result, tip_result


def sample_matched_sections(model, root, tip, max_segment=1.0, max_points=2000):
    if root.wire.Closed() != tip.wire.Closed():
        raise CADGeometryError("Both selected sections must either be open or closed.")
    topological_groups = _topological_edge_groups(model, root, tip)
    if topological_groups:
        root_points, tip_points = _sample_topological_groups(
            topological_groups, max_segment, max_points, root.wire.Closed()
        )
        return root_points, tip_points, 0.0
    topological_pairs = _topological_edge_pairs(model, root, tip)
    if topological_pairs:
        root_points, tip_points = _sample_topological_pairs(
            topological_pairs, max_segment, max_points, root.wire.Closed()
        )
        return root_points, tip_points, 0.0

    sample_count = int(np.ceil(max(root.length, tip.length) / max_segment))
    sample_count = max(64, min(sample_count, max_points))
    root_points = np.asarray([_array(point) for point in root.wire.sample(sample_count)[0]])
    tip_points = np.asarray([_array(point) for point in tip.wire.sample(sample_count)[0]])

    _, _, horizontal, vertical = section_frame(root, tip)
    root_xy = np.column_stack((root_points @ horizontal, root_points @ vertical))
    if root.wire.Closed():
        tip_points, match_error = align_section_samples(
            root_xy, tip_points, horizontal, vertical
        )
        # Put the seam at the left-most matched ruling to create a predictable
        # leading-edge entry for ordinary wing profiles.
        tip_xy = np.column_stack((tip_points @ horizontal, tip_points @ vertical))
        start = int(np.argmin(root_xy[:, 0] + tip_xy[:, 0]))
        root_points = np.roll(root_points, -start, axis=0)
        tip_points = np.roll(tip_points, -start, axis=0)
        root_points = np.vstack((root_points, root_points[0]))
        tip_points = np.vstack((tip_points, tip_points[0]))
    else:
        direct = _similarity_error(
            root_xy,
            np.column_stack((tip_points @ horizontal, tip_points @ vertical)),
        )
        reversed_points = tip_points[::-1]
        reverse = _similarity_error(
            root_xy,
            np.column_stack((reversed_points @ horizontal,
                             reversed_points @ vertical)),
        )
        if reverse < direct:
            tip_points = reversed_points
            match_error = reverse
        else:
            match_error = direct
    return root_points, tip_points, match_error


def _lateral_faces_for_wires(model, root, tip, root_wire, tip_wire):
    result = []
    for face in model.shape.Faces():
        if ((root.face is not None and face.isSame(root.face))
                or (tip.face is not None and face.isSame(tip.face))):
            continue
        root_touching = any(_edge_in_face(edge, face)
                            for edge in root_wire.Edges())
        tip_touching = any(_edge_in_face(edge, face)
                           for edge in tip_wire.Edges())
        if root_touching and tip_touching:
            result.append(face)
    return result


def _surface_error(model, root, tip, root_points, tip_points,
                   lateral_faces=None):
    require_cadquery()
    if lateral_faces is None:
        lateral_faces = _lateral_faces_for_wires(
            model, root, tip, root.wire, tip.wire
        )
    if not lateral_faces:
        return 0.0
    lateral = cq.Compound.makeCompound(lateral_faces)
    path_count = len(root_points) - int(np.allclose(root_points[0], root_points[-1]))
    sample_indices = np.unique(np.linspace(0, path_count - 1, min(path_count, 48), dtype=int))
    maximum = 0.0
    for index in sample_indices:
        for fraction in (0.2, 0.4, 0.6, 0.8):
            point = root_points[index] + fraction * (tip_points[index] - root_points[index])
            vertex = cq.Vertex.makeVertex(*point)
            maximum = max(maximum, float(lateral.distance(vertex)))
    return maximum


def _points_to_closed_polyline(points, contour):
    """Return each point's 2D distance to a closed polyline."""
    points = np.asarray(points, dtype=float)
    contour = np.asarray(contour, dtype=float)
    if len(contour) > 1 and np.allclose(contour[0], contour[-1]):
        contour = contour[:-1]
    starts = contour
    deltas = np.roll(contour, -1, axis=0) - contour
    lengths_squared = np.maximum(np.sum(deltas * deltas, axis=1), 1e-12)
    result = np.empty(len(points), dtype=float)
    for start in range(0, len(points), 256):
        batch = points[start:start + 256]
        relative = batch[:, None, :] - starts[None, :, :]
        fractions = np.clip(
            np.sum(relative * deltas[None, :, :], axis=2)
            / lengths_squared[None, :],
            0.0, 1.0,
        )
        projections = (starts[None, :, :]
                       + fractions[:, :, None] * deltas[None, :, :])
        result[start:start + len(batch)] = np.sqrt(np.min(
            np.sum((batch[:, None, :] - projections) ** 2, axis=2), axis=1
        ))
    return result


def _sample_wire_edges(wire, max_segment, max_points=2000):
    """Sample a wire while retaining every CAD edge junction exactly."""
    edges = wire.Edges()
    counts = [max(2, int(np.ceil(edge.Length() / max_segment)) + 1)
              for edge in edges]
    total = sum(count - 1 for count in counts)
    if total > max_points:
        scale = max_points / total
        counts = [max(2, int(np.floor((count - 1) * scale)) + 1)
                  for count in counts]
    result = []
    for edge_index, (edge, count) in enumerate(zip(edges, counts)):
        points = np.asarray([_array(point) for point in edge.positions(
            np.linspace(0.0, 1.0, count)
        )])
        if edge_index:
            if np.linalg.norm(result[-1] - points[-1]) < np.linalg.norm(
                    result[-1] - points[0]):
                points = points[::-1]
            points = points[1:]
        result.extend(points)
    return np.asarray(result)


def _cross_section_error(model, root_points, tip_points, origin, axis,
                         horizontal, vertical, span, max_segment):
    """Compare intermediate CAD sections in both directions to the wire sweep."""
    root_depth = (root_points - origin) @ axis
    tip_depth = (tip_points - origin) @ axis
    depth_delta = tip_depth - root_depth
    if np.any(np.abs(depth_delta) < 1e-9):
        return 0.0, None

    worst = 0.0
    worst_fraction = None
    for section_fraction in (0.25, 0.5, 0.75):
        station = section_fraction * span
        ruling_fraction = (station - root_depth) / depth_delta
        generated = root_points + ruling_fraction[:, None] * (
            tip_points - root_points
        )
        generated_xy = np.column_stack((
            generated @ horizontal, generated @ vertical
        ))

        plane = cq.Plane(
            origin=tuple(origin + station * axis),
            xDir=tuple(horizontal), normal=tuple(axis),
        )
        try:
            section_shape = cq.Workplane(plane).add(model.shape).section().val()
            wires = section_shape.Wires()
            if not wires:
                continue
            # The hot wire follows the selected outer boundary. Closed internal
            # loops are diagnosed separately as unsupported features.
            cad_wire = max(wires, key=lambda wire: wire.Length())
            cad_points = _sample_wire_edges(cad_wire, max_segment)
        except Exception:
            continue
        cad_xy = np.column_stack((cad_points @ horizontal, cad_points @ vertical))
        # _surface_error already measures generated-to-CAD distance. This
        # reverse check catches portions of the requested section that the
        # generated ruled surface never reaches.
        deviation = float(np.max(
            _points_to_closed_polyline(cad_xy, generated_xy)
        ))
        if deviation > worst:
            worst = deviation
            worst_fraction = section_fraction
    return worst, worst_fraction


def _boundary_bow_error(model, root, tip, origin, axis, span):
    """Measure CAD boundary curves that a full-span wire must make straight."""
    excluded = [section.face for section in (root, tip)
                if section.face is not None]
    tolerance = max(span * 1e-5, 1e-5)
    worst = 0.0
    for face in model.shape.Faces():
        if any(face.isSame(cap) for cap in excluded):
            continue
        for edge in face.Edges():
            try:
                adjacent = edge.ancestors(model.shape, "Face").Faces()
                distinct = []
                for candidate in adjacent:
                    if not any(candidate.isSame(existing) for existing in distinct):
                        distinct.append(candidate)
                if len(distinct) < 2:
                    continue
            except Exception:
                continue
            vertices = edge.Vertices()
            if len(vertices) != 2:
                continue
            endpoints = np.asarray([_array(vertex) for vertex in vertices])
            depths = (endpoints - origin) @ axis
            connects_sections = (
                (abs(depths[0]) <= tolerance
                 and abs(depths[1] - span) <= tolerance)
                or (abs(depths[1]) <= tolerance
                    and abs(depths[0] - span) <= tolerance)
            )
            if not connects_sections:
                continue
            points = np.asarray([
                _array(point) for point in edge.positions(
                    np.linspace(0.0, 1.0, 65)
                )
            ])
            start, end = endpoints
            delta = end - start
            length_squared = float(np.dot(delta, delta))
            if length_squared <= 1e-12:
                continue
            fractions = np.clip(
                ((points - start) @ delta) / length_squared, 0.0, 1.0
            )
            chord = start + fractions[:, None] * delta
            worst = max(worst, float(np.max(np.linalg.norm(points - chord, axis=1))))
    return worst


def _choose_workspace_axes(root_points, tip_points, origin, axis, horizontal,
                           vertical, machine):
    """Rotate the stock about its span axis when that uses the workspace better."""
    root_relative = root_points - origin
    tip_relative = tip_points - origin
    root_depth = machine.foam_left_gap + root_relative @ axis
    tip_depth = machine.foam_left_gap + tip_relative @ axis
    depth_delta = tip_depth - root_depth
    if np.any(np.abs(depth_delta) < 1e-9):
        return horizontal, vertical, 0.0
    left_fraction = -root_depth / depth_delta
    right_fraction = (machine.tower_span - root_depth) / depth_delta
    available = np.array([
        machine.horizontal_travel - 2 * machine.margin,
        machine.vertical_travel - 2 * machine.margin,
    ])

    def evaluate(angle_degrees):
        angle = np.radians(angle_degrees)
        rotated_horizontal = np.cos(angle) * horizontal + np.sin(angle) * vertical
        rotated_vertical = -np.sin(angle) * horizontal + np.cos(angle) * vertical
        root_xy = np.column_stack((
            root_relative @ rotated_horizontal,
            root_relative @ rotated_vertical,
        ))
        tip_xy = np.column_stack((
            tip_relative @ rotated_horizontal,
            tip_relative @ rotated_vertical,
        ))
        delta_xy = tip_xy - root_xy
        left = root_xy + left_fraction[:, None] * delta_xy
        right = root_xy + right_fraction[:, None] * delta_xy
        extents = np.array([
            np.ptp(np.r_[left[:, 0], right[:, 0]]),
            np.ptp(np.r_[left[:, 1], right[:, 1]]),
        ])
        utilization = float(np.max(extents / np.maximum(available, 1e-9)))
        return utilization, rotated_horizontal, rotated_vertical

    initial = evaluate(0.0)
    if initial[0] <= 1.0:
        return horizontal, vertical, 0.0
    coarse = [(evaluate(angle)[0], float(angle))
              for angle in np.linspace(-90.0, 90.0, 181)]
    _, best_angle = min(coarse)
    fine_angles = np.linspace(best_angle - 1.0, best_angle + 1.0, 81)
    _, best_angle = min((evaluate(angle)[0], float(angle))
                        for angle in fine_angles)
    _, fitted_horizontal, fitted_vertical = evaluate(best_angle)
    return fitted_horizontal, fitted_vertical, best_angle


def _inner_wires(section):
    if section.face is None:
        return []
    return [wire for wire in section.face.Wires()
            if not wire.isSame(section.wire)]


def _wire_center(wire):
    points = np.asarray([_array(point) for point in wire.sample(64)[0]])
    return np.mean(points, axis=0)


def _match_inner_wires(root, tip, horizontal, vertical):
    root_wires = _inner_wires(root)
    tip_wires = _inner_wires(tip)
    if len(root_wires) != len(tip_wires):
        return None
    remaining = list(tip_wires)
    pairs = []
    scale = max(root.length, tip.length, 1.0)
    for root_wire in root_wires:
        root_center = _wire_center(root_wire) - root.center
        root_xy = np.array([
            np.dot(root_center, horizontal), np.dot(root_center, vertical)
        ])
        best = None
        for index, tip_wire in enumerate(remaining):
            tip_center = _wire_center(tip_wire) - tip.center
            tip_xy = np.array([
                np.dot(tip_center, horizontal), np.dot(tip_center, vertical)
            ])
            length_error = abs(np.log(max(root_wire.Length(), 1e-9)
                                      / max(tip_wire.Length(), 1e-9)))
            score = np.linalg.norm(root_xy - tip_xy) / scale + length_error
            if best is None or score < best[0]:
                best = score, index
        pairs.append((root_wire, remaining.pop(best[1])))
    return pairs


def _sample_inner_pair(root_wire, tip_wire, root, tip, horizontal, vertical,
                       max_segment):
    count = max(64, int(np.ceil(max(root_wire.Length(), tip_wire.Length())
                                 / max_segment)))
    root_points = np.asarray([
        _array(point) for point in root_wire.sample(count)[0]
    ])
    tip_original = np.asarray([
        _array(point) for point in tip_wire.sample(count)[0]
    ])
    root_center = np.mean(root_points, axis=0)
    root_signature = np.column_stack((
        (root_points - root_center) @ horizontal,
        (root_points - root_center) @ vertical,
    ))
    root_scale = max(np.sqrt(np.mean(np.sum(root_signature ** 2, axis=1))), 1e-12)
    root_signature /= root_scale
    best = None
    for reverse in (False, True):
        candidate = tip_original[::-1] if reverse else tip_original
        center = np.mean(candidate, axis=0)
        signature = np.column_stack((
            (candidate - center) @ horizontal,
            (candidate - center) @ vertical,
        ))
        candidate_scale = max(
            np.sqrt(np.mean(np.sum(signature ** 2, axis=1))), 1e-12
        )
        signature /= candidate_scale
        for shift in range(count):
            error = float(np.mean((root_signature
                                   - np.roll(signature, shift, axis=0)) ** 2))
            if best is None or error < best[0]:
                best = error, np.roll(candidate, shift, axis=0)
    tip_points = best[1]
    return (np.vstack((root_points, root_points[0])),
            np.vstack((tip_points, tip_points[0])))


def project_rulings(root_points, tip_points, origin, axis, horizontal, vertical,
                    machine):
    root_relative = root_points - origin
    tip_relative = tip_points - origin
    root_local = np.column_stack((
        root_relative @ horizontal,
        root_relative @ vertical,
        machine.foam_left_gap + root_relative @ axis,
    ))
    tip_local = np.column_stack((
        tip_relative @ horizontal,
        tip_relative @ vertical,
        machine.foam_left_gap + tip_relative @ axis,
    ))
    delta = tip_local - root_local
    if np.any(np.abs(delta[:, 2]) < 1e-9):
        raise CADGeometryError("A ruling is parallel to the tower planes.")
    left_fraction = -root_local[:, 2] / delta[:, 2]
    right_fraction = (machine.tower_span - root_local[:, 2]) / delta[:, 2]
    tower_left = root_local[:, :2] + left_fraction[:, None] * delta[:, :2]
    tower_right = root_local[:, :2] + right_fraction[:, None] * delta[:, :2]

    combined_x = np.r_[tower_left[:, 0], tower_right[:, 0]]
    combined_y = np.r_[tower_left[:, 1], tower_right[:, 1]]
    width = float(np.ptp(combined_x))
    height = float(np.ptp(combined_y))
    available_x = machine.horizontal_travel - 2 * machine.margin
    available_y = machine.vertical_travel - 2 * machine.margin
    if width > available_x + 1e-9 or height > available_y + 1e-9:
        raise CADGeometryError(
            "Projected tower paths need {:.1f} x {:.1f} mm, but the configured "
            "workspace provides {:.1f} x {:.1f} mm after margins.".format(
                width, height, available_x, available_y
            )
        )
    x_shift = machine.horizontal_travel * 0.5 - 0.5 * (combined_x.min() + combined_x.max())
    y_shift = machine.vertical_travel * 0.5 - 0.5 * (combined_y.min() + combined_y.max())
    shift = np.array([x_shift, y_shift])
    tower_left += shift
    tower_right += shift
    root_local[:, :2] += shift
    tip_local[:, :2] += shift
    return root_local, tip_local, tower_left, tower_right, shift


def build_cad_toolpath(model, root_index, tip_index, machine,
                       max_segment=1.0, include_internal=False):
    if root_index == tip_index:
        raise CADGeometryError("Choose two different end sections.")
    root = model.sections[root_index]
    tip = model.sections[tip_index]
    parallel = abs(float(np.dot(root.normal, tip.normal)))
    if parallel < np.cos(np.radians(12.0)):
        raise CADGeometryError("Selected end sections are not parallel enough for this cutter.")

    origin, axis, horizontal, vertical = section_frame(root, tip)
    span = float(np.dot(tip.center - root.center, axis))
    if span <= 1e-6:
        raise CADGeometryError("Selected end sections have no usable separation.")
    if machine.foam_left_gap < 0 or machine.foam_left_gap + span > machine.tower_span:
        raise CADGeometryError(
            "The {:.1f} mm model span at a {:.1f} mm left gap does not fit "
            "between towers {:.1f} mm apart.".format(
                span, machine.foam_left_gap, machine.tower_span
            )
        )

    outer_root_points, outer_tip_points, match_error = sample_matched_sections(
        model, root, tip, max_segment
    )
    horizontal, vertical, stock_rotation = _choose_workspace_axes(
        outer_root_points, outer_tip_points, origin, axis,
        horizontal, vertical, machine
    )
    ruling_error = _surface_error(
        model, root, tip, outer_root_points, outer_tip_points
    )
    boundary_bow_error = _boundary_bow_error(
        model, root, tip, origin, axis, span
    )
    section_error, section_fraction = _cross_section_error(
        model, outer_root_points, outer_tip_points,
        origin, axis, horizontal, vertical,
        span, max_segment,
    )

    limitations = []
    interior_paths = []
    root_inner = _inner_wires(root)
    tip_inner = _inner_wires(tip)
    ignored_interior_count = (
        max(len(root_inner), len(tip_inner)) if not include_internal else 0
    )
    if include_internal and (root_inner or tip_inner):
        inner_pairs = _match_inner_wires(root, tip, horizontal, vertical)
        if inner_pairs is None:
            limitations.append(
                "Interior contours do not pass through both selected end "
                "faces, so a full-span straight wire cannot cut them."
            )
        else:
            for root_wire, tip_wire in inner_pairs:
                inner_root, inner_tip = _sample_inner_pair(
                    root_wire, tip_wire, root, tip,
                    horizontal, vertical, max_segment,
                )
                faces = _lateral_faces_for_wires(
                    model, root, tip, root_wire, tip_wire
                )
                ruling_error = max(
                    ruling_error,
                    _surface_error(
                        model, root, tip, inner_root, inner_tip, faces
                    ),
                )
                interior_paths.append((inner_root, inner_tip))

    # Interior loops are reached through a slit from the outer seam and the
    # same slit is retraced after each loop. Ignored loops contribute neither
    # motion nor deviation.
    root_parts = [outer_root_points]
    tip_parts = [outer_tip_points]
    for inner_root, inner_tip in interior_paths:
        root_parts.extend((inner_root, outer_root_points[:1]))
        tip_parts.extend((inner_tip, outer_tip_points[:1]))
    root_points = np.vstack(root_parts)
    tip_points = np.vstack(tip_parts)
    root_local, tip_local, tower_left, tower_right, shift = project_rulings(
        root_points, tip_points, origin, axis, horizontal, vertical, machine
    )

    model_vertices = []
    for mesh in model.face_meshes:
        relative = mesh.vertices - origin
        transformed = np.column_stack((
            relative @ horizontal + shift[0],
            relative @ vertical + shift[1],
            machine.foam_left_gap + relative @ axis,
        ))
        model_vertices.append(transformed)

    wire_lengths = np.sqrt(
        machine.tower_span ** 2
        + (tower_right[:, 0] - tower_left[:, 0]) ** 2
        + (tower_right[:, 1] - tower_left[:, 1]) ** 2
    )
    # A poor similarity match is useful diagnostic information, but actual
    # ruled-surface deviation is the authoritative manufacturability measure.
    surface_error = max(
        ruling_error, section_error, boundary_bow_error, match_error * 0.01
    )
    if section_fraction is not None:
        if boundary_bow_error > 1e-6:
            boundary_detail = (
                " A CAD boundary curve that must be one straight wire ruling "
                "bows {:.3f} mm away from its endpoint chord; this part of the "
                "error is unavoidable by changing contour correspondence."
            ).format(boundary_bow_error)
        else:
            boundary_detail = ""
        deviation_detail = (
            "A straight hot wire can create only a ruled surface. At {:.0f}% "
            "of the foam span, the generated section and the STEP section "
            "differ by {:.3f} mm; the STEP skin changes nonlinearly between "
            "its end faces.{}"
        ).format(section_fraction * 100.0, section_error, boundary_detail)
    else:
        deviation_detail = (
            "The straight-wire rulings depart from the STEP skin by up to "
            "{:.3f} mm."
        ).format(surface_error)

    return CADToolpath(
        root, tip, root_points, tip_points,
        root_local[:, :2], tip_local[:, :2], root_local, tip_local,
        tower_left, tower_right,
        model_vertices, origin, axis, horizontal, vertical, stock_rotation,
        surface_error, deviation_detail, tuple(limitations), len(interior_paths),
        ignored_interior_count,
        float(wire_lengths.min()), float(wire_lengths.max()),
    )
