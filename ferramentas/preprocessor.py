from __future__ import annotations

import itertools
import sys
from typing import Dict, List, Tuple, Optional, TextIO

import matplotlib.pyplot as plt
import networkx as nx
import shapely as spl
import shapely.geometry.base

Segment = spl.LineString
Target = spl.Point
Depot = spl.Point
Obstacle = spl.Polygon
VisibilityGraph = nx.Graph
Direction = Tuple[float, float]


def _pt(p: spl.Point) -> Tuple[float, float]:
    return (p.x, p.y)


def _axis_value(p: spl.Point, ref: spl.Point, direction: Direction) -> float:
    dx, dy = direction
    return (p.x - ref.x) * dx + (p.y - ref.y) * dy


def _crosses_interior(a: spl.Point, b: spl.Point, obs: spl.Polygon) -> bool:
    seg = spl.LineString([_pt(a), _pt(b)])
    shrunken_obs = obs.buffer(-1e-6)
    if shrunken_obs.is_empty:
        return False
    return seg.intersects(shrunken_obs)


def _free_of_all(a: spl.Point, b: spl.Point, obstacles: List[Obstacle]) -> bool:
    return not any(_crosses_interior(a, b, o) for o in obstacles)


def _extract_blocked_segments(
    seg: spl.LineString,
    obstacles: List[Obstacle],
    ref: spl.Point,
    direction: Direction
) -> Tuple[List[Tuple[spl.Point, spl.Point]], spl.Polygon]:
    blocked = []
    obs_union = spl.Polygon()

    for obs in obstacles:
        if not seg.intersects(obs):
            continue

        inter = seg.intersection(obs)
        if inter.is_empty:
            continue

        obs_union = obs_union.union(obs)

        subs = []
        if inter.geom_type == 'LineString':
            subs = [inter]
        elif inter.geom_type in ('MultiLineString', 'GeometryCollection'):
            subs = [g for g in inter.geoms if g.geom_type == 'LineString']

        for sub in subs:
            coords = list(sub.coords)
            p1, p2 = spl.Point(coords[0]), spl.Point(coords[-1])
            if _axis_value(p1, ref, direction) > _axis_value(p2, ref, direction):
                p1, p2 = p2, p1
            blocked.append((p1, p2))

    blocked.sort(key=lambda t: _axis_value(t[0], ref, direction))
    return blocked, obs_union


def coverage_line_detour(
    start: Target,
    end: Target,
    obstacles: List[Obstacle],
    direction: Direction,
) -> Tuple[List[spl.Point], float]:
    ref = start
    if _axis_value(start, ref, direction) > _axis_value(end, ref, direction):
        start, end = end, start

    seg = Segment([_pt(start), _pt(end)])
    blocked, obs_union = _extract_blocked_segments(
        seg, obstacles, ref, direction)

    if not blocked:
        return [start, end], 0.0

    if obs_union.geom_type == 'MultiPolygon':
        polygons = [spl.Polygon(p) for p in obs_union.geoms]
    else:
        polygons = [obs_union]

    border = [spl.Point(x, y) for p in polygons for x, y in p.exterior.coords]
    shrunken_obs_union = obs_union.buffer(-1e-6)

    def _crosses(a: spl.Point, b: spl.Point) -> bool:
        if shrunken_obs_union.is_empty:
            return False
        return spl.LineString([_pt(a), _pt(b)]).intersects(shrunken_obs_union)

    border.extend([spl.Point(0, 0), spl.Point(0, 0)])
    src, tgt = len(border) - 2, len(border) - 1
    n = len(border)

    G: VisibilityGraph = nx.Graph()
    G.add_nodes_from(range(n))

    for i in range(n - 2):
        for j in range(i + 1, n - 2):
            if not _crosses(border[i], border[j]):
                G.add_edge(i, j, weight=border[i].distance(border[j]))

    full_path, total_cost = [start], 0.0
    current_pt = start

    for entry, exit_pt in blocked:
        total_cost += current_pt.distance(entry)
        full_path.append(entry)
        border[src], border[tgt] = entry, exit_pt

        G.remove_edges_from(list(G.edges(src)))
        G.remove_edges_from(list(G.edges(tgt)))

        for i in range(n - 2):
            for node in (src, tgt):
                if not _crosses(border[i], border[node]):
                    G.add_edge(
                        i, node, weight=border[i].distance(border[node]))

        if not _crosses(entry, exit_pt):
            G.add_edge(src, tgt, weight=entry.distance(exit_pt))

        try:
            idx_path = nx.shortest_path(
                G, source=src, target=tgt, weight='weight')
            pts = [border[k] for k in idx_path]
            total_cost += sum(pts[k].distance(pts[k + 1])
                              for k in range(len(pts) - 1))
            full_path.extend(pts[1:])
            current_pt = exit_pt
        except nx.NetworkXNoPath:
            raise RuntimeError("Could not find shortest path")

    full_path.append(end)
    total_cost += current_pt.distance(end)

    return full_path, total_cost - start.distance(end)


def free_transition_detour(
    s: spl.Point,
    t: spl.Point,
    obstacles: List[Obstacle],
) -> Tuple[List[spl.Point], float]:
    if _free_of_all(s, t, obstacles):
        return [s, t], 0.0

    nodes: List[spl.Point] = [s, t]
    for obs in obstacles:
        nodes.extend(spl.Point(c) for c in obs.exterior.coords[:-1])

    G: VisibilityGraph = nx.Graph()
    G.add_nodes_from(range(len(nodes)))

    for i, j in itertools.combinations(range(len(nodes)), 2):
        if _free_of_all(nodes[i], nodes[j], obstacles):
            G.add_edge(i, j, weight=nodes[i].distance(nodes[j]))

    try:
        idx_path = nx.shortest_path(G, source=0, target=1, weight='weight')
        pts = [nodes[k] for k in idx_path]
        cost = sum(pts[k].distance(pts[k + 1]) for k in range(len(pts) - 1))
        return pts, cost - s.distance(t)
    except nx.NetworkXNoPath:
        raise RuntimeError("Could not find the shortest path.")


class CoverageLine:
    def __init__(self, start: Target, end: Target) -> None:
        self._start = start
        self._end = end
        self.segments: List[Segment] = [Segment([_pt(start), _pt(end)])]

    def start(self) -> Target:
        return self._start

    def end(self) -> Target:
        return self._end


class PlaneContext:
    def __init__(self, safety_margin: float = 0.0) -> None:
        # Margem de segurança (m): distância pela qual cada obstáculo é
        # expandido antes do cálculo dos desvios. Zero mantém o obstáculo original.
        self.safety_margin = safety_margin
        self.lines: List[CoverageLine] = []
        self.depots: List[Depot] = []
        self.obstacles: List[Obstacle] = []
        self.true_obstacles: List[Obstacle] = []

    def add_depot(self, depot: Depot) -> None:
        self.depots.append(depot)

    def add_line(self, start: Target, end: Target) -> None:
        self.lines.append(CoverageLine(start, end))

    def add_obstacle(self, obstacle: Obstacle) -> None:
        if isinstance(obstacle, Obstacle) and obstacle.is_valid:
            self.true_obstacles.append(obstacle)
            if self.safety_margin > 0:
                # join_style=2: junção em quina (mitre), preserva os vértices
                self.obstacles.append(
                    obstacle.buffer(self.safety_margin, join_style=2))
            else:
                self.obstacles.append(obstacle)
        else:
            raise RuntimeError("Obstacles must be simple polygons.")

    def all_targets(self) -> List[Target]:
        return [p for line in self.lines for p in (line.start(), line.end())]


class PathPlanningContext:
    def __init__(self, plane: PlaneContext, direction: Direction) -> None:
        self.plane = plane
        self.direction = direction
        self.line_results: Dict[Tuple[int, int],
                                Tuple[List[spl.Point], float]] = {}
        self.transition_results: Dict[Tuple[int, int],
                                      Tuple[List[spl.Point], float]] = {}
        self.all_transition_results: Dict[Tuple[int, int],
                                          Tuple[List[spl.Point], float]] = {}

    def compute_all(self) -> List[str]:
        self.line_results.clear()
        self.transition_results.clear()
        self.all_transition_results.clear()
        warnings: List[str] = []

        for i, line in enumerate(self.plane.lines):
            s_id = 2 * i + 1
            e_id = 2 * i + 2
            try:
                path, delta_c = coverage_line_detour(
                    line.start(), line.end(), self.plane.obstacles, self.direction
                )
                self.line_results[(s_id, e_id)] = (path, delta_c)
            except RuntimeError as exc:
                warnings.append(f"Coverage line ({s_id},{e_id}): {exc}")

        nodes: List[spl.Point] = []
        if self.plane.depots:
            nodes.append(self.plane.depots[0])
        nodes.extend(self.plane.all_targets())

        for i, j in itertools.combinations(range(len(nodes)), 2):
            if i > 0 and j > 0 and (i - 1) // 2 == (j - 1) // 2:
                continue

            s, t = nodes[i], nodes[j]
            s_id, e_id = i, j
            try:
                pts, dc = free_transition_detour(s, t, self.plane.obstacles)
                self.all_transition_results[(s_id, e_id)] = (pts, dc)
                if dc > 1e-10:
                    self.transition_results[(s_id, e_id)] = (pts, dc)
            except RuntimeError as exc:
                warnings.append(f"Transition ({s_id},{e_id}): {exc}")

        return warnings


class IO:
    @staticmethod
    def load_plane_from_file(path: str, plane: PlaneContext) -> None:
        with open(path, "r", encoding="utf-8") as f:
            lines = [ln.rstrip("\n") for ln in f]

        if not lines:
            raise ValueError(f"Input file '{path}' is empty.")

        depot_coords = [float(c) for c in lines[0].split()]
        if len(depot_coords) < 2:
            raise ValueError("First line must contain depot coordinates: x y")
        plane.add_depot(Depot(depot_coords[0], depot_coords[1]))

        point_pair: List[List[float]] = []
        for lineno, raw in enumerate(lines[1:], start=2):
            coords = [float(c) for c in raw.split()]
            if not coords:
                continue
            if len(coords) < 2:
                raise ValueError(
                    f"Line {lineno}: expected 'x y', got: {raw!r}")
            point_pair.append(coords)
            if len(point_pair) == 2:
                plane.add_line(
                    Target(point_pair[0][0], point_pair[0][1]),
                    Target(point_pair[1][0], point_pair[1][1]),
                )
                point_pair.clear()

        if point_pair:
            coords_str = " ".join(map(str, point_pair[0]))
            raise ValueError(
                f"Odd number of target points — last line has no pair: {coords_str}"
            )

    @staticmethod
    def serialize_results(plane: PlaneContext, paths: PathPlanningContext) -> str:
        parts: List[str] = []

        for (s_id, e_id), (_, dc) in sorted(paths.line_results.items()):
            if dc > 1e-10:
                parts.append(f"{s_id} {e_id} {dc:.6f}")

        for (s_id, e_id), (_, dc) in sorted(paths.transition_results.items()):
            parts.append(f"{s_id} {e_id} {dc:.6f}")

        if plane.true_obstacles:
            parts.append("@obstacles")
            for obs in plane.true_obstacles:
                for x, y in obs.exterior.coords[:-1]:
                    parts.append(f"{x} {y}")
                parts.append(";")

        parts.append("@freesegments")

        for (s_id, e_id), (path, _) in sorted(paths.line_results.items()):
            seg_strs = []
            for k in range(len(path) - 1):
                p1, p2 = path[k], path[k+1]
                seg_strs.append(f"({p1.x} {p1.y}, {p2.x} {p2.y})")
            parts.append(f"{s_id} {e_id} " + " ".join(seg_strs) + " ;")

        for (s_id, e_id), (path, _) in sorted(paths.all_transition_results.items()):
            seg_strs = []
            for k in range(len(path) - 1):
                p1, p2 = path[k], path[k+1]
                seg_strs.append(f"({p1.x} {p1.y}, {p2.x} {p2.y})")
            parts.append(f"{s_id} {e_id} " + " ".join(seg_strs) + " ;")

        return "\n".join(parts)

    @staticmethod
    def export_results(
        plane: PlaneContext,
        paths: PathPlanningContext,
        out_file: Optional[str] = None,
    ) -> None:
        content = IO.serialize_results(plane, paths)
        if out_file:
            with open(out_file, "w", encoding="utf-8") as f:
                f.write(content)
        else:
            sys.stdout.write(content + "\n")


class Graphics:
    def __init__(
        self,
        plane: PlaneContext,
        paths: PathPlanningContext,
        output_file: Optional[str] = None,
        on_obstacle_added: Optional[callable] = None,
    ):
        self.plane = plane
        self.paths = paths
        self.output_file = output_file
        self._on_obstacle_added = on_obstacle_added or (lambda obs: None)

        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.obstacle_points: List[spl.Point] = []
        self.detour_artists: List = []
        self.transition_artists: List = []
        self._placement_artists: List = []
        self._show_transitions: bool = True

        self._setup_plot()
        self.fig.canvas.mpl_connect('key_press_event', self.on_key)

    def _setup_plot(self) -> None:
        self.ax.grid(True, linestyle='--', alpha=0.5)
        self.ax.set_title(
            "Coverage lines\n",
            fontsize=10,
        )

        for i, line in enumerate(self.plane.lines):
            self.ax.plot(
                [line.start().x, line.end().x],
                [line.start().y, line.end().y],
                color='black', linewidth=1.0, zorder=3,
            )
            for j, p in enumerate((line.start(), line.end())):
                tid = 2 * i + j + 1
                self.ax.scatter(p.x, p.y, color='black', s=30, zorder=5)
                self.ax.annotate(
                    str(tid), (p.x, p.y),
                    textcoords='offset points', xytext=(4, 4),
                    fontsize=8, color='navy',
                )

        for idx, depot in enumerate(self.plane.depots):
            self.ax.plot(depot.x, depot.y, '*', color="black",
                         markersize=5, zorder=7)
            self.ax.annotate(
                f'D{idx}', (depot.x, depot.y),
                textcoords='offset points', xytext=(4, 4),
                fontsize=12, color='black', fontweight='bold',
            )

    def _clear_placement_artists(self) -> None:
        for art in self._placement_artists:
            art.remove()
        self._placement_artists.clear()

    def on_key(self, event) -> None:
        if not event.inaxes:
            return

        if event.key == 'p':
            p = spl.Point(event.xdata, event.ydata)
            self.obstacle_points.append(p)
            art, = self.ax.plot(p.x, p.y, marker='o',
                                color='gray', markersize=5, zorder=7)
            self._placement_artists.append(art)
            self.fig.canvas.draw_idle()

        elif event.key == 'F':
            if len(self.obstacle_points) < 3:
                print("Need at least 3 points to form a polygon.")
                return

            poly = spl.Polygon([(q.x, q.y) for q in self.obstacle_points])

            if not poly.is_valid:
                print("Polygon is self-intersecting; discarded.")
                self._clear_placement_artists()
                self.obstacle_points.clear()
                return

            try:
                self.plane.add_obstacle(poly)
            except RuntimeError as exc:
                print(f"Could not add obstacle: {exc}")
                self._clear_placement_artists()
                self.obstacle_points.clear()
                return

            self._on_obstacle_added(poly)

            xs_c = [q.x for q in self.obstacle_points]
            ys_c = [q.y for q in self.obstacle_points]
            self.ax.fill(xs_c, ys_c, alpha=0.35, color='gray', zorder=2)

            self._clear_placement_artists()
            self.obstacle_points.clear()
            self.fig.canvas.draw_idle()

        elif event.key == 'C':
            for artist in self.detour_artists:
                artist.remove()
            self.detour_artists.clear()

            for artist in self.transition_artists:
                artist.remove()
            self.transition_artists.clear()

            warnings = self.paths.compute_all()
            for w in warnings:
                print(f"Warning: {w}")

            content = IO.serialize_results(self.plane, self.paths)
            if self.output_file:
                with open(self.output_file, "w", encoding="utf-8") as f:
                    f.write(content)
            else:
                sys.stdout.write(content + "\n")

            for (s_id, e_id), (path, dc) in self.paths.line_results.items():
                if dc > 1e-10 and len(path) > 1:
                    xs = [q.x for q in path]
                    ys = [q.y for q in path]
                    art, = self.ax.plot(
                        xs, ys, color='gray', alpha=0.8,
                        linestyle='--', linewidth=3.0, zorder=4,
                    )
                    self.detour_artists.append(art)

            for (s_id, e_id), (path, _) in self.paths.transition_results.items():
                if len(path) > 1:
                    xs = [q.x for q in path]
                    ys = [q.y for q in path]
                    art, = self.ax.plot(
                        xs, ys, color='steelblue', alpha=0.6,
                        linestyle=':', linewidth=1.5, zorder=3,
                    )
                    art.set_visible(self._show_transitions)
                    self.transition_artists.append(art)

            self.fig.canvas.draw_idle()

        elif event.key == 'T':
            self._show_transitions = not self._show_transitions
            for art in self.transition_artists:
                art.set_visible(self._show_transitions)
            self.fig.canvas.draw_idle()

    def show(self) -> None:
        plt.tight_layout()
        plt.show()


def main() -> None:
    if len(sys.argv) < 4:
        print(
            "Usage: python preprocessor.py <input_file> <dx> <dy> "
            "[optional_output_file] [safety_margin_m]")
        sys.exit(1)

    input_file = sys.argv[1]
    dx, dy = float(sys.argv[2]), float(sys.argv[3])
    output_file = sys.argv[4] if len(sys.argv) > 4 else None
    safety_margin = float(sys.argv[5]) if len(sys.argv) > 5 else 0.0

    mag = (dx ** 2 + dy ** 2) ** 0.5
    if mag < 1e-12:
        print("Direction vector must be non-zero.")
        sys.exit(1)

    direction: Direction = (dx / mag, dy / mag)

    plane = PlaneContext(safety_margin)
    IO.load_plane_from_file(input_file, plane)

    paths = PathPlanningContext(plane, direction)

    ui = Graphics(
        plane, paths, output_file
    )
    ui.show()


if __name__ == '__main__':
    main()
