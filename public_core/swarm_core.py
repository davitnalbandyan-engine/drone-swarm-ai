#!/usr/bin/env python3
"""
Drone Swarm AI — Public Core (Reduced)

What this proves publicly:
- Hierarchical coverage control: TRAVEL / SWEEP phases
- Shared swarm_goal selection using frontier utility (info gain vs distance)
- Connectivity-aware gating: regroup if graph disconnects; local goals only when safe
- Minimal metrics + RViz markers (drones + links + goals + swarm_goal + phase text)

Scope: simulation-only coordination logic (no physics, no sensors, no collision guarantees).
"""

import math
import random
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class SwarmCore(Node):
    def __init__(self):
        super().__init__("drone_swarm_public_core")

        # ---------- ROS ----------
        self.pub = self.create_publisher(MarkerArray, "/swarm/markers", 10)
        self.metrics_pub = self.create_publisher(String, "/swarm/metrics", 10)
        self.cmd_sub = self.create_subscription(String, "/swarm/cmd", self.on_cmd, 10)

        # ---------- Mode ----------
        self.mode = "coverage"  # coverage | return | flock | pause
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.tick)

        # ---------- Swarm ----------
        self.N = 8
        self.coordinator_id = 0
        self.pos = []
        self.vel = []

        # ---------- Space ----------
        self.bound = 5.0
        self.home = (0.0, 0.0)

        # ---------- Connectivity ----------
        self.comms_radius = 3.2
        self.local_allow_frac = 0.80  # allow local goals only if d_to_coord <= frac*comms
        self.tether_frac = 0.92       # if far, pull to coordinator

        # ---------- Boids-ish rules ----------
        self.neighbor_radius = 2.0
        self.separation_radius = 0.6
        self.w_sep = 1.7
        self.w_coh = 0.30
        self.w_align = 0.60
        self.w_bound = 0.85

        # ---------- Speeds ----------
        self.max_speed = 1.6
        self.max_speed_coord = 1.5

        # ---------- Coverage grid (frontier / utility) ----------
        self.grid_n = 20
        self.total_cells = self.grid_n * self.grid_n
        self.visited = set()  # visited cells
        self.sensor_radius_m = 0.9  # simple "mark visited around drone"
        self.visit_threshold = 0.10  # (reduced model: visit is boolean)

        # ---------- Goals ----------
        self.local_goals = [(0.0, 0.0) for _ in range(self.N)]
        self.swarm_goal = (0.0, 0.0)

        self.local_goal_period = 0.40
        self.swarm_goal_period = 0.22
        self._last_local_goal_t = 0.0
        self._last_swarm_goal_t = 0.0

        # ---------- Hierarchy phases ----------
        self.cover_phase = "TRAVEL"  # TRAVEL or SWEEP
        self.phase_timer = 0.0
        self.travel_max_time = 6.5
        self.sweep_time = 2.8
        self.travel_reach_dist = 1.0

        # ---------- Frontier utility ----------
        self.gain_window_r = 2
        self.dist_penalty = 0.22
        self.frontier_samples = 220
        self._last_gain = 0

        # ---------- Success (simple) ----------
        self.success_percent = 85.0
        self._success_hold = 0.0
        self.success_hold_s = 4.0

        self.reset()

    # ---------------- Commands ----------------
    def on_cmd(self, msg: String):
        m = msg.data.strip().lower()
        if m in ["coverage", "return", "flock", "pause"]:
            self.mode = m
            self.get_logger().info(f"Mode = {self.mode}")
            if m == "coverage":
                self.cover_phase = "TRAVEL"
                self.phase_timer = 0.0
        elif m == "reset":
            self.reset()
            self.get_logger().info("Reset done.")

    # ---------------- Reset ----------------
    def reset(self):
        self.visited.clear()

        self.pos = []
        self.vel = []
        # coordinator near center
        self.pos.append([random.uniform(-0.5, 0.5), random.uniform(-0.5, 0.5), 1.0])
        self.vel.append([0.0, 0.0, 0.0])

        # others spread
        for _ in range(self.N - 1):
            self.pos.append([random.uniform(-2.0, 2.0), random.uniform(-2.0, 2.0), 1.0])
            self.vel.append([random.uniform(-0.2, 0.2), random.uniform(-0.2, 0.2), 0.0])

        self.local_goals = [(0.0, 0.0) for _ in range(self.N)]
        self.swarm_goal = (0.0, 0.0)
        self.cover_phase = "TRAVEL"
        self.phase_timer = 0.0
        self._success_hold = 0.0
        self._last_gain = 0

    # ---------------- Grid helpers ----------------
    def _cell_index(self, x, y):
        u = (x + self.bound) / (2.0 * self.bound)
        v = (y + self.bound) / (2.0 * self.bound)
        u = clamp(u, 0.0, 0.999999)
        v = clamp(v, 0.0, 0.999999)
        return int(u * self.grid_n), int(v * self.grid_n)

    def _cell_center(self, ix, iy):
        cell = (2.0 * self.bound) / self.grid_n
        x = -self.bound + (ix + 0.5) * cell
        y = -self.bound + (iy + 0.5) * cell
        return x, y

    def _mark_visited(self):
        # reduced sensor model: mark cells within sensor_radius as visited
        cell = (2.0 * self.bound) / self.grid_n
        r_cells = int(math.ceil(self.sensor_radius_m / cell))
        for i in range(self.N):
            px, py, _ = self.pos[i]
            ix0, iy0 = self._cell_index(px, py)
            for dx in range(-r_cells, r_cells + 1):
                ix = ix0 + dx
                if ix < 0 or ix >= self.grid_n:
                    continue
                for dy in range(-r_cells, r_cells + 1):
                    iy = iy0 + dy
                    if iy < 0 or iy >= self.grid_n:
                        continue
                    cx, cy = self._cell_center(ix, iy)
                    d = math.hypot(cx - px, cy - py)
                    if d <= self.sensor_radius_m:
                        self.visited.add((ix, iy))

    def _known_percent(self):
        return 100.0 * len(self.visited) / float(self.total_cells)

    # ---------------- Connectivity ----------------
    def _lcc_size(self):
        # largest connected component under comms radius
        r2 = self.comms_radius * self.comms_radius
        adj = [[] for _ in range(self.N)]
        for i in range(self.N):
            for j in range(i + 1, self.N):
                dx = self.pos[j][0] - self.pos[i][0]
                dy = self.pos[j][1] - self.pos[i][1]
                if dx * dx + dy * dy <= r2:
                    adj[i].append(j)
                    adj[j].append(i)

        seen = [False] * self.N
        best = 0
        for i in range(self.N):
            if seen[i]:
                continue
            stack = [i]
            seen[i] = True
            size = 0
            while stack:
                k = stack.pop()
                size += 1
                for nb in adj[k]:
                    if not seen[nb]:
                        seen[nb] = True
                        stack.append(nb)
            best = max(best, size)
        return best

    # ---------------- Frontier + Utility ----------------
    def _window_gain(self, ix, iy):
        r = self.gain_window_r
        gain = 0
        for dx in range(-r, r + 1):
            x = ix + dx
            if x < 0 or x >= self.grid_n:
                continue
            for dy in range(-r, r + 1):
                y = iy + dy
                if y < 0 or y >= self.grid_n:
                    continue
                if (x, y) not in self.visited:
                    gain += 1
        return gain

    def _frontier_cells(self):
        # frontier = unvisited with a visited neighbor
        f = []
        for ix in range(self.grid_n):
            for iy in range(self.grid_n):
                if (ix, iy) in self.visited:
                    continue
                if ((ix - 1, iy) in self.visited or (ix + 1, iy) in self.visited or
                    (ix, iy - 1) in self.visited or (ix, iy + 1) in self.visited):
                    f.append((ix, iy))
        return f

    def _update_swarm_goal(self):
        frontier = self._frontier_cells()
        candidates = frontier if frontier else [(ix, iy) for ix in range(self.grid_n) for iy in range(self.grid_n) if (ix, iy) not in self.visited]
        if not candidates:
            self.swarm_goal = self.home
            self._last_gain = 0
            return

        ci = self.coordinator_id
        cpx, cpy, _ = self.pos[ci]

        best_score = -1e18
        best_xy = self.swarm_goal
        best_gain = 0

        for _ in range(self.frontier_samples):
            ix, iy = random.choice(candidates)
            gain = self._window_gain(ix, iy)
            tx, ty = self._cell_center(ix, iy)
            dist = math.hypot(tx - cpx, ty - cpy)
            score = gain - self.dist_penalty * dist + random.uniform(-0.05, 0.05)
            if score > best_score:
                best_score = score
                best_xy = (tx, ty)
                best_gain = gain

        self.swarm_goal = best_xy
        self._last_gain = best_gain

    def _update_local_goals(self):
        # local goals = nearest unvisited cell to each drone (cheap)
        unvisited = [(ix, iy) for ix in range(self.grid_n) for iy in range(self.grid_n) if (ix, iy) not in self.visited]
        if not unvisited:
            for i in range(self.N):
                self.local_goals[i] = self.home
            return

        samples = 55
        for i in range(self.N):
            if i == self.coordinator_id:
                continue
            px, py, _ = self.pos[i]
            best = None
            best_d2 = 1e18
            for _ in range(samples):
                ix, iy = random.choice(unvisited)
                tx, ty = self._cell_center(ix, iy)
                d2 = (tx - px) ** 2 + (ty - py) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best = (tx, ty)
            self.local_goals[i] = best if best else (px, py)

    # ---------------- Main tick ----------------
    def tick(self):
        if self.mode == "pause":
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # coverage map update (reduced)
        if self.mode == "coverage":
            self._mark_visited()

        known = self._known_percent()

        # completion -> return
        if self.mode == "coverage":
            if known >= self.success_percent:
                self._success_hold += self.dt
            else:
                self._success_hold = 0.0
            if self._success_hold >= self.success_hold_s:
                self.mode = "return"
                self.get_logger().info("MISSION COMPLETE -> RETURN")

        # connectivity health
        lcc = self._lcc_size()
        regroup = (self.mode == "coverage" and lcc < self.N)

        # phase logic
        if self.mode == "coverage":
            self.phase_timer += self.dt
            # if disconnected -> TRAVEL immediately
            if regroup and self.cover_phase != "TRAVEL":
                self.cover_phase = "TRAVEL"
                self.phase_timer = 0.0

            # switch TRAVEL->SWEEP when close or timeout
            cpx, cpy, _ = self.pos[self.coordinator_id]
            sgx, sgy = self.swarm_goal
            d_goal = math.hypot(sgx - cpx, sgy - cpy)

            if self.cover_phase == "TRAVEL":
                if d_goal <= self.travel_reach_dist or self.phase_timer >= self.travel_max_time:
                    self.cover_phase = "SWEEP"
                    self.phase_timer = 0.0
            else:  # SWEEP
                if self.phase_timer >= self.sweep_time:
                    self.cover_phase = "TRAVEL"
                    self.phase_timer = 0.0
                    # force new goal immediately after sweep
                    self._update_swarm_goal()

        # goals update
        if self.mode == "coverage":
            if (now - self._last_swarm_goal_t) >= self.swarm_goal_period:
                self._update_swarm_goal()
                self._last_swarm_goal_t = now
            if (now - self._last_local_goal_t) >= self.local_goal_period:
                self._update_local_goals()
                self._last_local_goal_t = now

        # ---------- coordinator motion: follow swarm_goal ----------
        ci = self.coordinator_id
        cpx, cpy, _ = self.pos[ci]
        cvx, cvy, _ = self.vel[ci]

        if self.mode == "coverage":
            tx, ty = self.swarm_goal
        elif self.mode == "return":
            tx, ty = self.home
        else:
            tx, ty = 0.0, 0.0

        # simple PD-ish
        ax_c = 1.25 * (tx - cpx) - 0.35 * cvx
        ay_c = 1.25 * (ty - cpy) - 0.35 * cvy
        if abs(cpx) > self.bound:
            ax_c -= cpx
        if abs(cpy) > self.bound:
            ay_c -= cpy

        # integrate coordinator
        self.vel[ci][0] += ax_c * self.dt
        self.vel[ci][1] += ay_c * self.dt
        sp = math.hypot(self.vel[ci][0], self.vel[ci][1])
        if sp > self.max_speed_coord:
            self.vel[ci][0] = self.vel[ci][0] / sp * self.max_speed_coord
            self.vel[ci][1] = self.vel[ci][1] / sp * self.max_speed_coord
        self.pos[ci][0] += self.vel[ci][0] * self.dt
        self.pos[ci][1] += self.vel[ci][1] * self.dt

        # ---------- other drones ----------
        # phase weights
        if self.mode == "coverage":
            if self.cover_phase == "TRAVEL":
                w_goal = 0.0     # local goal OFF
                w_swarm = 0.60   # swarm_goal ON
            else:  # SWEEP
                w_goal = 0.90
                w_swarm = 0.12
        elif self.mode == "return":
            w_goal = 0.65
            w_swarm = 0.0
        else:
            w_goal = 0.0
            w_swarm = 0.0

        for i in range(self.N):
            if i == ci:
                continue

            px, py, _ = self.pos[i]
            vx, vy, _ = self.vel[i]

            # neighbors
            neigh = []
            for j in range(self.N):
                if j == i:
                    continue
                qx, qy, _ = self.pos[j]
                dx = qx - px
                dy = qy - py
                d2 = dx * dx + dy * dy
                if d2 <= self.neighbor_radius * self.neighbor_radius:
                    neigh.append((j, dx, dy, d2))

            # separation
            ax_sep = ay_sep = 0.0
            for (_, dx, dy, d2) in neigh:
                if d2 < 1e-9:
                    continue
                if d2 <= self.separation_radius * self.separation_radius:
                    inv = 1.0 / d2
                    ax_sep -= dx * inv
                    ay_sep -= dy * inv

            # cohesion
            ax_coh = ay_coh = 0.0
            if neigh:
                ncx = sum(self.pos[j][0] for (j, _, __, ___) in neigh) / len(neigh)
                ncy = sum(self.pos[j][1] for (j, _, __, ___) in neigh) / len(neigh)
                ax_coh = ncx - px
                ay_coh = ncy - py

            # alignment
            ax_al = ay_al = 0.0
            if neigh:
                avx = sum(self.vel[j][0] for (j, _, __, ___) in neigh) / len(neigh)
                avy = sum(self.vel[j][1] for (j, _, __, ___) in neigh) / len(neigh)
                ax_al = avx - vx
                ay_al = avy - vy

            # boundary
            ax_b = ay_b = 0.0
            if abs(px) > self.bound:
                ax_b -= px
            if abs(py) > self.bound:
                ay_b -= py

            # vectors
            # local goal
            gx, gy = self.local_goals[i]
            ax_goal = gx - px
            ay_goal = gy - py

            # swarm goal
            sgx, sgy = self.swarm_goal
            ax_sg = sgx - px
            ay_sg = sgy - py

            # tether/regroup to coordinator when far
            dx_c = self.pos[ci][0] - px
            dy_c = self.pos[ci][1] - py
            d_c = math.hypot(dx_c, dy_c)

            ax_tether = ay_tether = 0.0
            if self.mode == "coverage":
                if d_c > self.tether_frac * self.comms_radius:
                    ax_tether = dx_c
                    ay_tether = dy_c

            # gate local goal in coverage SWEEP
            goal_scale = 1.0
            swarm_scale = 1.0

            if self.mode == "coverage":
                if self.cover_phase == "TRAVEL":
                    goal_scale = 0.0
                else:
                    if d_c > (self.local_allow_frac * self.comms_radius):
                        goal_scale = 0.0
                        swarm_scale = 1.35  # follow swarm_goal more if not allowed

            # force regroup stronger if disconnected
            w_tether = 2.0 if regroup else 1.2

            ax = (
                self.w_sep * ax_sep +
                self.w_coh * ax_coh +
                self.w_align * ax_al +
                self.w_bound * ax_b +
                (w_goal * goal_scale) * ax_goal +
                (w_swarm * swarm_scale) * ax_sg +
                w_tether * ax_tether
            )
            ay = (
                self.w_sep * ay_sep +
                self.w_coh * ay_coh +
                self.w_align * ay_al +
                self.w_bound * ay_b +
                (w_goal * goal_scale) * ay_goal +
                (w_swarm * swarm_scale) * ay_sg +
                w_tether * ay_tether
            )

            # integrate
            self.vel[i][0] += ax * self.dt
            self.vel[i][1] += ay * self.dt
            sp = math.hypot(self.vel[i][0], self.vel[i][1])
            if sp > self.max_speed:
                self.vel[i][0] = self.vel[i][0] / sp * self.max_speed
                self.vel[i][1] = self.vel[i][1] / sp * self.max_speed

            self.pos[i][0] += self.vel[i][0] * self.dt
            self.pos[i][1] += self.vel[i][1] * self.dt

        # publish markers + metrics
        self.publish_markers(known, lcc, regroup)
        self.publish_metrics(known, lcc, regroup)

    # ---------------- Visualization ----------------
    def publish_markers(self, known, lcc, regroup):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Swarm goal
        sg = Marker()
        sg.header.frame_id = "map"
        sg.header.stamp = stamp
        sg.ns = "swarm_goal"
        sg.id = 1
        sg.type = Marker.SPHERE
        sg.action = Marker.ADD if self.mode == "coverage" else Marker.DELETE
        sg.pose.position.x = float(self.swarm_goal[0])
        sg.pose.position.y = float(self.swarm_goal[1])
        sg.pose.position.z = 1.0
        sg.scale.x = sg.scale.y = sg.scale.z = 0.35
        sg.color.r, sg.color.g, sg.color.b, sg.color.a = 1.0, 1.0, 0.2, 0.95
        arr.markers.append(sg)

        # Phase text
        txt = Marker()
        txt.header.frame_id = "map"
        txt.header.stamp = stamp
        txt.ns = "ui"
        txt.id = 2
        txt.type = Marker.TEXT_VIEW_FACING
        if self.mode == "coverage":
            txt.action = Marker.ADD
            txt.pose.position.x = -4.6
            txt.pose.position.y = 4.6
            txt.pose.position.z = 2.2
            txt.scale.z = 0.32
            txt.color.r, txt.color.g, txt.color.b, txt.color.a = 0.95, 0.95, 0.98, 1.0
            txt.text = f"PHASE: {self.cover_phase} | known={known:.1f}% | conn={lcc}/{self.N}" + (" | REGROUP" if regroup else "")
        else:
            txt.action = Marker.DELETE
        arr.markers.append(txt)

        # Local goals (only visible in SWEEP to prove hierarchy)
        g = Marker()
        g.header.frame_id = "map"
        g.header.stamp = stamp
        g.ns = "local_goals"
        g.id = 3
        g.type = Marker.SPHERE_LIST
        if self.mode == "coverage" and self.cover_phase == "SWEEP":
            g.action = Marker.ADD
            g.scale.x = g.scale.y = g.scale.z = 0.11
            g.color.r, g.color.g, g.color.b, g.color.a = 1.0, 0.2, 0.2, 0.85
            g.points = []
            for i in range(self.N):
                if i == self.coordinator_id:
                    continue
                p = Point()
                p.x = float(self.local_goals[i][0])
                p.y = float(self.local_goals[i][1])
                p.z = 1.0
                g.points.append(p)
        else:
            g.action = Marker.DELETE
        arr.markers.append(g)

        # Drones
        for i in range(self.N):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "drones"
            m.id = 100 + i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(self.pos[i][0])
            m.pose.position.y = float(self.pos[i][1])
            m.pose.position.z = 1.0
            if i == self.coordinator_id:
                m.scale.x = m.scale.y = m.scale.z = 0.34
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.82, 0.15, 1.0
            else:
                m.scale.x = m.scale.y = m.scale.z = 0.25
                m.color.r, m.color.g, m.color.b, m.color.a = 0.2, 0.8, 1.0, 1.0
            arr.markers.append(m)

        # Links (comms)
        link = Marker()
        link.header.frame_id = "map"
        link.header.stamp = stamp
        link.ns = "links"
        link.id = 200
        link.type = Marker.LINE_LIST
        link.action = Marker.ADD
        link.scale.x = 0.03
        link.color.r, link.color.g, link.color.b, link.color.a = 1.0, 1.0, 1.0, 0.75
        link.points = []

        r2 = self.comms_radius * self.comms_radius
        for i in range(self.N):
            for j in range(i + 1, self.N):
                dx = self.pos[j][0] - self.pos[i][0]
                dy = self.pos[j][1] - self.pos[i][1]
                if dx * dx + dy * dy <= r2:
                    pA = Point()
                    pA.x, pA.y, pA.z = float(self.pos[i][0]), float(self.pos[i][1]), 1.0
                    pB = Point()
                    pB.x, pB.y, pB.z = float(self.pos[j][0]), float(self.pos[j][1]), 1.0
                    link.points.append(pA)
                    link.points.append(pB)
        arr.markers.append(link)

        self.pub.publish(arr)

    def publish_metrics(self, known, lcc, regroup):
        msg = String()
        msg.data = (
            f"mode={self.mode} "
            f"phase={self.cover_phase if self.mode=='coverage' else '-'} "
            f"known={known:.1f}% target={self.success_percent:.0f}% "
            f"conn={lcc}/{self.N} regroup={1 if regroup else 0} "
            f"swarm_goal=({self.swarm_goal[0]:.2f},{self.swarm_goal[1]:.2f}) "
            f"gain={self._last_gain}"
        )
        self.metrics_pub.publish(msg)


def main():
    rclpy.init()
    node = SwarmCore()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
