"""Monotonic phase timing and sampled controller-feedback velocities.

Velocity is a finite difference of SDK feedback, not a commanded speed or an
independent hardware measurement. Missing samples and gaps are never filled.
"""
from __future__ import annotations

from collections import defaultdict, deque
from copy import deepcopy
import math


def vector(value, length):
    if not isinstance(value, (list, tuple)) or len(value) < length:
        return None
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in value[:length]):
        return None
    return list(value[:length])


class PerformanceTracker:
    def __init__(self, started_monotonic, *, simulated):
        self.started = started_monotonic
        self.simulated = simulated
        self.last_elapsed = 0.0
        self.phase = "other"
        self.resume_phase = "other"
        self.phases = defaultdict(float)
        self.previous = {}
        self.samples = deque(maxlen=600)
        self.stats = {"tcp": {"distance": 0.0, "seconds": 0.0, "peak": 0.0, "intervals": 0},
                      "joints": {"distance": [0.0] * 6, "seconds": 0.0, "peak": [0.0] * 6, "intervals": 0}}
        self.feedback_count = 0
        self.invalid_intervals = 0
        self.sample_sources = set()
        self.sample_intervals = []

    def consume(self, row):
        elapsed = float(row["elapsed_s"])
        self.phases[self.phase] += max(0.0, elapsed - self.last_elapsed)
        self.last_elapsed = elapsed
        event, stage, data = row["event"], row["stage"], row.get("data") or {}
        if event in {"operator_confirmation_required", "relative_gap_requested"}:
            self.resume_phase = self.phase
            self.phase = "confirmation" if event == "operator_confirmation_required" else "gap"
        elif event in {"operator_confirmed", "relative_gap_chosen"}:
            self.phase = self.resume_phase
        elif event == "motion_telemetry":
            self._feedback(data)
        elif event == "gripper_commanded" or stage == "gripper":
            self.phase = "gripper_wait"
        elif "commanded_pose" in data or "commanded_angles" in data or event == "pose_polled":
            self.phase = "motion"
        elif stage in {"move_pose", "move_start", "place_transfer", "motion"}:
            self.phase = "motion"
        elif stage in {"semantic", "transcription", "voice_input"}:
            self.phase = "ai"
        elif stage in {"perception", "depth"}:
            self.phase = "perception"
        elif stage in {"planning", "place_planning", "tool_resolution", "preferred_ik", "place_preflight"}:
            self.phase = "planning"
        elif stage == "preflight":
            self.phase = "preflight"

    def _feedback(self, data):
        self.feedback_count += 1
        self.sample_sources.add(data.get("source", "unavailable"))
        chart = {"elapsed_s": float(data.get("sample_time_s", self.started)) - self.started,
                 "tcp_speed_mm_s": None, "joint_speeds_deg_s": None, "phase": data.get("label", "")}
        for channel, key, count in (("tcp", "coords", 3), ("joints", "angles", 6)):
            current = vector(data.get(key), count)
            t = data.get(channel + "_time_s")
            if current is None or type(t) not in (int, float) or not math.isfinite(t):
                self.previous.pop(channel, None)
                continue
            prior = self.previous.get(channel)
            self.previous[channel] = (t, current, data.get("label"))
            if not prior or prior[2] != data.get("label"):
                continue
            dt = t - prior[0]
            delta = [current[i] - prior[1][i] for i in range(count)]
            if not .005 <= dt <= 1.0 or (channel == "joints" and any(abs(v) > 180 for v in delta)):
                self.invalid_intervals += 1
                continue
            stat = self.stats[channel]
            stat["seconds"] += dt
            stat["intervals"] += 1
            if channel == "tcp":
                distance = math.sqrt(sum(v * v for v in delta))
                speed = distance / dt
                stat["distance"] += distance
                stat["peak"] = max(stat["peak"], speed)
                chart["tcp_speed_mm_s"] = speed
                self.sample_intervals.append(dt)
                if len(self.sample_intervals) > 600:
                    del self.sample_intervals[0]
            else:
                speeds = [abs(v) / dt for v in delta]
                for i, speed in enumerate(speeds):
                    stat["distance"][i] += abs(delta[i])
                    stat["peak"][i] = max(stat["peak"][i], speed)
                chart["joint_speeds_deg_s"] = speeds
        self.samples.append(chart)

    def result(self, elapsed=None):
        elapsed = self.last_elapsed if elapsed is None else max(self.last_elapsed, float(elapsed))
        phases = dict(self.phases)
        phases[self.phase] = phases.get(self.phase, 0) + elapsed - self.last_elapsed
        tcp, joints = self.stats["tcp"], self.stats["joints"]
        return {"schema_version": 1, "elapsed_s": elapsed,
                "phases_s": {k: round(v, 6) for k, v in phases.items()}, "simulated": self.simulated,
                "measurement_source": "synthetic_feedback" if self.simulated else "controller_feedback",
                "feedback_sources": sorted(self.sample_sources), "feedback_samples": self.feedback_count,
                "invalid_intervals": self.invalid_intervals,
                "effective_tcp_sample_hz": len(self.sample_intervals) / sum(self.sample_intervals) if self.sample_intervals else None,
                "tcp": {"mean_mm_s": tcp["distance"] / tcp["seconds"] if tcp["seconds"] else None,
                        "sampled_peak_mm_s": tcp["peak"] if tcp["intervals"] else None, "valid_intervals": tcp["intervals"]},
                "joints": {"mean_deg_s": [v / joints["seconds"] for v in joints["distance"]] if joints["seconds"] else None,
                           "sampled_peak_deg_s": list(joints["peak"]) if joints["intervals"] else None, "valid_intervals": joints["intervals"]},
                "samples": deepcopy(list(self.samples)), "chart_sample_limit": 600,
                "limitations": ["Finite differences of timestamped SDK feedback; includes sampled pauses and arrival checks.",
                                "No interpolation over missing feedback, changed motion phases, or gaps longer than one second.",
                                "Sampled peaks are not continuous-time peak speed; full raw feedback is in motion_events.jsonl.",
                                "Synthetic feedback is not hardware speed evidence."]}
