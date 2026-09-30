"""Read saved CSV runs and serve an offline review dashboard."""

import ast
import csv
import json
import math
import shutil
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlsplit


PAGE = Path(__file__).resolve().parent.parent / "review" / "index.html"
STATUS_FLAGS = ("picked_up", "slip", "impact_x", "impact_y", "impact_z", "roll_over")


def parse_value(text):
    """Turn a CSV cell back into a number or a small SDK tuple when possible."""
    if text is None or text == "":
        return None
    if text in ("True", "False"):
        return 1 if text == "True" else 0
    try:
        value = float(text)
        return value if math.isfinite(value) else None
    except ValueError:
        pass
    if len(text) < 1000 and text[0] in "([":
        try:
            value = ast.literal_eval(text)
            if isinstance(value, (list, tuple)):
                return [parse_value(str(item)) for item in value]
        except (ValueError, SyntaxError, TypeError):
            pass
    return text


class RunStore:
    def __init__(self, data_dir, max_points=10000, close_tof_mm=200):
        self.data_dir = Path(data_dir)
        self.max_points = max_points
        self.close_tof_mm = close_tof_mm

    def list_runs(self):
        """List direct child directories containing saved run data."""
        if not self.data_dir.exists():
            return []
        runs = []
        for path in self.data_dir.iterdir():
            if path.is_symlink() or not path.is_dir():
                continue
            csv_files = [file for file in path.glob("*.csv")
                         if file.is_file() and not file.is_symlink()]
            csv_names = sorted(file.stem for file in csv_files)
            summary_path = path / "run_summary.json"
            files = list(csv_files)
            if summary_path.is_file() and not summary_path.is_symlink():
                files.append(summary_path)
            if files:
                revision = max(file.stat().st_mtime_ns for file in files)
                runs.append({"name": path.name, "streams": csv_names,
                             "revision": revision})
        return sorted(runs, key=lambda item: item["name"], reverse=True)

    def _run_path(self, name):
        if name not in {run["name"] for run in self.list_runs()}:
            raise ValueError("unknown run")
        return self.data_dir / name

    def _summary(self, run_path):
        path = run_path / "run_summary.json"
        if not path.exists() or path.is_symlink():
            return {}
        with path.open(encoding="utf-8") as file:
            return json.load(file)

    def load_run(self, name):
        """Read one run, keeping at most max_points chart samples per CSV."""
        run_path = self._run_path(name)
        summary = self._summary(run_path)
        streams = {}
        issues = []
        if summary.get("status") in ("failed", "interrupted", "stopped"):
            issues.append({"level": "error", "time_s": None,
                           "message": f"Run {summary['status']}: {summary.get('error') or 'stopped early'}"})
        elif summary.get("status") == "no_safe_direction":
            issues.append({"level": "warning", "time_s": None,
                           "message": summary.get("error") or "No safe direction found for exploration"})
        if summary.get("dropped_csv_rows", 0):
            issues.append({"level": "warning", "time_s": None,
                           "message": f"CSV queue dropped {summary['dropped_csv_rows']} rows"})
        
        camera_health = summary.get("camera_health") or {}

        if camera_health.get("skipped_reads", 0):
            issues.append({
                "level": "warning",
                "time_s": None,
                "message": (
                    "Camera skipped {} unavailable/broken reads; "
                    "recovered {} gaps. Last: {}"
                ).format(
                    camera_health["skipped_reads"],
                    camera_health.get("recovered_gaps", 0),
                    camera_health.get("last_skip_reason")
                    or "unknown",
                ),
            })
            
        for error in summary.get("logger_errors", []):
            issues.append({"level": "error", "time_s": None, "message": str(error)})
        for cell, alignment in (summary.get("exploration") or {}).get("alignments", {}).items():
            if alignment.get("status") == "stalled" or alignment.get("status", "").startswith("skipped_"):
                issues.append({"level": "warning", "time_s": None,
                               "message": f"Alignment {cell}: {alignment.get('reason') or alignment['status']}"})
        for cell, alignment in (summary.get("exploration") or {}).get("heading_alignments", {}).items():
            if alignment.get("status") not in ("applied", "no_suitable_wall"):
                issues.append({"level": "warning", "time_s": None,
                               "message": f"Wall heading {cell}: {alignment.get('reason') or alignment['status']}"})
        motion_stop = (summary.get("exploration") or {}).get("last_motion_stop") or {}
        lane = (summary.get("exploration") or {}).get("last_ir_lane") or {}
        round2_lane = (summary.get("round2_navigation") or {}).get("last_ir_lane") or {}
        for alignment in (summary.get("round2_alignment") or {}).get("events", []):
            if alignment.get("status") == "stopped":
                issues.append({"level": "warning", "time_s": None,
                               "message": "Round 2 alignment {} at {}: {}".format(
                                   alignment.get("phase"), alignment.get("cell"), alignment.get("error"))})
        if round2_lane.get("status") == "aborted":
            issues.append({"level": "warning", "time_s": None,
                           "message": "Round 2 IR lane {}: aborted (not retained)".format(
                               round2_lane.get("cells"))})
        if lane.get("status") in ("blocked", "aborted"):
            issues.append({"level": "warning", "time_s": None,
                           "message": "IR lane {}: {} (not retained)".format(
                               lane.get("cells"), lane["status"])})
        if motion_stop.get("center_confirmed") is False:
            issues.append({"level": "warning", "time_s": None,
                           "message": "Emergency stop at cell {} was {:.3f} m from planned center".format(
                                          motion_stop.get("center_cell"),
                                          motion_stop.get("planned_center_error_m", 0.0))})
        if motion_stop.get("wall_confirmed") is False:
            issues.append({"level": "warning", "time_s": None,
                           "message": "Emergency obstacle was not confirmed as a grid wall; topology was preserved."})
        for inspection in (summary.get("exploration") or {}).get("wall_inspections", []):
            if inspection.get("status") == "stopped":
                issues.append({"level": "error", "time_s": None,
                               "message": "Target inspection stopped at cell {} direction {}: {}".format(
                                   inspection.get("cell"), inspection.get("direction"),
                                   inspection.get("reason", "unknown cause"))})
            for target in inspection.get("targets", []):
                if target.get("status") not in ("fire_command_accepted",):
                    issues.append({"level": "warning", "time_s": None,
                                   "message": "Target {} / {} at cell {}: {}".format(
                                       target.get("color"), target.get("shape"),
                                       inspection.get("cell"), target.get("status"))})
        progress = (summary.get("exploration") or {}).get("target_progress") or {}
        if progress.get("status") == "stopped" and progress.get("interrupted_from"):
            issues.append({"level": "warning", "time_s": None,
                           "message": "Target inspection interrupted at cell {} direction {} while {} "
                                      "(action {})".format(progress.get("cell"),
                                                           progress.get("direction"),
                                                           progress["interrupted_from"],
                                                           progress.get("action_state"))})
        for end in ("rear", "front"):
            for recovery in summary.get(end + "_ir_recoveries", []):
                issues.append({"level": "warning" if recovery["status"] != "cleared" else "info",
                               "time_s": recovery["elapsed_s"],
                               "message": "{} IR {} recovery {} ({:.3f} m){}{}".format(
                                   end.title(), recovery["side"], recovery["status"],
                                   recovery["distance_m"],
                                   " via " + recovery["direction"] if recovery.get("direction") else "",
                                   ": " + recovery["reason"] if recovery.get("reason") else "")})

        for path in sorted(run_path.glob("*.csv")):
            if not path.is_file() or path.is_symlink():
                continue
            name_of_stream = path.stem
            with path.open(newline="", encoding="utf-8") as file:
                row_count = max(0, sum(1 for _ in csv.reader(file)) - 1)
            if self.max_points == 1:
                sample_indices = {row_count - 1}
            elif row_count <= self.max_points:
                sample_indices = set(range(row_count))
            else:
                sample_indices = {
                    round(index * (row_count - 1) / (self.max_points - 1))
                    for index in range(self.max_points)
                }
            samples = []
            gap_count = 0
            largest_gap = 0.0
            previous_time = None
            first_time = None
            last_time = None
            active_flags = {}
            tof_was_close = False
            tof_was_invalid = False
            ir_was_detected = {}
            stream_frequency = summary.get("stream_settings", {}).get(name_of_stream, {}).get("frequency_hz")
            gap_limit = max(1.5, 3 / stream_frequency) if isinstance(stream_frequency, (int, float)) and stream_frequency > 0 else 1.5

            with path.open(newline="", encoding="utf-8") as file:
                reader = csv.DictReader(file)
                columns = [field for field in (reader.fieldnames or [])
                           if field not in ("timestamp", "elapsed_s", "relative_time")]
                ir_settings = {end: summary.get(end + "_ir_settings")
                               for end in ("rear", "front")} if name_of_stream == "adapter" else {}
                ir_column_indices = {}
                for end, settings in ir_settings.items():
                    if settings is None:
                        continue
                    for side in ("right", "left"):
                        port = settings[side]
                        column = "io_{}".format((port["id"] - 1) * 2 + port["port"])
                        ir_column_indices[end, side] = columns.index(column) if column in columns else None
                        ir_was_detected[end, side] = False
                for row_number, row in enumerate(reader):
                    time_text = row.get("elapsed_s") or row.get("relative_time")
                    try:
                        elapsed = float(time_text)
                    except (TypeError, ValueError):
                        continue
                    values = [parse_value(row.get(column)) for column in columns]
                    if first_time is None:
                        first_time = elapsed
                    if previous_time is not None and elapsed - previous_time > gap_limit:
                        gap = elapsed - previous_time
                        gap_count += 1
                        largest_gap = max(largest_gap, gap)
                        if gap_count <= 10:
                            issues.append({"level": "warning", "time_s": round(elapsed, 2),
                                           "message": f"{name_of_stream}: no samples for {gap:.2f} s"})
                    previous_time = elapsed
                    last_time = elapsed

                    if name_of_stream == "status":
                        for column, value in zip(columns, values):
                            short_name = column[7:] if column.startswith("status_") else column
                            if short_name not in STATUS_FLAGS:
                                continue
                            active = value not in (None, 0, False)
                            if active and not active_flags.get(short_name) and len(issues) < 80:
                                issues.append({"level": "warning", "time_s": round(elapsed, 2),
                                               "message": f"status: {short_name} detected"})
                            active_flags[short_name] = active

                    for (end, side), index in ir_column_indices.items():
                        settings = ir_settings[end]
                        io = values[index] if index is not None else None
                        active_io = settings[side].get("active_io", settings.get("active_io"))
                        detected = io == active_io if io in (0, 1) else False
                        if detected and not ir_was_detected[end, side] and len(issues) < 80:
                            issues.append({"level": "warning", "time_s": round(elapsed, 2),
                                           "message": f"{end.title()} IR {side} detected (IO {int(io)})"})
                        ir_was_detected[end, side] = detected

                    if name_of_stream == "tof" and values:
                        distance = values[0]
                        invalid = distance == 0
                        if invalid and not tof_was_invalid and len(issues) < 80:
                            issues.append({"level": "warning", "time_s": round(elapsed, 2),
                                           "message": "ToF #0 returned 0 mm; scan skipped"})
                        tof_was_invalid = invalid
                        close = isinstance(distance, (int, float)) and 0 < distance < self.close_tof_mm
                        if close and not tof_was_close and len(issues) < 80:
                            issues.append({"level": "warning", "time_s": round(elapsed, 2),
                                           "message": f"ToF #0 below {self.close_tof_mm} mm ({distance:.0f} mm)"})
                        tof_was_close = close

                    sample = [row_number, elapsed, values]
                    if row_number in sample_indices:
                        samples.append(sample)

            streams[name_of_stream] = {
                "columns": columns,
                "samples": samples,
                "total_rows": row_count,
                "displayed_rows": len(samples),
                "gap_count": gap_count,
                "largest_gap_s": round(largest_gap, 3),
                "first_s": first_time,
                "last_s": last_time,
            }

        duration = max((item["last_s"] or 0 for item in streams.values()), default=0)
        return {"name": name, "summary": summary, "streams": streams,
                "duration_s": duration, "issues": issues[:80]}

    def csv_path(self, run_name, stream_name):
        run_path = self._run_path(run_name)
        if stream_name not in {file.stem for file in run_path.glob("*.csv") if file.is_file()}:
            raise ValueError("unknown stream")
        path = run_path / f"{stream_name}.csv"
        if path.is_symlink():
            raise ValueError("symlinked CSV is not allowed")
        return path


def make_server(store, host, port):
    """Create the local HTTP server; call serve_forever() to run it."""
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            parsed = urlsplit(self.path)
            query = parse_qs(parsed.query)
            try:
                if parsed.path == "/":
                    self._send("text/html; charset=utf-8", PAGE.read_bytes())
                elif parsed.path == "/api/runs":
                    self._json(store.list_runs())
                elif parsed.path == "/api/run":
                    self._json(store.load_run(query.get("name", [""])[0]))
                elif parsed.path == "/api/csv":
                    path = store.csv_path(query.get("run", [""])[0],
                                          query.get("stream", [""])[0])
                    self._send_file(path)
                else:
                    self.send_error(404)
            except ValueError as error:
                self.send_error(400, str(error))

        def _json(self, value):
            self._send("application/json; charset=utf-8",
                       json.dumps(value, ensure_ascii=False).encode("utf-8"))

        def _send(self, content_type, body, attachment=None):
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            if attachment:
                self.send_header("Content-Disposition", f'attachment; filename="{attachment}"')
            self.end_headers()
            self.wfile.write(body)

        def _send_file(self, path):
            self.send_response(200)
            self.send_header("Content-Type", "text/csv; charset=utf-8")
            self.send_header("Content-Length", str(path.stat().st_size))
            self.send_header("Content-Disposition", f'attachment; filename="{path.name}"')
            self.end_headers()
            with path.open("rb") as file:
                shutil.copyfileobj(file, self.wfile)

        def log_message(self, format, *args):
            pass

    server = ThreadingHTTPServer((host, port), Handler)
    server.daemon_threads = True
    return server
