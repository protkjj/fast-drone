"""Keep a compact, auditable record without committing multi-MB flight traces."""
import argparse
import hashlib
import json
from pathlib import Path


def summarize(source):
    raw = source.read_bytes()
    reports = json.loads(raw)["reports"]
    if len(reports) != 6:
        raise ValueError("Wait for all six paired step cases before publishing a summary")
    result = {"schema": 1, "raw_log_sha256": hashlib.sha256(raw).hexdigest(),
              "raw_log_filename": source.name, "reports": []}
    for report in reports:
        entry = {key: report[key] for key in (
            "configuration", "profile_id", "implementation", "solver", "simulated_seconds",
            "wall_seconds", "failure", "counts", "metrics", "final")}
        entry["failed_solves"] = [dict(solve, control_tick=index, time_s=index*.02)
                                  for index, solve in enumerate(report["solves"])
                                  if not solve["success"]]
        result["reports"].append(entry)
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(summarize(args.source), indent=2, ensure_ascii=False)+"\n")
