#!/usr/bin/env python3
"""Evaluate a SKiD-SLAM run against a dataset manifest.

    ./evaluation/run_evaluation.py evaluation/manifests/park.yaml \
        --results-root results/park

See evaluation/README.md for the file conventions.
"""

import argparse
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

from skid_eval.manifest import ManifestError, load_manifest  # noqa: E402
from skid_eval.report import render_json, render_text  # noqa: E402
from skid_eval.runner import evaluate  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("manifest", help="dataset manifest to evaluate")
    parser.add_argument("--results-root",
                        help="directory the manifest's relative paths resolve "
                             "against (default: the manifest's directory)")
    parser.add_argument("--json", action="store_true",
                        help="emit JSON instead of a text report")
    parser.add_argument("--output", help="write the report here instead of stdout")
    arguments = parser.parse_args(argv)

    try:
        manifest = load_manifest(arguments.manifest)
    except (ManifestError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    try:
        results = evaluate(manifest, arguments.results_root)
    except (ValueError, OSError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2

    text = render_json(results) if arguments.json else render_text(results)
    if arguments.output:
        pathlib.Path(arguments.output).write_text(text + "\n", encoding="utf-8")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
