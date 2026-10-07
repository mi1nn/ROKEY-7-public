"""Local read-only report export: python -m rescue_operations DB [--report-id UUID]."""

import argparse
import json
import sys

from .ledger import OperationsError, read_reports


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("database")
    parser.add_argument("--report-id")
    parser.add_argument("--limit", type=int, default=100)
    args = parser.parse_args(argv)
    try:
        result = read_reports(args.database, limit=args.limit, report_id=args.report_id)
    except OperationsError as exc:
        print(str(exc), file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
