#!/bin/sh
# Build submission.tar.gz with Makefile at the archive root.
# Lists top-level entries by name instead of tarring '.', whose explicit '.' member the grader rejects
# as an unsafe path (see spec/submission_layout.md).
set -e
cd "$(dirname "$0")"
rm -f submission.tar.gz
COPYFILE_DISABLE=1 tar czf submission.tar.gz \
  --exclude='__pycache__' --exclude='*.pyc' --exclude='.DS_Store' \
  Makefile README.md .gitignore src tests tools robots fsm
tar tzf submission.tar.gz | grep -qx 'Makefile' || { echo "Makefile not at archive root" >&2; exit 1; }
if tar tzf submission.tar.gz | grep -qE '^\./?$|(^|/)\.\.(/|$)'; then echo "unsafe path in archive" >&2; exit 1; fi
echo "wrote submission.tar.gz ($(tar tzf submission.tar.gz | wc -l | tr -d ' ') entries)"
