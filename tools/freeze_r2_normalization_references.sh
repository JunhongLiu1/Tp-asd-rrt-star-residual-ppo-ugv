#!/usr/bin/env bash
set -eo pipefail

SOURCE="${1:-}"
DEST="${HOME}/terrain_radiation_ws/config/final_score_references_r2_frozen.json"

if [ -z "${SOURCE}" ] || [ ! -f "${SOURCE}" ]; then
    echo "Usage: $0 /absolute/path/to/normalization_references_candidate.json" >&2
    exit 1
fi

mkdir -p "$(dirname "${DEST}")"
python3 - "${SOURCE}" "${DEST}" <<'PY'
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

source = Path(sys.argv[1]).resolve()
destination = Path(sys.argv[2]).resolve()
data = json.loads(source.read_text(encoding='utf-8'))
data['status'] = 'frozen_for_formal_experiments'
data['frozen_at_utc'] = datetime.now(timezone.utc).isoformat()
data['source_candidate'] = str(source)
destination.write_text(json.dumps(data, indent=2), encoding='utf-8')
print('Frozen references:', destination)
PY
