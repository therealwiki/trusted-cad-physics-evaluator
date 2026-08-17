from __future__ import annotations

import hashlib
import json
from pathlib import Path
from typing import Any


def canonical_contract_bytes(contract: dict[str, Any]) -> bytes:
    """Stable bytes used to lock evaluator settings before candidate attempts."""
    return json.dumps(contract, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()


def load_contract(path: str | Path) -> dict[str, Any]:
    with Path(path).open(encoding="utf-8") as stream:
        contract = json.load(stream)
    if not contract.get("frozen_before_attempts"):
        raise ValueError("recorded optimization requires a frozen contract")
    return contract


def contract_sha256(contract: dict[str, Any]) -> str:
    return hashlib.sha256(canonical_contract_bytes(contract)).hexdigest()
