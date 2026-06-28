import os
import re
import subprocess
import sys
from pathlib import Path

from dotenv import load_dotenv


_URL_PATTERN = re.compile(
    r"https://[^/]+/documents/(?P<documentId>[a-f0-9]+)/w/(?P<workspaceId>[a-f0-9]+)/e/(?P<elementId>[a-f0-9]+)",
    re.IGNORECASE,
)


def _die(msg: str) -> None:
    print(f"\n[ERROR] {msg}", file=sys.stderr)
    sys.exit(1)


def load_api_keys() -> tuple[str, str]:
    env_file = Path(".env")
    if not env_file.exists():
        _die("No .env file found. Copy .env.example to .env and add keys.")

    load_dotenv(env_file)
    access_key = os.getenv("ONSHAPE_ACCESS_KEY", "").strip()
    secret_key = os.getenv("ONSHAPE_SECRET_KEY", "").strip()

    if not access_key or not secret_key:
        _die("Missing ONSHAPE_ACCESS_KEY or ONSHAPE_SECRET_KEY in .env.")
    return access_key, secret_key


def parse_onshape_url(url: str) -> dict:
    match = _URL_PATTERN.search(url.strip())
    if not match:
        _die("Could not parse Onshape URL.")
    return match.groupdict()


def build_config(ids: dict, robot_name: str, assembly_name: str, output_format: str) -> dict:
    return {
        "documentId":    ids["documentId"],
        "workspaceId":   ids["workspaceId"],
        "elementId":     ids["elementId"],
        "assemblyName":  assembly_name,
        "robotName":     robot_name,
        "output_format": output_format,
    }


def run_conversion(target_dir: Path) -> None:
    cmd = ["uv", "run", "onshape-to-robot", str(target_dir)]
    print(f"\n  Running: {' '.join(cmd)}\n")
    result = subprocess.run(cmd, env=os.environ.copy())
    if result.returncode != 0:
        _die("onshape-to-robot failed. Check terminal output.")
