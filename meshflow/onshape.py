import json
import os
import re
import subprocess
import sys
from pathlib import Path

from dotenv import load_dotenv


CONFIG_FILE = Path.home() / ".config" / "meshflow" / ".env"

_URL_PATTERN = re.compile(
    r"https://[^/]+/documents/(?P<documentId>[a-f0-9]+)/w/(?P<workspaceId>[a-f0-9]+)/e/(?P<elementId>[a-f0-9]+)",
    re.IGNORECASE,
)

_PLACEHOLDERS = {"your_access_key_here", "your_secret_key_here", ""}


def _die(msg: str) -> None:
    print(f"\n[ERROR] {msg}", file=sys.stderr)
    sys.exit(1)


def load_api_keys() -> tuple[str, str]:
    if not CONFIG_FILE.exists():
        _die("No config found. Run 'meshflow init' to set up your Onshape API keys.")

    load_dotenv(CONFIG_FILE)
    access_key = os.getenv("ONSHAPE_ACCESS_KEY", "").strip()
    secret_key = os.getenv("ONSHAPE_SECRET_KEY", "").strip()

    if access_key in _PLACEHOLDERS or secret_key in _PLACEHOLDERS:
        _die("API keys not set. Run 'meshflow init' to edit your config.")
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


_MESHFLOW_ROOT = Path(__file__).parent.parent

def run_conversion(target_dir: Path) -> None:
    cmd = ["uv", "run", "--project", str(_MESHFLOW_ROOT), "onshape-to-robot", str(target_dir)]
    print(f"\n  Running: uv run onshape-to-robot {target_dir}\n")

    result = subprocess.run(cmd, env=os.environ.copy(), capture_output=True, text=True)
    print(result.stdout, end="")
    if result.stderr:
        print(result.stderr, end="", file=sys.stderr)

    if result.returncode == 0:
        return

    combined = result.stdout + result.stderr
    if "KeyError: 'mass'" in combined or "ERROR: 'mass'" in combined:
        print(
            "\n[WARN] Some parts have no material/mass assigned in Onshape.\n"
            "       Retrying with dynamics disabled — inertial values set to 1e-9 placeholder.\n"
            "       Assign materials in Onshape and re-export for accurate simulation.\n"
        )
        config_path = target_dir / "config.json"
        config = json.loads(config_path.read_text())
        config["no_dynamics"] = True
        config_path.write_text(json.dumps(config, indent=4))

        result = subprocess.run(cmd, env=os.environ.copy())
        if result.returncode != 0:
            _die("onshape-to-robot failed even with dynamics disabled. Check terminal output.")
    else:
        _die("onshape-to-robot failed. Check terminal output.")
