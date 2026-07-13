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
        _die(
            "Could not parse Onshape URL.\n\n"
            "  Expected format:\n"
            "    https://cad.onshape.com/documents/<docId>/w/<workspaceId>/e/<elementId>\n\n"
            "  Common issues:\n"
            "    - URL is a version link (/v/) — switch to workspace (/w/) in Onshape\n"
            "    - URL was copied before the page fully loaded\n"
            "    - Extra query parameters were included — paste the URL up to the element ID only"
        )
    return match.groupdict()


def build_config(ids: dict, robot_name: str, output_format: str) -> dict:
    return {
        "documentId":    ids["documentId"],
        "workspaceId":   ids["workspaceId"],
        "elementId":     ids["elementId"],
        "robotName":     robot_name,
        "output_format": output_format,
    }


_MESHFLOW_ROOT = Path(__file__).parent.parent


def _failure_hint(output: str) -> str:
    low = output.lower()
    if "401" in output or "unauthorized" in low:
        return (
            "\n\n  Looks like an authentication failure (HTTP 401).\n"
            "  Your Onshape API keys may be wrong or expired.\n"
            "  Run 'meshflow init' to update them."
        )
    if "403" in output or "forbidden" in low:
        return (
            "\n\n  Access denied (HTTP 403).\n"
            "  The document may be private. Make sure you have access to it in Onshape,\n"
            "  or check that your API keys belong to the account that owns the document."
        )
    if "404" in output or "not found" in low:
        return (
            "\n\n  Document or element not found (HTTP 404).\n"
            "  The URL may point to a deleted document, or the assembly was moved.\n"
            "  Double-check the URL in Onshape and try again."
        )
    if "connection" in low or "timeout" in low or "network" in low or "name resolution" in low:
        return (
            "\n\n  Network error — could not reach cad.onshape.com.\n"
            "  Check your internet connection and try again."
        )
    if "no assembly found" in low or "not an assembly" in low or "assembly" in low and "type" in low:
        return (
            "\n\n  No assembly found at the given URL.\n"
            "  Make sure you copied the URL from an Assembly tab — not a Part Studio or Drawing.\n"
            "  In Onshape, click the Assembly tab at the bottom, then copy the URL."
        )
    return (
        "\n  Check the output above for details.\n"
        "  If the URL was copied from a Part Studio or Drawing, switch to the Assembly tab and retry."
    )


def run_conversion(target_dir: Path) -> None:
    cmd = ["uv", "run", "--project", str(_MESHFLOW_ROOT), "onshape-to-robot", str(target_dir)]
    print(f"\n  Running: uv run onshape-to-robot {target_dir}\n")

    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"   # force line-by-line flush through the pipe
    proc = subprocess.Popen(
        cmd, env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    output_lines: list[str] = []
    for line in proc.stdout:
        print(line, end="", flush=True)
        output_lines.append(line)
    proc.wait()
    combined = "".join(output_lines)

    if proc.returncode == 0:
        return

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
        _hint = _failure_hint(combined)
        _die(f"onshape-to-robot failed.{_hint}")
