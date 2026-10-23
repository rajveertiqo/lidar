"""Who may change the device — FastAPI dependencies for the guarded endpoints."""

import hmac
import ipaddress
import socket
from urllib.parse import urlsplit

from fastapi import Header, HTTPException

import config

TOKEN_HEADER = "X-Admin-Token"


def require_admin(x_admin_token: str = Header(default="")) -> None:
    """Allow the request only with the right admin token."""
    token = config.ADMIN_TOKEN
    if not token:
        raise HTTPException(
            status_code=503,
            detail="no admin token is configured on this device, so "
            "changes to how it steers are refused — start it "
            "with ./start_lidar.sh or the service, which set "
            "SHAPPE_ADMIN_TOKEN from ~/.config/shappe-guide.env "
            "(and add one there if it has none)",
        )
    if not hmac.compare_digest(x_admin_token.encode("utf-8"), token.encode("utf-8")):
        raise HTTPException(
            status_code=401,
            detail=f"this changes how the device steers — "
            f"send the admin token in {TOKEN_HEADER}",
        )


# ---- Cross-site requests -------------------------------------------------------

UNSAFE_METHODS = frozenset({"POST", "PUT", "PATCH", "DELETE"})

ALLOWED_HOSTS_ENV = "SHAPPE_ALLOWED_HOSTS"


def host_name(host: str) -> str:
    """The name part of a Host header, lower-case: "192.168.0.50:8000" -> "192.168.0.50", "[fe80::1]:8000" -> "fe80::1", "Shappe.local." -> "shappe.local"."""
    try:
        name = urlsplit("//" + (host or "").strip()).hostname or ""
    except ValueError:  # "[not closed", a bad IPv6 literal
        return ""
    return name.rstrip(".")


def parse_allowed_hosts(text: str) -> frozenset:
    """SHAPPE_ALLOWED_HOSTS's value -> the set of names it lists."""
    return frozenset(
        n for n in (host_name(part) for part in (text or "").split(",")) if n
    )


# config.ALLOWED_HOSTS_TEXT is SHAPPE_ALLOWED_HOSTS's value.
ALLOWED_HOSTS = parse_allowed_hosts(getattr(config, "ALLOWED_HOSTS_TEXT", ""))


def _host_is_this_device(name: str) -> bool:
    """Is `name` one this device is reached by, as opposed to a name someone else's DNS points at it?"""
    if not name:
        return False
    try:
        ipaddress.ip_address(name)
        return True
    except ValueError:
        pass
    me = socket.gethostname().strip().lower().rstrip(".")
    return name in {"localhost", me, me + ".local"} or name in ALLOWED_HOSTS


def _page_refusal(what: str, origin: str, host: str, allowed_origins) -> str:
    """The rule shared by HTTP writes and WebSocket handshakes, for a request that carries an Origin (i.e."""
    name = host_name(host)
    if not _host_is_this_device(name):
        return (
            f"{what} addressed to host {name or host!r} refused: that is not a "
            f"name this device answers to, which is what a DNS-rebinding page "
            f"looks like. Open the device by its IP address, localhost or its "
            f"own host name (.local), or add the name to {ALLOWED_HOSTS_ENV} "
            f"(comma-separated) in ~/.config/shappe-guide.env"
        )
    if origin in allowed_origins:
        return ""
    try:
        netloc = urlsplit(origin).netloc.lower()
    except ValueError:
        netloc = ""
    if netloc and host and netloc == host.strip().lower():
        return ""
    return f"cross-site {what} from {origin!r} refused"


def cross_site_refusal(method: str, origin: str, host: str, allowed_origins=()) -> str:
    """Why a request must be refused as cross-site, or "" to allow it."""
    if method.upper() not in UNSAFE_METHODS or not origin:
        return ""
    return _page_refusal(method.upper(), origin, host, allowed_origins)


def websocket_refusal(origin: str, host: str, allowed_origins=()) -> str:
    """The same rule for a WebSocket handshake, or "" to allow it."""
    if not origin:
        return ""
    return _page_refusal("WebSocket", origin, host, allowed_origins)
